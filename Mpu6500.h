#pragma once
// ============================================================================
//  Mpu6500.h  -  raw-register MPU6500 driver + complementary filter
//  (no external library needed)
//
//  The mounting of the board (which axis is up / front) is an Orientation that
//  is set at run time, see ImuCalibration.h.
// ============================================================================
#include <Wire.h>
#include "Config.h"
#include "ImuCalibration.h"

class Mpu6500 {
 public:
  // Returns false if the chip does not answer / is not recognised
  bool begin() {
    _ready = false;
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(10);

    _whoAmI = readReg(0x75);
    if (_whoAmI != 0x70 && _whoAmI != 0x71 && _whoAmI != 0x73 && _whoAmI != 0x68) return false;

    writeReg(0x6B, 0x80);  // device reset
    delay(100);
    writeReg(0x6B, 0x01);  // wake, clock = PLL
    delay(10);
    writeReg(0x1A, 0x03);  // gyro DLPF ~41 Hz
    writeReg(0x19, 0x00);  // sample rate 1 kHz
    writeReg(0x1B, 0x08);  // gyro  +-500 dps (65.5 LSB/dps)
    writeReg(0x1C, 0x00);  // accel +-2 g     (16384 LSB/g)
    writeReg(0x1D, 0x03);  // accel DLPF ~44 Hz (MPU6500 register)
    _init = false;
    _ready = true;
    return true;
  }

  // Frees a stuck I2C bus (a slave holding SDA low after a glitch): 9 clock pulses, then release.
  void busClear() {
    Wire.end();
    pinMode(Pins::I2C_SDA, INPUT_PULLUP);
    pinMode(Pins::I2C_SCL, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(Pins::I2C_SCL, LOW);  delayMicroseconds(5);
      digitalWrite(Pins::I2C_SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(Pins::I2C_SCL, INPUT_PULLUP);
  }

  void setOrientation(const Orientation& o) {
    _o = o;
    _init = false;   // re-seed the angle from the accelerometer
  }
  const Orientation& orientation() const { return _o; }

  // Robot must stand still. Measures the zero-rate offset of all 3 gyro axes.
  bool calibrateGyro(uint16_t samples = 400) {
    if (!_ready) return false;
    double  sum[3] = {0, 0, 0};
    int16_t lo[3] = {32767, 32767, 32767}, hi[3] = {-32768, -32768, -32768};
    uint16_t ok = 0;
    int16_t  r[7];
    for (uint16_t i = 0; i < samples; i++) {
      if (readRaw(r)) {
        for (int k = 0; k < 3; k++) {
          const int16_t g = r[4 + k];
          sum[k] += g;
          if (g < lo[k]) lo[k] = g;
          if (g > hi[k]) hi[k] = g;
        }
        ok++;
      }
      delay(3);
    }
    if (ok < samples / 2) return false;
    for (int k = 0; k < 3; k++)
      if ((hi[k] - lo[k]) > 400) return false;  // moved (> ~6 dps spread)
    for (int k = 0; k < 3; k++) _bias[k] = (float)(sum[k] / ok);
    _init = false;
    return true;
  }

  // Average accelerometer reading (in g, sensor axes). false = moved / I2C error.
  bool readAccelAverage(float out[3], uint16_t samples = 100, float maxSpread = 0.08f) {
    if (!_ready) return false;
    double  sum[3] = {0, 0, 0};
    float   lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    uint16_t ok = 0;
    int16_t  r[7];
    for (uint16_t i = 0; i < samples; i++) {
      if (readRaw(r)) {
        for (int k = 0; k < 3; k++) {
          const float a = r[k] / 16384.0f;
          sum[k] += a;
          if (a < lo[k]) lo[k] = a;
          if (a > hi[k]) hi[k] = a;
        }
        ok++;
      }
      delay(4);
    }
    if (ok < samples / 2) return false;
    for (int k = 0; k < 3; k++) {
      if ((hi[k] - lo[k]) > maxSpread) return false;
      out[k] = (float)(sum[k] / ok);
    }
    return true;
  }

  // Shift the zero-rate offset so that a measured mean pitch rate becomes zero
  void adjustBias(float rateDps) { _bias[_o.gyroAxis] += rateDps * _o.gyroSign * 65.5f; }
  // same for the yaw gyro (rotation about the vertical axis)
  void adjustYawBias(float rateDps) { _bias[_o.upAxis] += rateDps * _o.upSign * 65.5f; }

  // Call at a fixed rate with the real dt. Returns false on I2C error.
  // ag = forward acceleration of the axle (in g) that the accelerometer is known to be seeing
  // (the wheels' own acceleration). It is taken out of the tilt reading; without this the
  // accelerometer reports the "apparent vertical" while the wheels accelerate, which hides the
  // real lean from the filter and lets the robot run away.
  bool update(float dt, float alpha, float ag = 0.0f) {
    if (!_ready) return false;
    int16_t r[7];
    if (!readRaw(r)) return false;

    for (int k = 0; k < 3; k++) {
      _acc[k] = r[k] / 16384.0f;
      _gyr[k] = ((float)r[4 + k] - _bias[k]) / 65.5f;
    }
    // plausibility: a standing or moving robot always feels about 1 g (all zeros / garbage = bad read)
    const float n2 = _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2];
    if (n2 < 0.09f || n2 > 16.0f) return false;
    float accAngle = _o.accAngleDeg(_acc);
    if (ag != 0.0f) {
      const float f0 = _o.fwdSign * _acc[_o.fwdAxis], u0 = _o.upSign * _acc[_o.upAxis];
      const float th = _angle * 0.017453293f;
      const float fc = f0 + ag * cosf(th), uc = u0 - ag * sinf(th);
      accAngle += (atan2f(fc, uc) - atan2f(f0, u0)) * 57.29578f;
    }
    _fwdG = _o.fwdSign * _acc[_o.fwdAxis];   // g, + = robot forward
    _upG  = _o.upSign * _acc[_o.upAxis];     // g, + = up
    _rate = _o.gyroSign * _gyr[_o.gyroAxis];  // deg/s, forward lean positive

    // Slow gyro-offset tracking (B-Robot does the same): over a long time the pitch rate of a
    // robot that is not tumbling averages to zero, so any steady rate is sensor offset.
    // The measured value is first limited to +-10 LSB (0.15 dps) around the current offset and then
    // followed with a time constant of about 20 s, so the offset can move by at most ~0.5 LSB/s.
    {
      const float raw = (float)r[4 + _o.gyroAxis];
      const float b   = _bias[_o.gyroAxis];
      _bias[_o.gyroAxis] = b + (dt / 20.0f) * (clampf(raw, b - 10.0f, b + 10.0f) - b);
    }

    if (!_init) {
      _angle = accAngle;
      _init = true;
    } else {
      _angle = alpha * (_angle + _rate * dt) + (1.0f - alpha) * accAngle;
    }
    return true;
  }

  float   angle() const { return _angle; }   // deg, forward lean positive
  float   rate() const { return _rate; }     // deg/s
  // Rotation about the vertical axis, deg/s, counter-clockwise seen from above (right-hand rule).
  // Only the axis is known for sure; the controller checks the sign against the wheels.
  float   yawRate() const { return _o.upSign * _gyr[_o.upAxis]; }
  // Horizontal acceleration of the axle (mm/s^2, + = forward) as the accelerometer sees it.
  // Static lean gives fwd = sin(th), up = cos(th); an axle acceleration a adds
  // fwd -= a/g*cos(th), up += a/g*sin(th), so  a = g*(sin(th)*up - cos(th)*fwd).
  float   axleAccel() const {
    const float th = _angle * 0.017453293f;
    return 9810.0f * (sinf(th) * _upG - cosf(th) * _fwdG);
  }
  float   acc(int axis) const { return _acc[axis]; }   // g, raw sensor axes
  float   gyr(int axis) const { return _gyr[axis]; }   // deg/s, raw sensor axes (bias removed)
  bool    ready() const { return _ready; }
  uint8_t whoAmI() const { return _whoAmI; }

 private:
  bool        _ready = false, _init = false;
  float       _angle = 0, _rate = 0;
  float       _bias[3] = {0, 0, 0};
  float       _acc[3] = {0, 0, 0}, _gyr[3] = {0, 0, 0};
  float       _fwdG = 0, _upG = 1;
  uint8_t     _whoAmI = 0;
  Orientation _o;

  uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(Imu::I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom((uint8_t)Imu::I2C_ADDR, (uint8_t)1) != 1) return 0xFF;
    return Wire.read();
  }

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(Imu::I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  // r[0..2]=accel XYZ, r[3]=temp, r[4..6]=gyro XYZ
  bool readRaw(int16_t* r) {
    Wire.beginTransmission(Imu::I2C_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)Imu::I2C_ADDR, (uint8_t)14) != 14) return false;
    for (int i = 0; i < 7; i++) {
      int hi = Wire.read();
      int lo = Wire.read();
      r[i] = (int16_t)((hi << 8) | lo);
    }
    return true;
  }
};
