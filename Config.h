#pragma once
// ============================================================================
//  Config.h  -  pins, hardware constants and runtime-tunable settings
// ============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#define FW_VERSION "2.5"

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------- PINS ---------------------------------------
namespace Pins {
constexpr uint8_t I2C_SDA = 21;  // MPU6500 SDA
constexpr uint8_t I2C_SCL = 22;  // MPU6500 SCL
constexpr uint8_t STEP_L  = 26;  // left  DRV8825 STEP  (must be a GPIO < 32)
constexpr uint8_t DIR_L   = 27;  // left  DRV8825 DIR
constexpr uint8_t STEP_R  = 25;  // right DRV8825 STEP  (must be a GPIO < 32)
constexpr uint8_t DIR_R   = 33;  // right DRV8825 DIR
constexpr uint8_t ENABLE  = 13;  // EN of BOTH drivers (active LOW)
}  // namespace Pins

// ---------------------------- HARDWARE --------------------------------------
namespace Hw {
constexpr int      FULL_STEPS_PER_REV = 200;     // 1.8 deg motor
// Factory defaults only - microstepping and motor direction are set on the dashboard
// ("Motors & hardware") and stored in flash.
constexpr int      MICROSTEPS         = 8;       // CNC shield: caps on MS1+MS2 = 1/8
constexpr bool     INVERT_LEFT        = false;   // flip if a wheel turns the wrong way
constexpr bool     INVERT_RIGHT       = true;    // motors are mirrored -> one is inverted
constexpr uint32_t TICK_HZ            = 40000;   // step-generator ISR rate
constexpr float    MAX_STEP_RATE      = 10000.0f;// hard limit, steps/s per motor
constexpr uint32_t LOOP_MS            = 5;       // control loop = 200 Hz
constexpr float    LOOP_S             = 0.005f;
constexpr uint32_t JOY_TIMEOUT_MS     = 500;     // joystick failsafe
}  // namespace Hw

// ------------------------------ IMU -----------------------------------------
// These are only the FACTORY DEFAULTS for how the MPU board is mounted
// (board flat, Z up, X pointing to the front). The real mounting is detected
// from the dashboard ("IMU setup": capture upright, capture forward) and stored
// in flash, so you normally never edit these.
namespace Imu {
constexpr uint8_t I2C_ADDR     = 0x68;   // AD0 low = 0x68, AD0 high = 0x69
constexpr int     ACC_FWD_AXIS = 0;      // 0=X 1=Y 2=Z
constexpr float   ACC_FWD_SIGN = -1.0f;
constexpr int     ACC_UP_AXIS  = 2;
constexpr float   ACC_UP_SIGN  = +1.0f;
constexpr int     GYRO_AXIS    = 1;
constexpr float   GYRO_SIGN    = +1.0f;
}  // namespace Imu

// ------------------------------ WIFI ----------------------------------------
namespace Net {
constexpr const char* AP_SSID = "BalanceBot";
constexpr const char* AP_PASS = "balance123";  // at least 8 characters
}  // namespace Net

// ============================================================================
//  Settings: every value here can be changed live from the dashboard
// ============================================================================
struct ParamDef {
  const char* key;
  float*      ptr;
  float       def, lo, hi;
  bool        hw;   // hardware calibration: "Reset defaults" leaves it alone
};

class Settings {
 public:
  // MASTER PID: multiplies the matching gain of every loop below
  //   mKp -> balKp, spdKp, posKp      mKi -> spdKi      mKd -> balKd, spdKd
  float mKp, mKi, mKd;
  // inner balance loop (PD, output = wheel acceleration)
  float balKp, balKd;
  // outer speed loop (PID, output = target lean angle)
  float spdKp, spdKi, spdKd;
  // position / heading hold
  float posKp, hdgKp;
  // automatic balance-point trim (deg/s per mm/s of drift) and gyro yaw assist (0/1)
  float trimRate, yawAssist;
  // robot height (centre of mass above the axle, mm; 0 = not known yet) and tune-on-every-start flag
  float comH, tuneOnStart;
  // robot / filter
  float angleOffset, alpha, maxTilt, fallAngle, maxAccel;
  // driving
  float maxSpeed, maxWheel, maxTurn, rampUp, rampDown, turnRamp, returnSpeed;
  // geometry
  float wheelDia, wheelBase;
  // flags stored as float (0 / 1)
  float holdPos;
  // motors (hardware, stored in flash)
  float microsteps, invLeft, invRight, invDrive, invSteer;
  // IMU mounting (written by the dashboard calibration)
  float imuUpAxis, imuUpSign, imuFwdAxis, imuFwdSign, imuGyroAxis, imuGyroSign;

  Settings() {
    reg("mKp",         &mKp,           1.0f,    0.2f,    3.0f);
    reg("mKi",         &mKi,           1.0f,    0.0f,    5.0f);
    reg("mKd",         &mKd,           1.0f,    0.2f,    3.0f);
    reg("balKp",       &balKp,        573.0f,   0.0f, 3000.0f);
    reg("balKd",       &balKd,         93.8f,   0.0f,  300.0f);
    reg("spdKp",       &spdKp,         0.0140f,  0.0f,    0.05f);
    reg("spdKi",       &spdKi,         0.0007f,  0.0f,    0.05f);
    reg("spdKd",       &spdKd,         0.0017f,  0.0f,    0.02f);
    reg("posKp",       &posKp,         0.33f,    0.0f,    3.0f);
    reg("hdgKp",       &hdgKp,         3.0f,    0.0f,   10.0f);
    reg("trimRate",    &trimRate,      0.010f,  0.0f,    0.02f);
    reg("yawAssist",   &yawAssist,     1.0f,    0.0f,    1.0f);
    reg("comH",        &comH,          0.0f,    0.0f,  600.0f);
    reg("tuneOnStart", &tuneOnStart,   0.0f,    0.0f,    1.0f);
    reg("angleOffset", &angleOffset,   0.0f,  -15.0f,   15.0f, true);
    reg("alpha",       &alpha,         0.995f,   0.9f,    0.9999f);
    reg("maxTilt",     &maxTilt,       8.0f,    1.0f,   20.0f);
    reg("fallAngle",   &fallAngle,    35.0f,   15.0f,   60.0f);
    reg("maxAccel",    &maxAccel,   4000.0f,  500.0f, 10000.0f);
    reg("maxSpeed",    &maxSpeed,    400.0f,   50.0f, 1000.0f);
    reg("maxWheel",    &maxWheel,    800.0f,  100.0f, 1200.0f);
    reg("maxTurn",     &maxTurn,     120.0f,   10.0f,  360.0f);
    reg("rampUp",      &rampUp,      900.0f,   50.0f, 3000.0f);
    reg("rampDown",    &rampDown,   1200.0f,   50.0f, 4000.0f);
    reg("turnRamp",    &turnRamp,    600.0f,   50.0f, 3000.0f);
    reg("returnSpeed", &returnSpeed, 300.0f,   50.0f,  800.0f);
    reg("wheelDia",    &wheelDia,     66.0f,   20.0f,  200.0f, true);
    reg("wheelBase",   &wheelBase,   150.0f,   50.0f,  400.0f, true);
    reg("holdPos",     &holdPos,       0.0f,    0.0f,    1.0f);
    reg("microsteps",  &microsteps,   (float)Hw::MICROSTEPS, 1.0f, 32.0f, true);
    reg("invLeft",     &invLeft,      Hw::INVERT_LEFT  ? 1.0f : 0.0f, 0.0f, 1.0f, true);
    reg("invRight",    &invRight,     Hw::INVERT_RIGHT ? 1.0f : 0.0f, 0.0f, 1.0f, true);
    reg("invDrive",    &invDrive,     0.0f,    0.0f,    1.0f, true);
    reg("invSteer",    &invSteer,     0.0f,    0.0f,    1.0f, true);
    reg("imuUpAxis",   &imuUpAxis,   (float)Imu::ACC_UP_AXIS,  0.0f, 2.0f, true);
    reg("imuUpSign",   &imuUpSign,   Imu::ACC_UP_SIGN,        -1.0f, 1.0f, true);
    reg("imuFwdAxis",  &imuFwdAxis,  (float)Imu::ACC_FWD_AXIS, 0.0f, 2.0f, true);
    reg("imuFwdSign",  &imuFwdSign,  Imu::ACC_FWD_SIGN,       -1.0f, 1.0f, true);
    reg("imuGyroAxis", &imuGyroAxis, (float)Imu::GYRO_AXIS,    0.0f, 2.0f, true);
    reg("imuGyroSign", &imuGyroSign, Imu::GYRO_SIGN,          -1.0f, 1.0f, true);
  }
  Settings(const Settings&) = delete;
  Settings& operator=(const Settings&) = delete;

  // gains the controller really uses = individual value x master value
  struct Gains { float balKp, balKd, spdKp, spdKi, spdKd, posKp; };
  Gains gains() const {
    return { balKp * mKp, balKd * mKd, spdKp * mKp, spdKi * mKi, spdKd * mKd, posKp * mKp };
  }

  // DRV8825 microstepping is 1, 2, 4, 8, 16 or 32
  int micro() const {
    int m = 1;
    while (m < 32 && (float)(m * 2) <= microsteps * 1.2f) m *= 2;   // nearest power of two
    return m;
  }

  bool set(const char* key, float v) {
    if (isnan(v)) return false;
    for (int i = 0; i < _n; i++) {
      if (strcmp(_p[i].key, key) == 0) {
        *_p[i].ptr = clampf(v, _p[i].lo, _p[i].hi);
        if (strcmp(key, "microsteps") == 0) *_p[i].ptr = (float)micro();
        return true;
      }
    }
    return false;
  }

  bool get(const char* key, float& out) const {
    for (int i = 0; i < _n; i++) {
      if (strcmp(_p[i].key, key) == 0) { out = *_p[i].ptr; return true; }
    }
    return false;
  }

  // tuning values go back to defaults; IMU mounting, balance trim, motor setup and wheel size / base are kept
  void resetDefaults() {
    for (int i = 0; i < _n; i++)
      if (!_p[i].hw) *_p[i].ptr = _p[i].def;
  }

  void load() {
    Preferences pref;
    if (!pref.begin("bbot", true)) return;
    for (int i = 0; i < _n; i++)
    {
      const float v = pref.getFloat(_p[i].key, _p[i].def);
      *_p[i].ptr = isfinite(v) ? clampf(v, _p[i].lo, _p[i].hi) : _p[i].def;   // a NaN in flash must not get in
    }
    pref.end();
  }

  void save() {
    Preferences pref;
    if (!pref.begin("bbot", false)) return;
    for (int i = 0; i < _n; i++) pref.putFloat(_p[i].key, *_p[i].ptr);
    pref.end();
  }

  // store a single value (used right after a calibration)
  void saveKey(const char* key) {
    Preferences pref;
    if (!pref.begin("bbot", false)) return;
    for (int i = 0; i < _n; i++)
      if (strcmp(_p[i].key, key) == 0) pref.putFloat(key, *_p[i].ptr);
    pref.end();
  }

  // {"p":{"balKp":560,...}}
  size_t toJson(char* buf, size_t cap) const {
    size_t n = snprintf(buf, cap, "{\"p\":{");
    for (int i = 0; i < _n && n < cap; i++) {
      n += snprintf(buf + n, cap - n, "%s\"%s\":%.6g", i ? "," : "", _p[i].key, *_p[i].ptr);
    }
    if (n < cap) n += snprintf(buf + n, cap - n, "}}");
    if (n >= cap) n = cap ? cap - 1 : 0;      // truncated: report what is really in the buffer
    return n;
  }

 private:
  static constexpr int MAX_PARAMS = 48;
  ParamDef _p[MAX_PARAMS];
  int      _n = 0;

  void reg(const char* key, float* ptr, float def, float lo, float hi, bool hw = false) {
    if (_n >= MAX_PARAMS) { *ptr = def; Serial.println("Settings: too many parameters (raise MAX_PARAMS)"); return; }
    _p[_n++] = {key, ptr, def, lo, hi, hw};
    *ptr = def;
  }
};
