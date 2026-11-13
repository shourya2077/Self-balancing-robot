#pragma once
// ============================================================================
//  BalanceController.h
//
//  Control structure (all cascaded, runs at 200 Hz):
//
//   joystick / hold / return-home
//          |  desired speed v  (through ramp up / ramp down limiter)
//          v
//   [Speed PID]  error = v - measured speed      -> target lean angle (deg)
//          v
//   [Balance PD] error = angle - target lean     -> wheel ACCELERATION
//          v                                        (integrated -> wheel speed)
//   wheel speed +/- steering  ->  StepperMotors
//
//  Every loop gain is "individual value x master value" (Settings::gains()).
//  Units: mm, mm/s, mm/s^2, degrees (angles), rad/s (turn rate inside).
//
//  Extra safety / setup helpers
//   * IMU setup (capture upright / capture forward) finds the sensor mounting
//   * "balance point" button stores the trim
//   * tilt test: wheels simply follow the tilt (bench check, wheels in the air)
//   * runaway cut-off: wheels pinned at their speed limit for ~0.8 s -> motors off
//   * gyro offset is re-measured while the robot is held still in ARMED state
// ============================================================================
#include <stdarg.h>
#include "Config.h"
#include "Utils.h"
#include "Mpu6500.h"
#include "ImuCalibration.h"
#include "StepperMotors.h"

enum class RobotState : uint8_t {
  STOPPED = 0, ARMED = 1, BALANCING = 2, FALLEN = 3, FAULT = 4, TILT_TEST = 5, RUNAWAY = 6
};
enum class NavPhase : uint8_t { IDLE = 0, TURN_TO_TARGET = 1, DRIVE = 2, FINAL_TURN = 3 };

struct Telemetry {
  float      angle = 0, rate = 0, speed = 0, x = 0, y = 0, heading = 0, tilt = 0;
  float      wheel = 0, accel = 0;                         // commanded wheel speed / acceleration
  float      ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;  // raw sensor axes (g, deg/s)
  RobotState state = RobotState::STOPPED;
  NavPhase   nav   = NavPhase::IDLE;
  bool       hold  = false;
  char       ori[32] = "";
  float      dtMax = 0;          // longest control-loop period in the last second of balancing (ms)
  uint32_t   late  = 0;          // loops that were more than 8 ms late (while balancing)
  float      lim   = 0;          // top wheel speed possible right now (mm/s)
  float      trimAuto = 0;       // balance-point correction learned while balancing (deg)
  bool       yawGyro  = false;   // gyro yaw sign has been confirmed by a real turn
  float      resp  = 9;          // wheel response: +1 = axle accelerates as commanded, ~0 = wheels
                                 // are not delivering (skipped steps), < 0 = motors reversed, 9 = n/a
  char       fall[128] = "";      // why the last fall / cut-off happened
  int        tune  = -1;         // auto-tune progress 0..100, -1 = not running
  float      wobHz = 0, wobDeg = 0;   // size and frequency of the wobble in the last 2 s of balancing
};

// Gains for a robot whose centre of mass is Lmm above the axle (inverted pendulum with wheel
// acceleration as the input:  theta'' = (g*theta - a) / L ).  Pole placement: natural frequency
// wn = 1.7*sqrt(g/L) (limited to 6..12 rad/s), damping 1.4.  Kp = g + L*wn^2, Kd = 2*zeta*wn*L.
// (Found by simulating robots from 6 to 35 cm with sensor lag, late loops, motor lag and vibration:
//  this gives the widest range of master-gain values that stay calm.)
// The speed / position loops are kept about 5x slower than the balance loop.
struct GainDesign { float balKp, balKd, spdKp, spdKi, spdKd, posKp, wn; };
static inline GainDesign designGains(float Lmm) {
  const float g = 9810.0f;
  const float L = clampf(Lmm, 40.0f, 600.0f);
  float wn = 1.7f * sqrtf(g / L);
  wn = clampf(wn, 6.0f, 12.0f);
  const float kp = g + L * wn * wn;          // mm/s^2 per rad
  const float kd = 2.0f * 1.4f * wn * L;     // mm/s^2 per (rad/s)
  const float ws = wn / 5.0f;                // speed-loop bandwidth, rad/s
  GainDesign d;
  d.balKp = kp * DEG_TO_RAD;                 // per degree
  d.balKd = kd * DEG_TO_RAD;                 // per deg/s
  d.spdKp = ws / 171.2f;                     // deg of lean per mm/s (g*pi/180 = 171 mm/s^2 per deg)
  d.spdKi = d.spdKp * 0.06f * ws / 2.9f;
  d.spdKd = d.spdKp * 0.12f;
  d.posKp = 0.4f * ws / 2.9f;
  d.wn = wn;
  return d;
}

class BalanceController {
 public:
  BalanceController(Settings& s, Mpu6500& imu, StepperMotors& motors)
      : _s(s), _imu(imu), _motors(motors) {}

  // ----------------------------------------------------------------- setup
  void begin() {
    refreshGeometry();
    _lastUs = micros();
    if (!_imu.begin()) {
      Serial.printf("MPU not found (WHO_AM_I=0x%02X)\n", _imu.whoAmI());
      enterFault();
      return;
    }
    Serial.printf("MPU found (WHO_AM_I=0x%02X)\n", _imu.whoAmI());
    _imu.setOrientation(Orientation::fromSettings(_s));
    refreshOri();
    for (int i = 0; i < 3; i++) {
      if (_imu.calibrateGyro()) { Serial.println("Gyro calibrated"); break; }
      Serial.println("Gyro calibration failed - keep the robot still (it is re-done when you press START)");
    }
  }

  // ------------------------------------------------- commands (thread-safe)
  void cmdStart()      { _cmdStart = true; }
  void cmdStop()       { _cmdStop = true; }
  void cmdCalibrate()  { _cmdCalib = true; }
  void cmdMarkHome()   { _cmdMark = true; }
  void cmdReturnHome() { _cmdReturn = true; }
  void cmdMotorTest()  { _cmdTest = true; }
  void cmdCalUpright() { _cmdCalUp = true; }
  void cmdCalForward() { _cmdCalFwd = true; }
  void cmdSetBalance() { _cmdSetBal = true; }
  void cmdTiltTest()   { _cmdTilt = true; }
  void cmdZeroImu()    { _cmdZero = true; }
  void cmdKeepTrim()   { _cmdKeepTrim = true; }
  void cmdAutoTune()   { _cmdTune = true; }
  void cmdUndoTune()   { _cmdUntune = true; }
  void cmdHeightGains(){ _cmdHeight = true; }
  void setJoystick(float x, float y) {
    if (!isfinite(x) || !isfinite(y)) { x = 0; y = 0; }   // a NaN would poison the whole speed chain
    _joyX = x; _joyY = y; _joyMs = millis();
  }

  Telemetry   telemetry() const { return _tel; }
  RobotState  state() const { return _state; }
  const char* message() const { return _msgBuf; }
  uint32_t    messageSeq() const { return _msgSeq; }
  void        notify(const char* m) { snprintf(_msgBuf, sizeof(_msgBuf), "%s", m); _msgSeq = _msgSeq + 1; }
  void notifyf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(_msgBuf, sizeof(_msgBuf), fmt, ap);
    va_end(ap);
    _msgSeq = _msgSeq + 1;
  }

  // --------------------------------------------- called every Hw::LOOP_MS
  void update() {
    const uint32_t now = micros();
    const float dtRaw = (now - _lastUs) * 1e-6f;
    float dt = dtRaw;
    _lastUs = now;
    if (dt < 0.001f || dt > 0.03f) dt = Hw::LOOP_S;

    // loop health (shown on the dashboard): longest period per second, count of late loops
    if (_state == RobotState::BALANCING) {
      if (dtRaw > _dtMaxAcc) _dtMaxAcc = dtRaw;
      if (dtRaw > 0.008f) _tel.late = _tel.late + 1;
      _healthT += dtRaw;
      if (_healthT >= 1.0f) { _tel.dtMax = _dtMaxAcc * 1000.0f; _dtMaxAcc = 0; _healthT = 0; }
    }

    // hardware settings from the dashboard take effect right away (geometry only while not balancing)
    _motors.setInvert(_s.invLeft > 0.5f, _s.invRight > 0.5f);
    if (_state != RobotState::BALANCING && _state != RobotState::TILT_TEST) refreshGeometry();
    _tel.lim = wheelLimit();

    if (_state == RobotState::FAULT) {
      clearCommands();            // buttons pressed during an IMU fault must not fire later by themselves
      recoverImu();
      return;
    }

    syncOrientation();
    // the accelerometer also feels the wheels' own acceleration: take the part we know out of it.
    // axle acceleration seen by the sensor = (1-k)*a_wheel + k*g*theta  (k = about 0.3 for any mounting height)
    float agAid = 0.0f;
    if (_state == RobotState::BALANCING)
      agAid = ((1.0f - ACC_KAPPA) * _aLP + ACC_KAPPA * 9810.0f * _imu.angle() * DEG_TO_RAD) / 9810.0f;
    if (!_imu.update(dt, _s.alpha, agAid)) {
      if (++_imuFails >= 10) { enterFault(); notify("IMU lost - motors off"); }
      return;
    }
    _imuFails = 0;

    const float angle = _imu.angle() - _s.angleOffset - _trimAuto;
    const float rate  = _imu.rate();

    handleCommands();   // may block ~0.5 s while capturing poses (only when stopped)
    updateOdometry(dt);

    switch (_state) {
      case RobotState::STOPPED:
      case RobotState::FALLEN:
      case RobotState::RUNAWAY:   idle(); break;
      case RobotState::ARMED:     armed(angle, rate, dt); break;
      case RobotState::BALANCING: balance(angle, rate, dt); break;
      case RobotState::TILT_TEST: tiltTest(angle, dt); break;
      default: break;
    }
    if (_state != RobotState::BALANCING) _accelCmd = 0;

    _tel.angle = angle;
    _tel.rate = rate;
    _tel.speed = _vMeas;
    _tel.x = _x;
    _tel.y = _y;
    _tel.heading = _psi * RAD_TO_DEG;
    _tel.tilt = _tiltCmd;
    _tel.wheel = _vCmd;
    _tel.accel = _accelCmd;
    _tel.ax = _imu.acc(0); _tel.ay = _imu.acc(1); _tel.az = _imu.acc(2);
    _tel.gx = _imu.gyr(0); _tel.gy = _imu.gyr(1); _tel.gz = _imu.gyr(2);
    _tel.state = _state;
    _tel.nav = _returning ? _phase : NavPhase::IDLE;
    _tel.hold = _holdCaptured;
    _tel.trimAuto = _trimAuto;
    _tel.yawGyro = _yawConfirmed && _s.yawAssist > 0.5f;
  }

 private:
  Settings&      _s;
  Mpu6500&       _imu;
  StepperMotors& _motors;

  // controllers
  PID         _anglePid, _speedPid;
  RampLimiter _fwdRamp, _turnRamp;
  LowPass     _vFilter;

  // state
  RobotState _state = RobotState::STOPPED;
  NavPhase   _phase = NavPhase::IDLE;
  Telemetry  _tel;
  uint32_t   _lastUs = 0;
  int        _imuFails = 0;
  uint32_t   _faultRetryMs = 0;
  float      _uprightS = 0, _armRateSum = 0, _armedT = 0;
  int        _armN = 0;
  float      _stepsPerMm = 7.7f;

  // balance variables
  float _vCmd = 0;      // wheel speed command (mm/s)
  float _vMeas = 0;     // filtered speed estimate (mm/s)
  float _tiltCmd = 0;   // target lean from speed loop (deg)
  float _accelCmd = 0;  // wheel acceleration command (mm/s^2)
  float _satS = 0;      // time the wheels have been pinned at their limit
  // "why did it fall" bookkeeping: fraction of the last ~0.3 s each condition was true
  float _accSatF = 0, _whlSatF = 0;
  // wheel response: slope of (axle acceleration seen by the accelerometer) vs (commanded wheel acceleration)
  float _corrXY = 0, _corrXX = 0;
  // loop health
  float _dtMaxAcc = 0, _healthT = 0;
  // automatic balance trim: slow correction of the balance point (deg), learned from steady drift
  float _trimAuto = 0, _vSlow = 0, _trimStart = 0;
  // gyro yaw assist: sign learned from how the gyro reacts to commanded turning
  bool  _yawConfirmed = false;   // gyro sign checked against a real turn
  float _wApplied = 0, _yawWinGyro = 0;
  float _yawSign = 1.0f, _yawXY = 0, _yawXX = 0, _yawT = 0, _yawI = 0, _armYawSum = 0;
  float _yawGain = 1.0f;   // how much real yaw rate one unit of commanded yaw rate gives (wheel base / wheel size errors)
  // navigation helpers
  float _holdIdleS = 0, _returnT = 0, _nearT = 0;
  float _runT = 0;           // seconds since balancing started (soft start)
  float _stuckT = 0;
  float _aLP = 0;            // wheel acceleration actually commanded, lightly filtered (mm/s^2)
  static constexpr float ACC_KAPPA = 0.3f;
  float _vRampPrev = 0, _ffLP = 0;   // previous ramped speed and filtered lean feed-forward
  // wobble monitor (2 s windows)
  float _wobT = 0, _wobMax = 0; int _wobZc = 0; bool _wobSide = false;
  // ---- auto-tune: measure the robot, then set the gains
  enum Tune : uint8_t { T_OFF = 0, T_SETTLE = 1, T_EXCITE = 2, T_VERIFY = 3 };
  Tune     _tn = T_OFF;
  float    _tnT = 0, _tnMaxA = 0;
  uint32_t _lfsr = 0xACE1u;
  int      _tnHold = 0;
  float    _dither = 0;
  float    _tnPrevRate = 0, _tnPrevTh = 0, _tnA[3] = {0, 0, 0};
  bool     _tnHave = false;
  int      _tnN = 0;
  float    _Sxx[4][4], _Sx[4], _Sxy[4], _Sy, _Syy;
  bool     _havePrev = false;
  float    _prevG[6], _prevAcc = 0, _prevM[3] = {1, 1, 1}, _prevH = 0;
  bool     _tuneAtStart = false;
  // flight recorder: last 3 s of balancing at 100 Hz, frozen when the robot falls
 public:
  struct Rec { int16_t ang, rate, vCmd, acc, tilt, vMeas; };   // deg*100, dps*10, mm/s, mm/s2/4, deg*100, mm/s
  static constexpr int REC_N = 300;
  int  recCount() const { return _recFull ? REC_N : _recN; }
  Rec  recAt(int i) const { const int start = _recFull ? _recN : 0; return _rec[(start + i) % REC_N]; }
  bool recFrozen() const { return _recFrozen; }
 private:
  Rec  _rec[REC_N];
  int  _recN = 0; bool _recFull = false, _recFrozen = false; uint8_t _recDiv = 0;

  // odometry / navigation
  int32_t _prevL = 0, _prevR = 0;
  float   _x = 0, _y = 0, _psi = 0;          // mm, mm, rad (CCW positive)
  float   _homeX = 0, _homeY = 0, _homeHdg = 0;
  float   _anchorX = 0, _anchorY = 0, _hdgTarget = 0;
  bool    _holdCaptured = false, _hdgCaptured = false, _returning = false, _prevHold = false;
  uint32_t _testEnd = 0, _tiltStartMs = 0;

  // IMU setup
  float _calUp[3] = {0, 0, 1};
  bool  _haveUp = false;

  // inputs
  volatile float    _joyX = 0, _joyY = 0;
  volatile uint32_t _joyMs = 0;
  volatile bool _cmdStart = false, _cmdStop = false, _cmdCalib = false;
  volatile bool _cmdMark = false, _cmdReturn = false, _cmdTest = false;
  volatile bool _cmdCalUp = false, _cmdCalFwd = false, _cmdSetBal = false, _cmdTilt = false;
  volatile bool _cmdZero = false, _cmdKeepTrim = false, _cmdTune = false, _cmdUntune = false, _cmdHeight = false;

  char              _msgBuf[160] = "";
  volatile uint32_t _msgSeq = 0;

  // ------------------------------------------------------------- helpers
  void refreshGeometry() {
    _stepsPerMm = (float)(Hw::FULL_STEPS_PER_REV * _s.micro()) / (PI * _s.wheelDia);
  }

  void refreshOri() { _imu.orientation().describe(_tel.ori, sizeof(_tel.ori)); }

  // settings are the source of truth for the mounting
  void syncOrientation() {
    const Orientation o = Orientation::fromSettings(_s);
    if (!(o == _imu.orientation())) {
      _imu.setOrientation(o);
      refreshOri();
    }
  }

  // wheel speed limit while balancing (mm/s)
  float wheelLimit() const { return fminf(0.95f * Hw::MAX_STEP_RATE / _stepsPerMm, _s.maxWheel); }
  // the joystick may only use part of it: balancing needs headroom
  float joyLimit() const { return fminf(_s.maxSpeed, 0.7f * wheelLimit()); }

  static float shape(float v) {  // deadband + rescale
    const float db = 0.08f;
    float a = fabsf(v);
    if (a < db) return 0.0f;
    a = (a - db) / (1.0f - db);
    return v < 0 ? -a : a;
  }

  void shutdown(RobotState next) {
    if (_state == RobotState::BALANCING) _recFrozen = true;
    _tn = T_OFF; _dither = 0; _tel.tune = -1;
    _motors.setSpeed(0, 0);
    _motors.enable(false);
    _vCmd = 0;
    _satS = 0;
    _returning = false;
    _phase = NavPhase::IDLE;
    _state = next;
    _tel.state = next;
  }

  void enterFault() {
    shutdown(RobotState::FAULT);
    clearCommands();
    _faultRetryMs = millis();
  }

  void clearCommands() {
    _cmdStart = _cmdStop = _cmdCalib = _cmdMark = _cmdReturn = _cmdTest = false;
    _cmdCalUp = _cmdCalFwd = _cmdSetBal = _cmdTilt = false;
    _cmdZero = _cmdKeepTrim = _cmdTune = _cmdUntune = _cmdHeight = false;
  }

  void recoverImu() {
    if (millis() - _faultRetryMs < 1000) return;
    _faultRetryMs = millis();
    _imu.busClear();
    if (_imu.begin() && _imu.calibrateGyro()) {
      _imuFails = 0;
      _state = RobotState::STOPPED;
      notify("IMU OK");
    }
    _tel.state = _state;
  }

  // average the angle (without trim) over ~0.5 s; false if the robot moved
  bool captureAngle(float& out) {
    float sum = 0, lo = 1e9f, hi = -1e9f;
    int   n = 0;
    for (int i = 0; i < 100; i++) {
      delay(5);
      if (_imu.update(0.005f, _s.alpha)) {
        const float a = _imu.angle();
        sum += a;
        if (a < lo) lo = a;
        if (a > hi) hi = a;
        n++;
      }
    }
    if (n < 50 || (hi - lo) > 2.0f) return false;
    out = sum / n;
    return true;
  }

  void applyOrientation(const Orientation& o) {
    _s.set("imuUpAxis", (float)o.upAxis);
    _s.set("imuUpSign", o.upSign);
    _s.set("imuFwdAxis", (float)o.fwdAxis);
    _s.set("imuFwdSign", o.fwdSign);
    _s.set("imuGyroAxis", (float)o.gyroAxis);
    _s.set("imuGyroSign", o.gyroSign);
    const float a0 = o.accAngleDeg(_calUp);     // where the upright pose reads
    _s.set("angleOffset", a0);
    const char* keys[] = {"imuUpAxis", "imuUpSign", "imuFwdAxis", "imuFwdSign",
                          "imuGyroAxis", "imuGyroSign", "angleOffset"};
    for (const char* k : keys) _s.saveKey(k);
    _imu.setOrientation(o);
    refreshOri();
    _yawConfirmed = false;
    _yawSign = 1.0f;
    _yawGain = 1.0f;
    _trimAuto = 0;
    _haveUp = false;
    notifyf("IMU setup saved: %s. Now run Tilt test", _tel.ori);
  }

  void handleCommands() {
    if (_cmdStop) {
      _cmdStop = false;
      _testEnd = 0;
      shutdown(RobotState::STOPPED);
      notify("Stopped");
    }
    const bool resting = (_state == RobotState::STOPPED || _state == RobotState::FALLEN ||
                          _state == RobotState::RUNAWAY);
    if (_cmdStart) {
      _cmdStart = false;
      if (resting) {
        _state = RobotState::ARMED;
        _uprightS = 0;
        _armRateSum = 0;
        _armYawSum = 0;
        _armN = 0;
        _armedT = 0;
        notify("Armed - hold robot upright and still");
      } else if (_state == RobotState::TILT_TEST) {
        notify("Press STOP first to leave the tilt test");
      }
    }
    if (_cmdCalib) {
      _cmdCalib = false;
      if (resting) notify(_imu.calibrateGyro() ? "Gyro calibrated" : "Calibration failed - keep still");
      else notify("Stop the robot first");
    }
    if (_cmdTest) {
      _cmdTest = false;
      if (resting) {
        refreshGeometry();
        _motors.enable(true);
        _testEnd = millis() + 1200;
        notify("Motor test: both wheels should roll FORWARD");
      }
    }
    if (_cmdCalUp) {
      _cmdCalUp = false;
      if (!resting) {
        notify("Stop the robot first");
      } else {
        float a[3];
        if (!_imu.readAccelAverage(a)) {
          notify("Hold the robot still and try again");
        } else if (const char* e = ImuCalibrator::checkUpright(a)) {
          notify(e);
        } else {
          for (int k = 0; k < 3; k++) _calUp[k] = a[k];
          _haveUp = true;
          notify("Upright captured. Now tilt forward ~25 deg, hold still, press step 2");
        }
      }
    }
    if (_cmdCalFwd) {
      _cmdCalFwd = false;
      if (!resting) {
        notify("Stop the robot first");
      } else if (!_haveUp) {
        notify("Do step 1 first (hold upright, press Capture upright)");
      } else {
        float t[3];
        Orientation o;
        if (!_imu.readAccelAverage(t)) notify("Hold the robot still and try again");
        else if (const char* e = ImuCalibrator::solve(_calUp, t, o)) notify(e);
        else applyOrientation(o);
      }
    }
    if (_cmdSetBal) {
      _cmdSetBal = false;
      float a = 0;
      if (!resting) {
        notify("Stop the robot first");
      } else if (!captureAngle(a)) {
        notify("Hold the robot still at its balance point and try again");
      } else {
        _s.set("angleOffset", a);
        _trimAuto = 0;
        _s.saveKey("angleOffset");
        notifyf("Balance point saved (sensor reads %.1f deg there)", a);
      }
    }
    if (_cmdZero) {
      _cmdZero = false;
      float a = 0;
      if (!resting) {
        notify("Stop the robot first");
      } else if (!_imu.calibrateGyro()) {
        notify("Hold the robot still - gyro calibration failed");
      } else if (!captureAngle(a)) {
        notify("Hold the robot still at its balance point and try again");
      } else if (fabsf(a) > 15.0f) {
        notify("Hold the robot closer to upright (within 15 deg) and try again");
      } else {
        _s.set("angleOffset", a);
        _trimAuto = 0;
        _s.saveKey("angleOffset");
        notifyf("IMU zeroed: gyro offset + balance point (%.1f deg)", a);
      }
    }
    if (_cmdKeepTrim) {
      _cmdKeepTrim = false;
      const float nv = _s.angleOffset + _trimAuto;
      if (fabsf(nv) > 15.0f) {
        notify("Trim out of range - redo Set balance point");
      } else {
        _s.set("angleOffset", nv);
        _trimAuto = 0;                       // the effective balance point does not change
        if (resting) { _s.saveKey("angleOffset"); notifyf("Auto-trim kept: balance trim %.2f deg (saved)", nv); }
        else         notifyf("Auto-trim kept: balance trim %.2f deg - Stop, then Save to flash", nv);
      }
    }
    if (_cmdTune) {
      _cmdTune = false;
      if (_state != RobotState::BALANCING) notify("Press START and let it balance first, then Auto-tune");
      else startTune();
    }
    if (_cmdUntune) {
      _cmdUntune = false;
      if (_havePrev) { restoreGains(); _havePrev = false; _tn = T_OFF; _dither = 0; _tel.tune = -1; notify("Gains from before the Auto-tune restored"); }
      else notify("Nothing to undo");
    }
    if (_cmdHeight) {
      _cmdHeight = false;
      if (_s.comH < 40.0f) notify("Set 'Robot height' first (mm from the axle up to the centre of mass)");
      else { savePrevGains(); applyDesign(_s.comH); notifyf("Gains set for a %.0f mm robot (not saved yet)", _s.comH); }
    }
    if (_cmdTilt) {
      _cmdTilt = false;
      if (resting) {
        refreshGeometry();
        _fwdRamp.reset();
        _vCmd = 0;
        _tiltStartMs = millis();
        _motors.enable(true);
        _state = RobotState::TILT_TEST;
        notify("Tilt test: tilt forward = wheels roll forward. Press STOP to end");
      } else {
        notify("Stop the robot first");
      }
    }
    if (_cmdMark) {
      _cmdMark = false;
      _homeX = _x; _homeY = _y; _homeHdg = _psi;
      notify("Home position saved");
    }
    if (_cmdReturn) {
      _cmdReturn = false;
      if (_state == RobotState::BALANCING) {
        _returning = true;
        _returnT = 0;
        _phase = NavPhase::TURN_TO_TARGET;
        _holdCaptured = false;
        _hdgCaptured = false;
        notify("Returning home");
      }
    }
  }

  // ---------------------------------------------------------------- auto-tune
  void savePrevGains() {
    _prevG[0] = _s.balKp; _prevG[1] = _s.balKd; _prevG[2] = _s.spdKp; _prevG[3] = _s.spdKi;
    _prevG[4] = _s.spdKd; _prevG[5] = _s.posKp; _prevAcc = _s.maxAccel;
    _prevM[0] = _s.mKp; _prevM[1] = _s.mKi; _prevM[2] = _s.mKd; _prevH = _s.comH;
    _havePrev = true;
  }
  void restoreGains() {
    _s.set("balKp", _prevG[0]); _s.set("balKd", _prevG[1]); _s.set("spdKp", _prevG[2]);
    _s.set("spdKi", _prevG[3]); _s.set("spdKd", _prevG[4]); _s.set("posKp", _prevG[5]);
    _s.set("maxAccel", _prevAcc); _s.set("mKp", _prevM[0]); _s.set("mKi", _prevM[1]);
    _s.set("mKd", _prevM[2]); _s.set("comH", _prevH);
  }
  void applyDesign(float L) {
    const GainDesign d = designGains(L);
    _s.set("balKp", d.balKp); _s.set("balKd", d.balKd);
    _s.set("spdKp", d.spdKp); _s.set("spdKi", d.spdKi); _s.set("spdKd", d.spdKd);
    _s.set("posKp", d.posKp);
    _s.set("mKp", 1.0f); _s.set("mKi", 1.0f); _s.set("mKd", 1.0f);
    if (_s.maxAccel < 4000.0f) _s.set("maxAccel", 4000.0f);
  }
  void startTune() {
    _tn = T_SETTLE; _tnT = 0; _tnHold = 0; _dither = 0; _tel.tune = 0;
    notify("Auto-tune started: keep the stick centred and let it balance (about 15 s)");
  }
  void abortTune(const char* why) {
    _tn = T_OFF; _dither = 0; _tel.tune = -1; notify(why);
  }
  static bool solveN(double A[4][5], double w[4]) {   // Gauss elimination with pivoting, 4 unknowns
    for (int c = 0; c < 4; c++) {
      int p = c;
      for (int r = c + 1; r < 4; r++) if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
      if (fabs(A[p][c]) < 1e-30) return false;
      if (p != c) for (int k = 0; k < 5; k++) { const double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; }
      for (int r = c + 1; r < 4; r++) {
        const double f = A[r][c] / A[c][c];
        for (int k = c; k < 5; k++) A[r][k] -= f * A[c][k];
      }
    }
    for (int r = 3; r >= 0; r--) {
      double s = A[r][4];
      for (int k = r + 1; k < 4; k++) s -= A[r][k] * w[k];
      w[r] = s / A[r][r];
    }
    return true;
  }
  // Before the accel is integrated: phase logic and the test signal (small random +-accel steps)
  void tunePre(float angle, float rate, float dt, bool joyActive) {
    if (_tn == T_OFF) { _dither = 0; return; }
    _tnT += dt;
    if (joyActive || _returning) { abortTune("Auto-tune cancelled (stick moved)"); return; }
    switch (_tn) {
      case T_SETTLE:
        _dither = 0;
        _tel.tune = (int)clampf(_tnT * 2.0f, 0.0f, 9.0f);
        if (fabsf(angle) < 2.5f && fabsf(rate) < 25.0f) _tnHold++; else _tnHold = 0;
        if (_tnHold > 200) {
          _tn = T_EXCITE; _tnT = 0; _tnHave = false; _tnN = 0; _tnMaxA = 0;
          for (int i = 0; i < 4; i++) { _Sx[i] = _Sxy[i] = 0; for (int j = 0; j < 4; j++) _Sxx[i][j] = 0; }
          _Sy = _Syy = 0; _tnA[0] = _tnA[1] = _tnA[2] = 0; _tnHold = 0;
        } else if (_tnT > 10.0f) {
          abortTune("Auto-tune: it never stood still - hold it upright for a moment, then try again");
        }
        break;
      case T_EXCITE:
        _tnMaxA = fmaxf(_tnMaxA, fabsf(angle));
        if (fabsf(angle) > 10.0f) { abortTune("Auto-tune stopped: it leaned too far. Try again"); return; }
        if (++_tnHold >= 16) {                       // new random level every 80 ms
          _tnHold = 0;
          const uint32_t bit = ((_lfsr >> 0) ^ (_lfsr >> 2) ^ (_lfsr >> 3) ^ (_lfsr >> 5)) & 1u;
          _lfsr = (_lfsr >> 1) | (bit << 15);
          _dither = (_lfsr & 1u) ? 400.0f : -400.0f;
        }
        _tel.tune = 10 + (int)clampf(_tnT / 12.0f * 80.0f, 0.0f, 80.0f);
        if (_tnT >= 12.0f) tuneFinish();
        break;
      case T_VERIFY:
        _dither = 0;
        _tnMaxA = fmaxf(_tnMaxA, fabsf(angle));
        _tel.tune = 90 + (int)clampf(_tnT / 3.0f * 10.0f, 0.0f, 9.0f);
        if (_tnT >= 3.0f) {
          if (_tnMaxA > 12.0f) {
            restoreGains();
            abortTune("Auto-tune: the new gains were not calmer - old gains restored");
          } else {
            _tn = T_OFF; _tel.tune = 100;
            notifyf("Auto-tune done: height %.0f mm, balance Kp %.0f Kd %.1f. Press STOP, then Save to flash",
                    _s.comH, _s.balKp, _s.balKd);
          }
        }
        break;
      default: break;
    }
  }
  // After the loop output is known: collect  d(rate)/dt = alpha*angle + beta1*a[k-1] + beta2*a[k-2] + beta3*a[k-3]
  void tunePost(float angle, float rate, float dt, float aCmd) {
    if (_tn != T_EXCITE) return;
    const float r = rate * DEG_TO_RAD;
    if (_tnHave) {
      const float y = (r - _tnPrevRate) / dt;                  // rad/s^2
      const float x[4] = {_tnPrevTh, _tnA[0] * 1e-3f, _tnA[1] * 1e-3f, _tnA[2] * 1e-3f};   // rad, m/s^2
      for (int i = 0; i < 4; i++) {
        _Sx[i] += x[i];
        _Sxy[i] += x[i] * y;
        for (int j = 0; j < 4; j++) _Sxx[i][j] += x[i] * x[j];
      }
      _Sy += y; _Syy += y * y; _tnN++;
    }
    _tnPrevRate = r; _tnPrevTh = angle * DEG_TO_RAD;
    _tnA[2] = _tnA[1]; _tnA[1] = _tnA[0]; _tnA[0] = aCmd;
    _tnHave = true;
  }
  void tuneFinish() {
    const double N = _tnN;
    if (N < 1500) { abortTune("Auto-tune: not enough data (loop too slow?)"); return; }
    double C[4][4], rv[4], A[4][5], w[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) C[i][j] = _Sxx[i][j] - (double)_Sx[i] * _Sx[j] / N;
      rv[i] = _Sxy[i] - (double)_Sx[i] * _Sy / N;
    }
    const double syy = _Syy - (double)_Sy * _Sy / N;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) A[i][j] = C[i][j];
      A[i][i] += 1e-4 * C[i][i] + 1e-12;       // tiny ridge: the three delayed inputs are almost the same signal
      A[i][4] = rv[i];
    }
    if (!solveN(A, w) || syy <= 0) { abortTune("Auto-tune failed (no usable data)"); return; }
    double ss = syy;
    for (int i = 0; i < 4; i++) { ss -= 2.0 * w[i] * rv[i]; for (int j = 0; j < 4; j++) ss += w[i] * C[i][j] * w[j]; }
    const double r2 = 1.0 - ss / syy;
    const double alpha = w[0], beta = w[1] + w[2] + w[3];
    if (r2 < 0.25) { notifyf("Auto-tune failed: measurement too noisy (fit %.0f%%) - check wheels slipping / vibration", r2 * 100); _tn = T_OFF; _dither = 0; _tel.tune = -1; return; }
    if (beta > -1.5 || beta < -30.0) {
      notifyf("Auto-tune failed: result not plausible (beta %.1f). Check motor direction and IMU setup", beta);
      _tn = T_OFF; _dither = 0; _tel.tune = -1; return;
    }
    const float L = (float)(-1000.0 / beta);               // mm
    const double ratio = alpha / (-9.81 * beta);           // should be about 1 for a real pendulum
    savePrevGains();
    applyDesign(L);
    _s.set("comH", L);
    _tn = T_VERIFY; _tnT = 0; _tnMaxA = 0; _dither = 0;
    notifyf("Auto-tune: robot height %.0f mm (fit %.0f%%%s). New gains on - checking", L, r2 * 100,
            (ratio < 0.4 || ratio > 2.5) ? ", gravity term odd" : "");
  }

  void updateOdometry(float dt) {
    const int32_t cl = _motors.stepsLeft(), cr = _motors.stepsRight();
    const float dl = (cl - _prevL) / _stepsPerMm;
    const float dr = (cr - _prevR) / _stepsPerMm;
    _prevL = cl;
    _prevR = cr;
    const float ds = 0.5f * (dl + dr);
    // heading: from the gyro once it has been checked against the wheels (it sees the REAL
    // rotation, also when a wheel slips or the wheel base setting is a bit off); else from the wheels
    float dpsi = (dr - dl) / _s.wheelBase;
    if (_s.yawAssist > 0.5f) dpsi = _yawSign * _imu.yawRate() * DEG_TO_RAD * dt;
    const float mid = _psi + 0.5f * dpsi;
    _x += ds * cosf(mid);
    _y += ds * sinf(mid);
    _psi += dpsi;
  }

  // ---------------------------------------------------------------- states
  void idle() {
    if (_testEnd != 0) {
      if ((int32_t)(millis() - _testEnd) < 0) {
        const float v = 150.0f * _stepsPerMm;
        _motors.setSpeed(v, v);
        return;
      }
      _testEnd = 0;
    }
    _motors.setSpeed(0, 0);
    _motors.enable(false);
  }

  // Bench check with the wheels in the air: wheel speed simply follows the tilt.
  // Tilt forward -> both wheels roll forward, tilt back -> backward, level -> stop.
  void tiltTest(float angle, float dt) {
    if (millis() - _tiltStartMs > 120000UL) {
      shutdown(RobotState::STOPPED);
      notify("Tilt test ended (2 minutes)");
      return;
    }
    float tgt = 0;
    if (angle > 1.0f) tgt = (angle - 1.0f) * 25.0f;
    else if (angle < -1.0f) tgt = (angle + 1.0f) * 25.0f;
    const float lim = fminf(300.0f, wheelLimit());
    tgt = clampf(tgt, -lim, lim);
    _vCmd = _fwdRamp.update(tgt, dt, 1500.0f, 1500.0f);
    _motors.setSpeed(_vCmd * _stepsPerMm, _vCmd * _stepsPerMm);
  }

  // Held upright and still for 1 s -> start. The same second is used to re-measure
  // the gyro offset (a wrong offset makes the wheels creep / run away).
  void armed(float angle, float rate, float dt) {
    _motors.setSpeed(0, 0);
    _motors.enable(false);
    _armedT += dt;
    if (_armedT > 30.0f) { shutdown(RobotState::STOPPED); notify("Arming timed out (30 s) - press START again"); return; }
    if (fabsf(angle) < 6.0f && fabsf(rate) < 15.0f) {
      _uprightS += dt;
      _armRateSum += rate;
      _armYawSum += _imu.yawRate();
      _armN++;
    } else {
      _uprightS = 0;
      _armRateSum = 0;
      _armYawSum = 0;
      _armN = 0;
    }
    if (_uprightS > 1.0f && _armN > 0) {
      const float mean = _armRateSum / _armN;
      if (fabsf(mean) < 6.0f) _imu.adjustBias(mean);
      const float yawMean = _armYawSum / _armN;
      if (fabsf(yawMean) < 2.0f) _imu.adjustYawBias(yawMean);
      engage();
    }
  }

  void engage() {
    refreshGeometry();
    _motors.resetCounts();
    _prevL = _prevR = 0;
    _x = _y = _psi = 0;
    _homeX = _homeY = _homeHdg = 0;
    _anglePid.reset();
    _speedPid.reset();
    _fwdRamp.reset();
    _turnRamp.reset();
    _vFilter.reset();
    _vCmd = _vMeas = _tiltCmd = _accelCmd = 0;
    _satS = 0;
    _accSatF = _whlSatF = 0;
    _corrXY = _corrXX = 0;
    _tel.resp = 9.0f;
    _dtMaxAcc = _healthT = 0;
    _tel.dtMax = 0;
    _tel.late = 0;
    _vSlow = 0;
    _runT = 0; _vRampPrev = 0; _ffLP = 0; _stuckT = 0; _aLP = 0;
    _wobT = _wobMax = 0; _wobZc = 0; _wobSide = false; _tel.wobHz = _tel.wobDeg = 0;
    _recN = 0; _recFull = false; _recFrozen = false; _recDiv = 0;
    _tn = T_OFF; _dither = 0; _tel.tune = -1;
    _tuneAtStart = _s.tuneOnStart > 0.5f;
    _tel.fall[0] = 0;
    _trimStart = _trimAuto;
    _yawI = _yawXY = _yawXX = _yawT = _yawWinGyro = _wApplied = 0;
    _holdIdleS = _returnT = _nearT = 0;
    _holdCaptured = _hdgCaptured = _returning = false;
    _phase = NavPhase::IDLE;
    _prevHold = _s.holdPos > 0.5f;
    _motors.enable(true);
    _state = RobotState::BALANCING;
    notify("Balancing");
  }

  // desired speed that brings 'err' (mm) to zero without overshooting the decel ramp
  float positionProfile(float err, float vmax) const {
    const float mag = fabsf(err);
    float v = _s.posKp * _s.mKp * mag;
    v = fminf(v, sqrtf(2.0f * _s.rampDown * 0.7f * mag));
    v = fminf(v, vmax);
    return err >= 0 ? v : -v;
  }

  // same idea for heading (rad -> rad/s)
  float turnProfile(float err, float wmax) const {
    const float mag = fabsf(err);
    float w = _s.hdgKp * mag;
    w = fminf(w, sqrtf(2.0f * _s.turnRamp * DEG_TO_RAD * 0.7f * mag));
    w = fminf(w, wmax);
    return err >= 0 ? w : -w;
  }

  float alongError(float tx, float ty) const {
    return (tx - _x) * cosf(_psi) + (ty - _y) * sinf(_psi);
  }

  void navigate(float& vDes, float& wDes) {
    const float maxW = _s.maxTurn * DEG_TO_RAD;
    const float dx = _homeX - _x, dy = _homeY - _y;
    const float dist = sqrtf(dx * dx + dy * dy);
    _returnT += Hw::LOOP_S;
    if (_returnT > 90.0f) {                  // never drive around forever
      _returning = false;
      _phase = NavPhase::IDLE;
      notify("Return home gave up after 90 s");
      return;
    }

    switch (_phase) {
      case NavPhase::TURN_TO_TARGET: {
        if (dist < 25.0f) { _phase = NavPhase::FINAL_TURN; break; }
        const float err = wrapPi(atan2f(dy, dx) - _psi);
        wDes = turnProfile(err, maxW);
        if (fabsf(err) < 5.0f * DEG_TO_RAD && fabsf(_turnRamp.value()) < 0.3f) _phase = NavPhase::DRIVE;
        break;
      }
      case NavPhase::DRIVE: {
        const float err = wrapPi(atan2f(dy, dx) - _psi);
        vDes = positionProfile(alongError(_homeX, _homeY), fminf(_s.returnSpeed, joyLimit()));
        wDes = (dist < 40.0f) ? 0.0f : turnProfile(err, maxW);   // the bearing is meaningless right at the target
        if (dist < 45.0f && fabsf(_vMeas) < 80.0f) _nearT += Hw::LOOP_S; else _nearT = 0;
        if ((dist < 20.0f && fabsf(_vMeas) < 60.0f) || _nearT > 1.5f) _phase = NavPhase::FINAL_TURN;   // close enough and settled
        else if (dist > 80.0f && fabsf(err) > 35.0f * DEG_TO_RAD) _phase = NavPhase::TURN_TO_TARGET;
        break;
      }
      case NavPhase::FINAL_TURN: {
        const float err = wrapPi(_homeHdg - _psi);
        wDes = turnProfile(err, maxW);
        if (fabsf(err) < 3.0f * DEG_TO_RAD && fabsf(_turnRamp.value()) < 0.1f) {
          _returning = false;
          _phase = NavPhase::IDLE;
          _anchorX = _x; _anchorY = _y; _holdCaptured = true;   // keep position if hold is on
          _hdgTarget = _homeHdg; _hdgCaptured = true;
          notify("Arrived home");
        }
        break;
      }
      default:
        _returning = false;
        break;
    }
  }

  void balance(float angle, float rate, float dt) {
    if (fabsf(angle) > _s.fallAngle) {
      const bool wasVerify = (_tn == T_VERIFY && _havePrev);
      if (wasVerify) restoreGains();
      const bool fwd = angle > 0;
      float wHz = _tel.wobHz, wDeg = _tel.wobDeg;
      if (_wobT > 0.6f) { wHz = _wobZc / 2.0f / _wobT; wDeg = fmaxf(wDeg, _wobMax); }
      shutdown(RobotState::FALLEN);
      const char* why;
      char wob[80];
      const float resp = _tel.resp;
      wob[0] = 0;
      if (wDeg > 2.5f && wHz >= 0.7f) {
        if (wHz >= 3.5f) snprintf(wob, sizeof(wob), "shaking at %.1f Hz: gains too high or vibration - lower Master or Auto-tune", wHz);
        else             snprintf(wob, sizeof(wob), "swaying at %.1f Hz and growing: gains don't fit - run Auto-tune", wHz);
      }
      if      (resp < -0.25f)                                  why = "wheels pushed the WRONG way - use Reverse left/right motor";

      else if (resp < 0.35f)                                   why = "wheels did not follow the commands (skipping steps?) - lower accel limit";
      else if (wob[0])                                         why = wob;
      else if (_whlSatF > 0.3f)                                why = "wheels hit the Max motor speed limit - raise it or lower the gains";
      else if (_accSatF > 0.5f)                                why = "wheel acceleration limit reached - raise Wheel accel limit";
      else                                                     why = "it leaned faster than the wheels reacted - try a higher Master Kp/Kd";
      snprintf(_tel.fall, sizeof(_tel.fall), "Fell %s: %s", fwd ? "forward" : "backward", why);
      if (wasVerify) notifyf("%s (Auto-tune gains undone)", _tel.fall);
      else           notify(_tel.fall);
      return;
    }

    _runT += dt;
    // ---------- wobble monitor (size and frequency of the sway, shown on the dashboard)
    _wobT += dt;
    _wobMax = fmaxf(_wobMax, fabsf(angle));
    if (!_wobSide && angle > 0.3f) { _wobSide = true; _wobZc++; }
    else if (_wobSide && angle < -0.3f) { _wobSide = false; _wobZc++; }
    if (_wobT >= 2.0f) {
      _tel.wobHz = _wobZc / 2.0f / _wobT;
      _tel.wobDeg = _wobMax;
      _wobT = 0; _wobZc = 0; _wobMax = 0;
    }
    // ---------- flight recorder (100 Hz, last 3 s, frozen at a fall)
    if (!_recFrozen && (++_recDiv & 1) == 0) {
      Rec& q = _rec[_recN];
      q.ang = (int16_t)clampf(angle * 100.0f, -32000.0f, 32000.0f);
      q.rate = (int16_t)clampf(rate * 10.0f, -32000.0f, 32000.0f);
      q.vCmd = (int16_t)clampf(_vCmd, -32000.0f, 32000.0f);
      q.acc = (int16_t)clampf(_accelCmd * 0.25f, -32000.0f, 32000.0f);
      q.tilt = (int16_t)clampf(_tiltCmd * 100.0f, -32000.0f, 32000.0f);
      q.vMeas = (int16_t)clampf(_vMeas, -32000.0f, 32000.0f);
      if (++_recN >= REC_N) { _recN = 0; _recFull = true; }
    }

    const float vLim = wheelLimit();
    const float maxJoy = joyLimit();

    // ---------- operator input (with failsafe)
    float jx = 0, jy = 0;
    if (millis() - _joyMs < Hw::JOY_TIMEOUT_MS) {
      jx = shape(_joyX);
      jy = shape(_joyY);
      if (_s.invDrive > 0.5f) jy = -jy;
      if (_s.invSteer > 0.5f) jx = -jx;
    }
    const bool joyActive = (jx != 0.0f || jy != 0.0f);
    if (_returning && joyActive) {
      _returning = false;
      _phase = NavPhase::IDLE;
      notify("Return cancelled");
    }

    if (_tuneAtStart && _runT > 1.5f && _tn == T_OFF) { _tuneAtStart = false; startTune(); }
    tunePre(angle, rate, dt, joyActive);

    const float maxW = _s.maxTurn * DEG_TO_RAD;
    const bool  hold = _s.holdPos > 0.5f;
    if (hold && !_prevHold) {   // switching Hold ON also saves this spot as "home"
      _homeX = _x; _homeY = _y; _homeHdg = _psi;
      notify("Hold ON - spot saved as home");
    }
    _prevHold = hold;
    float vDes = 0, wDes = 0;

    if (_returning) {
      navigate(vDes, wDes);
    } else {
      vDes = jy * maxJoy;
      wDes = -jx * maxW;                       // stick right = turn clockwise

      // ----- position hold
      if (jy != 0.0f || !hold) {
        _holdCaptured = false;
        _holdIdleS = 0;
      } else {
        if (!_holdCaptured) {
          _holdIdleS += dt;   // a drifting robot never gets below 25 mm/s: do not wait for ever
          if (fabsf(_fwdRamp.value()) < 15.0f && (fabsf(_vMeas) < 25.0f || _holdIdleS > 1.5f)) {
            _anchorX = _x; _anchorY = _y; _holdCaptured = true;
          }
        }
        if (_holdCaptured) vDes = positionProfile(alongError(_anchorX, _anchorY), maxJoy);
      }

      // ----- heading hold
      if (jx != 0.0f || !hold) {
        _hdgCaptured = false;
      } else {
        if (!_hdgCaptured && fabsf(_turnRamp.value()) < 0.1f) { _hdgTarget = _psi; _hdgCaptured = true; }
        if (_hdgCaptured) wDes = turnProfile(wrapPi(_hdgTarget - _psi), maxW);
      }
    }

    // ---------- speed ramps (ramp up / ramp down)
    vDes = clampf(vDes, -maxJoy, maxJoy);
    const float v = _fwdRamp.update(vDes, dt, _s.rampUp, _s.rampDown);
    const float w = _turnRamp.update(wDes, dt, _s.turnRamp * DEG_TO_RAD, _s.turnRamp * DEG_TO_RAD);

    // ---------- outer loop: speed PI -> target lean
    // Wheel speed is commanded relative to the body, so the speed over the ground is
    // vCmd + r * (pitch rate). Without the second term the speed loop sees a phase error
    // and the robot oscillates slowly (B-Robot / BalancingWii both include it).
    _vMeas = _vFilter.update(_vCmd + 0.5f * _s.wheelDia * rate * DEG_TO_RAD, dt, 0.05f);
    // ---------- automatic balance trim (BalancingWii / Brokking do the same):
    // with the stick centred the robot must not drift. If it does, the balance point is
    // wrong; nudge it slowly so that the average speed goes to zero.
    _vSlow += (_vMeas - _vSlow) * dt / 0.7f;
    if (!joyActive && !_returning && fabsf(v) < 40.0f && _s.trimRate > 0.0f && _runT > 1.0f)
      _trimAuto = clampf(_trimAuto - _s.trimRate * _vSlow * dt, -6.0f, 6.0f);

    const Settings::Gains g = _s.gains();           // individual value x master value
    _speedPid.setGains(g.spdKp, g.spdKi, g.spdKd);  // small D = damping of the speed loop
    _speedPid.setOutputLimits(-_s.maxTilt, _s.maxTilt);
    _speedPid.setIntegralLimit(_s.maxTilt * 0.25f);   // limit I-term windup (BalancingWii uses 1/6)
    // feed-forward: to accelerate at 'a' the robot must lean a/g, so lean as the ramp asks for
    // instead of waiting for a speed error to build up (this is what made the joystick feel slow)
    const float ffRaw = (v - _vRampPrev) / dt / 9810.0f * RAD_TO_DEG * 0.5f;
    _vRampPrev = v;
    _ffLP += (clampf(ffRaw, -_s.maxTilt, _s.maxTilt) - _ffLP) * dt / 0.08f;
    // Soft start: while the robot is still held in the hand the wheels cannot make it lean, so the
    // speed loop would wind the wheels up (positive feedback) and the robot would lurch when released.
    // The speed loop is therefore faded in between 0.15 s and 0.6 s after balancing starts.
    const float sf = clampf((_runT - 0.15f) / 0.45f, 0.0f, 1.0f);
    if (sf <= 0.0f) _speedPid.reset();
    _tiltCmd = clampf((_speedPid.update(v - _vMeas, dt) + _ffLP) * sf, -_s.maxTilt, _s.maxTilt);
    // the robot is asked to lean far more than it does for a while = it is not responding (held in
    // the hand, or the wheels are blocked): do not let the wheels wind up
    if (fabsf(_tiltCmd - angle) > fmaxf(0.45f * _s.maxTilt, 3.0f)) _stuckT += dt;
    else _stuckT = fmaxf(0.0f, _stuckT - 2.0f * dt);
    if (_stuckT > 0.4f) {
      _stuckT = 0; _vCmd = 0; _aLP = 0; _vFilter.reset(); _speedPid.reset(); _runT = 0; _ffLP = 0;
      _trimAuto = _trimStart;      // the trim it 'learned' while blocked or held is not real
      notify("Wheels were winding up while it stood still (held in the hand?) - reset");
    }

    // ---------- inner loop: balance PD -> acceleration -> speed
    _anglePid.setGains(g.balKp, 0.0f, g.balKd);
    _anglePid.setOutputLimits(-_s.maxAccel, _s.maxAccel);   // wheel accel limit: protects against skipped steps
    const float accel = _anglePid.update(angle - _tiltCmd, rate, dt) + _dither;
    _accelCmd = accel;
    const float vPrev = _vCmd;
    const float vLimSoft = vLim;
    _vCmd = clampf(_vCmd + accel * dt, -vLimSoft, vLimSoft);
    const float aCmd = (_vCmd - vPrev) / dt;        // wheel acceleration actually commanded (after limits)
    _aLP += (aCmd - _aLP) * dt / 0.012f;

    // ---------- bookkeeping for the "why did it fall" message
    {
      const float k = dt / 0.3f;
      _accSatF += ((fabsf(accel) >= 0.98f * _s.maxAccel ? 1.0f : 0.0f) - _accSatF) * k;
      _whlSatF += ((fabsf(_vCmd) >= 0.92f * vLim ? 1.0f : 0.0f) - _whlSatF) * k;
      // does the axle really accelerate the way the wheels were told to? (checks motor direction + skipped steps)
      const float kc = dt / 0.5f;
      const float aMeas = _imu.axleAccel();
      _corrXY += (aCmd * aMeas - _corrXY) * kc;
      _corrXX += (aCmd * aCmd - _corrXX) * kc;
      _tel.resp = (_corrXX > 10000.0f) ? clampf(_corrXY / _corrXX, -3.0f, 3.0f) : 9.0f;   // needs > ~100 mm/s^2 rms
    }
    tunePost(angle, rate, dt, aCmd);

    // ---------- runaway cut-off: wheels pinned at their limit = not balancing
    if (fabsf(_vCmd) >= 0.92f * vLim) _satS += dt;
    else _satS = fmaxf(0.0f, _satS - dt);
    if (_satS > 0.8f) {
      float wHz = _tel.wobHz, wDeg = _tel.wobDeg;
      if (_wobT > 0.6f) { wHz = _wobZc / 2.0f / _wobT; wDeg = fmaxf(wDeg, _wobMax); }
      const bool wasVerify = (_tn == T_VERIFY && _havePrev);
      if (wasVerify) restoreGains();
      shutdown(RobotState::RUNAWAY);
      if (wDeg > 2.5f && wHz >= 0.7f)
        snprintf(_tel.fall, sizeof(_tel.fall), "Runaway after %s at %.1f Hz - %s", wHz >= 3.5f ? "shaking" : "swaying", wHz,
                 wHz >= 3.5f ? "lower Master Kp,Kd" : "run Auto-tune");
      else
        snprintf(_tel.fall, sizeof(_tel.fall), "Runaway: wheels pinned at Max motor speed - check IMU setup, balance trim, gains");
      notify(_tel.fall);
      return;
    }

    // ---------- gyro yaw assist
    // The gyro sees the REAL rotation (also when a wheel slips, the two sides differ a little or the
    // wheel base setting is off). The sign is assumed from the right-hand rule and checked while turning.
    const float yawRaw = _imu.yawRate() * DEG_TO_RAD;    // rad/s about the vertical axis, + = counter-clockwise
    const bool  assist = _s.yawAssist > 0.5f;
    if (assist && fabsf(_wApplied) > 0.25f) {            // learn from what was really commanded
      _yawXY += _wApplied * yawRaw * dt;
      _yawXX += _wApplied * _wApplied * dt;
      _yawWinGyro += yawRaw * dt;
      _yawT  += dt;
      if (_yawT > 0.5f) {
        const float slope = _yawXY / _yawXX;
        if (!_yawConfirmed && fabsf(slope) > 0.2f) {
          if (slope < 0) {                               // gyro sign was wrong: fix it and the heading it made
            _yawSign = -1.0f;
            _psi += -2.0f * _yawWinGyro;
          }
          _yawGain = clampf(fabsf(slope), 0.3f, 3.0f);
          _yawConfirmed = true;
        }
        _yawXY = _yawXX = _yawT = _yawWinGyro = 0;
      }
    } else if (_yawT > 0.0f && fabsf(_wApplied) <= 0.25f) {
      _yawXY = _yawXX = _yawT = _yawWinGyro = 0;
    }
    // a small PI on the yaw rate makes turning accurate (whatever the wheel base setting) and
    // keeps straight driving straight; the feed-forward uses the learned wheel-base error
    float wCmd = w;
    if (assist) {
      const float yawMeas = _yawSign * yawRaw;
      if (fabsf(_wApplied) > 0.4f && _wApplied * yawMeas > 0.0f)   // keep the gain estimate fresh while turning
        _yawGain += (clampf(yawMeas / _wApplied, 0.3f, 3.0f) - _yawGain) * dt / 1.0f;
      const float e = w - yawMeas;
      const float iLim = fmaxf(0.3f, 0.5f * maxW);
      _yawI = clampf(_yawI + 3.0f * e * dt, -iLim, iLim);
      _yawI -= _yawI * dt / 8.0f;                         // forget slowly
      wCmd = w / _yawGain + 0.5f * e + _yawI;
    } else {
      _yawI = 0;
    }
    _wApplied = wCmd;

    // ---------- steering + output
    float half = 0.5f * _s.wheelBase * wCmd;      // mm/s
    const float room = fmaxf(0.0f, vLim - fabsf(_vCmd));
    half = clampf(half, -room, room);             // steering only uses what balancing leaves free
    _motors.setSpeed((_vCmd - half) * _stepsPerMm, (_vCmd + half) * _stepsPerMm);
  }
};
