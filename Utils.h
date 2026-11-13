#pragma once
// ============================================================================
//  Utils.h  -  small reusable control building blocks
// ============================================================================
#include "Config.h"
#include <math.h>

inline float wrapPi(float a) {
  if (!isfinite(a)) return 0.0f;          // (a NaN/inf here used to be an endless loop)
  a = fmodf(a, TWO_PI);
  if (a > PI) a -= TWO_PI;
  else if (a < -PI) a += TWO_PI;
  return a;
}

// ---------------------------------------------------------------------------
//  PID  (integral stored already multiplied by Ki -> live gain changes don't jump)
// ---------------------------------------------------------------------------
class PID {
 public:
  void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
  void setOutputLimits(float lo, float hi) { _lo = lo; _hi = hi; }
  void setIntegralLimit(float lim) { _iLim = lim; }   // max size of the I term itself
  void reset() { _i = 0; _prevErr = 0; _first = true; }

  // derivative computed from the error itself
  float update(float error, float dt) {
    float d = _first ? 0.0f : (error - _prevErr) / dt;
    _prevErr = error;
    _first = false;
    return update(error, d, dt);
  }

  // derivative supplied from outside (e.g. gyro rate)
  float update(float error, float dError, float dt) {
    _i = clampf(_i + _ki * error * dt, -_iLim, _iLim);
    return clampf(_kp * error + _i + _kd * dError, _lo, _hi);
  }

 private:
  float _kp = 0, _ki = 0, _kd = 0;
  float _lo = -1e9f, _hi = 1e9f, _iLim = 1e9f;
  float _i = 0, _prevErr = 0;
  bool  _first = true;
};

// ---------------------------------------------------------------------------
//  RampLimiter: separate accelerate / decelerate slopes (units per second^2)
// ---------------------------------------------------------------------------
class RampLimiter {
 public:
  void  reset(float v = 0) { _v = v; }
  float value() const { return _v; }

  float update(float target, float dt, float rateUp, float rateDown) {
    // "accelerating" = moving away from zero in the current direction
    bool accelerating = (fabsf(target) > fabsf(_v)) && (target * _v >= 0.0f);
    float maxStep = (accelerating ? rateUp : rateDown) * dt;
    _v += clampf(target - _v, -maxStep, maxStep);
    return _v;
  }

 private:
  float _v = 0;
};

// ---------------------------------------------------------------------------
//  First order low-pass
// ---------------------------------------------------------------------------
class LowPass {
 public:
  void  reset(float v = 0) { _y = v; }
  float value() const { return _y; }
  float update(float x, float dt, float tau) {
    _y += (dt / (tau + dt)) * (x - _y);
    return _y;
  }

 private:
  float _y = 0;
};
