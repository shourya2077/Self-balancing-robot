#pragma once
// ============================================================================
//  ImuCalibration.h  -  find out how the MPU board is mounted, automatically
//
//  The robot code needs to know which sensor axis points UP, which points to
//  the FRONT, and which axis the robot pitches around. Instead of editing
//  numbers, the dashboard asks for two poses:
//     1. robot held upright and still      -> tells which axis is "up"
//     2. robot tilted forward ~25 degrees  -> tells which axis is "front"
//  The third axis (the pitch axis) and the gyro direction follow from the
//  right-hand rule.
// ============================================================================
#include <math.h>
#include <stdio.h>
#include "Config.h"

struct Orientation {
  int   upAxis   = Imu::ACC_UP_AXIS;     // sensor axis (0=X 1=Y 2=Z) pointing up
  int   fwdAxis  = Imu::ACC_FWD_AXIS;    // sensor axis along the front/back direction
  int   gyroAxis = Imu::GYRO_AXIS;       // sensor axis the robot pitches around
  float upSign   = Imu::ACC_UP_SIGN;     // +1 if accel reads +1 g on upAxis when upright
  float fwdSign  = Imu::ACC_FWD_SIGN;    // sign that makes "lean forward" positive
  float gyroSign = Imu::GYRO_SIGN;       // sign that makes "lean forward" rate positive

  bool valid() const {
    return upAxis >= 0 && upAxis <= 2 && fwdAxis >= 0 && fwdAxis <= 2 && gyroAxis >= 0 && gyroAxis <= 2 &&
           upAxis != fwdAxis && upAxis != gyroAxis && fwdAxis != gyroAxis;
  }

  bool operator==(const Orientation& o) const {
    return upAxis == o.upAxis && fwdAxis == o.fwdAxis && gyroAxis == o.gyroAxis && upSign == o.upSign &&
           fwdSign == o.fwdSign && gyroSign == o.gyroSign;
  }

  // forward lean = positive angle (degrees); works with raw or g units
  float accAngleDeg(const float a[3]) const {
    return atan2f(fwdSign * a[fwdAxis], upSign * a[upAxis]) * RAD_TO_DEG;
  }

  // "up +Z, front +X"
  void describe(char* buf, size_t n) const {
    snprintf(buf, n, "up %c%c, front %c%c", upSign > 0 ? '+' : '-', "XYZ"[upAxis & 3],
             fwdSign < 0 ? '+' : '-', "XYZ"[fwdAxis & 3]);
  }

  static Orientation fromSettings(const Settings& s) {
    Orientation o;
    o.upAxis   = axisOf(s.imuUpAxis);
    o.fwdAxis  = axisOf(s.imuFwdAxis);
    o.gyroAxis = axisOf(s.imuGyroAxis);
    o.upSign   = s.imuUpSign >= 0 ? 1.0f : -1.0f;
    o.fwdSign  = s.imuFwdSign >= 0 ? 1.0f : -1.0f;
    o.gyroSign = s.imuGyroSign >= 0 ? 1.0f : -1.0f;
    return o.valid() ? o : Orientation();   // corrupted values -> factory default
  }

 private:
  static int axisOf(float v) { return (int)lroundf(clampf(v, 0.0f, 2.0f)); }
};

class ImuCalibrator {
 public:
  // a[] in g. nullptr = OK, otherwise a short message for the user.
  static const char* checkUpright(const float a[3]) {
    const float m = norm(a);
    if (m < 0.8f || m > 1.2f) return "Hold the robot still (sensor not reading 1 g)";
    int ax = 0;
    for (int i = 1; i < 3; i++) if (fabsf(a[i]) > fabsf(a[ax])) ax = i;
    if (fabsf(a[ax]) < 0.75f * m) return "Hold the robot more upright";
    return nullptr;
  }

  static const char* solve(const float up[3], const float tilted[3], Orientation& out) {
    const char* e = checkUpright(up);
    if (e) return e;
    const float mt = norm(tilted);
    if (mt < 0.8f || mt > 1.2f) return "Hold the robot still while tilted";

    int ua = 0;
    for (int i = 1; i < 3; i++) if (fabsf(up[i]) > fabsf(up[ua])) ua = i;
    const float us = up[ua] >= 0 ? 1.0f : -1.0f;

    float d[3];
    for (int i = 0; i < 3; i++) d[i] = tilted[i] - up[i];
    const int i1 = (ua + 1) % 3, i2 = (ua + 2) % 3;
    const int fa = fabsf(d[i1]) >= fabsf(d[i2]) ? i1 : i2;
    const int oa = (fa == i1) ? i2 : i1;

    if (fabsf(d[fa]) < 0.12f) return "Tilt the robot forward more (about 25 degrees)";
    if (fabsf(d[oa]) > 0.5f * fabsf(d[fa])) return "Tilt straight forward, not sideways";

    const float fs = d[fa] >= 0 ? 1.0f : -1.0f;
    const int   ga = 3 - ua - fa;
    const float eps = (fa == (ua + 1) % 3) ? 1.0f : -1.0f;   // handedness of (up, front, pitch)

    out.upAxis = ua;
    out.fwdAxis = fa;
    out.gyroAxis = ga;
    out.upSign = us;
    out.fwdSign = fs;
    out.gyroSign = -us * fs * eps;
    return nullptr;
  }

 private:
  static float norm(const float a[3]) { return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
};
