#pragma once
// ============================================================================
//  Dashboard.h  -  WiFi access point + web dashboard using ONLY the built-in
//  ESP32 <WebServer.h>  (no extra libraries needed).
//
//  The page talks to the robot with plain HTTP GET requests:
//     /            the dashboard page
//     /joy?x=&y=   joystick values (-1..1, y>0 = forward, x>0 = right);
//                  the reply is the live telemetry JSON, so one request
//                  both drives the robot and updates the screen
//     /log         flight recorder: last 3 s before a stop / fall (CSV text)
//     /cmd?c=      start | stop | mark | return | calib | test | save | reset
//                  calup | calfwd | setbal | tilt        (IMU setup + bench test)
//                  zero                                   (gyro offset + balance point in one go)
//                  keeptrim                               (store the learned auto-trim as balance trim)
//                  tune | untune | height                 (auto-tune, undo it, gains from Robot height)
//     /set?k=&v=   change a setting live; the reply shows the value the robot
//                  really applied and the gains it really uses (individual x master):
//                  {"ok":1,"v":560,"e":[balKp,balKd,spdKp,spdKi,spdKd,posKp]}
//     /params      all current settings as JSON (+ "e" effective gains, "lim" top wheel speed)
// ============================================================================
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "BalanceController.h"
#include "DashboardPage.h"

class Dashboard {
 public:
  Dashboard(Settings& s, BalanceController& r) : _s(s), _r(r), _server(80) {}

  void begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(Net::AP_SSID, Net::AP_PASS);
    WiFi.setSleep(false);   // modem sleep adds 100+ ms of joystick lag
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.softAPIP());

    _server.on("/",       HTTP_GET, [this]() { _server.sendHeader("Cache-Control", "no-cache");
                                               _server.send_P(200, "text/html", INDEX_HTML); });
    _server.on("/joy",    HTTP_GET, [this]() { onJoy(); });
    _server.on("/cmd",    HTTP_GET, [this]() { onCmd(); });
    _server.on("/set",    HTTP_GET, [this]() { onSet(); });
    _server.on("/params", HTTP_GET, [this]() { onParams(); });
    _server.on("/log",    HTTP_GET, [this]() { onLog(); });
    _server.onNotFound([this]() { _server.send(404, "text/plain", "Not found"); });
    _server.begin();
  }

  // call from loop()
  void update() { _server.handleClient(); }

 private:
  Settings&          _s;
  BalanceController& _r;
  WebServer          _server;

  void replyJson(const char* json) {
    _server.sendHeader("Cache-Control", "no-cache");
    _server.send(200, "application/json", json);
  }

  // flight recorder: the last 3 s of balancing (frozen when the robot stopped / fell), as CSV text
  void onLog() {
    String out;
    out.reserve(14000);
    out += "ms_before_end,angle_deg,gyro_dps,wheel_cmd_mm_s,wheel_accel_mm_s2,target_lean_deg,speed_mm_s\n";
    const int n = _r.recCount();
    if (n == 0) out += "(nothing recorded yet - balance first)\n";
    char row[72];
    for (int i = 0; i < n; i++) {
      const BalanceController::Rec q = _r.recAt(i);
      snprintf(row, sizeof(row), "%d,%.2f,%.1f,%d,%d,%.2f,%d\n", (i - n + 1) * 10, q.ang / 100.0f, q.rate / 10.0f,
               (int)q.vCmd, (int)q.acc * 4, q.tilt / 100.0f, (int)q.vMeas);
      out += row;
    }
    _server.sendHeader("Cache-Control", "no-cache");
    _server.send(200, "text/plain", out);
  }

  // make a char buffer safe to put between JSON quotes (terminated, no quote / backslash / control chars)
  static void jsonSafe(char* s, size_t cap) {
    s[cap - 1] = 0;
    for (size_t i = 0; s[i]; i++)
      if (s[i] == '"' || s[i] == '\\' || (unsigned char)s[i] < 32) s[i] = ' ';
  }

  void onJoy() {
    _r.setJoystick(clampf(_server.arg("x").toFloat(), -1.0f, 1.0f),
                   clampf(_server.arg("y").toFloat(), -1.0f, 1.0f));
    Telemetry t = _r.telemetry();
    // the control task writes these at the same time: keep the JSON valid whatever we copied
    float* fv[] = {&t.angle, &t.rate, &t.speed, &t.x, &t.y, &t.heading, &t.tilt, &t.wheel, &t.accel,
                   &t.ax, &t.ay, &t.az, &t.gx, &t.gy, &t.gz, &t.dtMax, &t.lim, &t.trimAuto,
                   &t.resp, &t.wobHz, &t.wobDeg};
    for (float* f : fv) if (!isfinite(*f)) *f = 0.0f;
    char msg[160];
    snprintf(msg, sizeof(msg), "%s", _r.message());
    jsonSafe(t.ori, sizeof(t.ori));
    jsonSafe(t.fall, sizeof(t.fall));
    jsonSafe(msg, sizeof(msg));
    float h = fmodf(t.heading, 360.0f);
    if (h > 180.0f) h -= 360.0f;
    if (h < -180.0f) h += 360.0f;
    char buf[1400];
    snprintf(buf, sizeof(buf),
             "{\"a\":%.2f,\"r\":%.1f,\"v\":%.0f,\"x\":%.0f,\"y\":%.0f,\"h\":%.1f,\"tl\":%.2f,"
             "\"ws\":%.0f,\"ac\":%.0f,"
             "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
             "\"st\":%d,\"nav\":%d,\"hold\":%d,\"ori\":\"%s\","
             "\"dt\":%.1f,\"late\":%lu,\"lim\":%.0f,\"rs\":%.2f,\"at\":%.2f,\"yg\":%d,\"fall\":\"%s\","
             "\"tn\":%d,\"wh\":%.1f,\"wd\":%.1f,"
             "\"mseq\":%lu,\"msg\":\"%s\"}",
             t.angle, t.rate, t.speed, t.x, t.y, h, t.tilt, t.wheel, t.accel,
             t.ax, t.ay, t.az, t.gx, t.gy, t.gz,
             (int)t.state, (int)t.nav, t.hold ? 1 : 0, t.ori,
             t.dtMax, (unsigned long)t.late, t.lim, t.resp, t.trimAuto, t.yawGyro ? 1 : 0, t.fall,
             t.tune, t.wobHz, t.wobDeg,
             (unsigned long)_r.messageSeq(), msg);
    replyJson(buf);
  }

  void onCmd() {
    const String c = _server.arg("c");
    if      (c == "start")  _r.cmdStart();
    else if (c == "stop")   _r.cmdStop();
    else if (c == "mark")   _r.cmdMarkHome();
    else if (c == "return") _r.cmdReturnHome();
    else if (c == "calib")  _r.cmdCalibrate();
    else if (c == "test")   _r.cmdMotorTest();
    else if (c == "calup")  _r.cmdCalUpright();
    else if (c == "calfwd") _r.cmdCalForward();
    else if (c == "setbal") _r.cmdSetBalance();
    else if (c == "tilt")   _r.cmdTiltTest();
    else if (c == "zero")   _r.cmdZeroImu();
    else if (c == "keeptrim") _r.cmdKeepTrim();
    else if (c == "tune")   _r.cmdAutoTune();
    else if (c == "untune") _r.cmdUndoTune();
    else if (c == "height") _r.cmdHeightGains();
    else if (c == "save") {
      const RobotState st = _r.state();
      if (st == RobotState::STOPPED || st == RobotState::FALLEN || st == RobotState::RUNAWAY) {
        _s.save();
        _r.notify("Saved to flash");
      } else {
        _r.notify("Stop the robot before saving");
      }
    } else if (c == "reset") {
      const RobotState st = _r.state();
      if (!(st == RobotState::STOPPED || st == RobotState::FALLEN || st == RobotState::RUNAWAY)) {
        _r.notify("Stop the robot before resetting");
        replyJson("{\"ok\":1}");
        return;
      }
      _s.resetDefaults();
      _r.notify("Tuning reset to defaults (IMU setup kept, not saved yet)");
    } else {
      _server.send(400, "application/json", "{\"ok\":0}");
      return;
    }
    replyJson("{\"ok\":1}");
  }

  void onSet() {
    const String k = _server.arg("k");
    float applied = 0;
    if (!_server.hasArg("v")) { replyJson("{\"ok\":0}"); return; }   // no value: do not write 0 / the minimum
    if (_s.set(k.c_str(), _server.arg("v").toFloat()) && _s.get(k.c_str(), applied)) {
      char buf[192];
      size_t n = snprintf(buf, sizeof(buf), "{\"ok\":1,\"v\":%.6g,\"e\":", applied);
      n += effJson(buf + n, sizeof(buf) - n);
      snprintf(buf + n, sizeof(buf) - n, "}");
      replyJson(buf);
    } else {
      replyJson("{\"ok\":0}");
    }
  }

  // the gains the controller really uses: individual value x master value
  size_t effJson(char* buf, size_t cap) const {
    const Settings::Gains g = _s.gains();
    return snprintf(buf, cap, "[%.6g,%.6g,%.6g,%.6g,%.6g,%.6g]",
                    g.balKp, g.balKd, g.spdKp, g.spdKi, g.spdKd, g.posKp);
  }

  void onParams() {
    static char buf[2048];
    size_t n = _s.toJson(buf, sizeof(buf) - 160);
    if (n >= 2 && buf[n - 1] == '}') {                 // turn  {"p":{...}}  into  {"p":{...},"e":[...],"lim":N}
      n -= 1;
      n += snprintf(buf + n, sizeof(buf) - n, ",\"e\":");
      n += effJson(buf + n, sizeof(buf) - n);
      snprintf(buf + n, sizeof(buf) - n, ",\"lim\":%.0f}", _r.telemetry().lim);
    }
    replyJson(buf);
  }
};
