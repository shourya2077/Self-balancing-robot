// ============================================================================
//  Self-balancing robot  -  ESP32 + 2x NEMA17 (DRV8825) + MPU6500
//
//  Board : "ESP32 Dev Module" (classic ESP32, Arduino-ESP32 core 2.x or 3.x)
//  Libs  : none (only libraries built into the ESP32 Arduino core)
//
//  Connect phone to WiFi  BalanceBot / balance123  ->  http://192.168.4.1
// ============================================================================
#include "Config.h"
#include "Utils.h"
#include "ImuCalibration.h"
#include "Mpu6500.h"
#include "StepperMotors.h"
#include "BalanceController.h"
#include "Dashboard.h"

static Settings          settings;
static Mpu6500           imu;
static StepperMotors     motors;
static BalanceController robot(settings, imu, motors);
static Dashboard         dashboard(settings, robot);

// 200 Hz control loop, pinned to core 1 with higher priority than loop()
static void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    robot.update();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(Hw::LOOP_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("BalanceBot firmware v%s\n", FW_VERSION);

  settings.load();   // saved values from flash (or defaults)
  motors.begin();    // drivers disabled until balancing starts
  robot.begin();     // IMU init + gyro calibration (keep robot still!)
  dashboard.begin(); // WiFi access point + web server

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 3, nullptr, 1);
}

void loop() {
  dashboard.update();
  delay(1);
}
