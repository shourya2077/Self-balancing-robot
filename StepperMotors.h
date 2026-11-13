#pragma once
// ============================================================================
//  StepperMotors.h  -  two DRV8825 drivers, jitter-free step generation
//
//  A 40 kHz timer ISR runs a phase accumulator per motor. Each motor's
//  "increment" sets its step rate, so speed can change smoothly every
//  control cycle. The ISR also counts steps (odometry).
//  Written for the classic ESP32 (direct GPIO register access).
// ============================================================================
#include "Config.h"
#include <soc/gpio_struct.h>

static_assert(Pins::STEP_L < 32 && Pins::STEP_R < 32, "STEP pins must be GPIO < 32");

namespace StepGen {

struct Channel {
  volatile uint32_t acc   = 0;
  volatile uint32_t inc   = 0;
  volatile int32_t  count = 0;
  volatile int8_t   dir   = 1;   // +1 = robot-forward, -1 = robot-backward
  volatile bool     pulse = false;
  uint32_t          mask  = 0;
};

static Channel     ch[2];
static hw_timer_t* timer = nullptr;

static void IRAM_ATTR isr() {
  for (int i = 0; i < 2; i++) {
    Channel& c = ch[i];
    if (c.pulse) {                       // end of step pulse
      GPIO.out_w1tc = c.mask;
      c.pulse = false;
    } else {
      uint32_t prev = c.acc;
      uint32_t next = prev + c.inc;
      c.acc = next;
      if (next < prev) {                 // accumulator overflow -> one step
        GPIO.out_w1ts = c.mask;
        c.pulse = true;
        c.count = c.count + c.dir;
      }
    }
  }
}

}  // namespace StepGen

class StepperMotors {
 public:
  void begin() {
    pinMode(Pins::STEP_L, OUTPUT);
    pinMode(Pins::STEP_R, OUTPUT);
    pinMode(Pins::DIR_L, OUTPUT);
    pinMode(Pins::DIR_R, OUTPUT);
    pinMode(Pins::ENABLE, OUTPUT);
    digitalWrite(Pins::STEP_L, LOW);
    digitalWrite(Pins::STEP_R, LOW);
    digitalWrite(Pins::ENABLE, HIGH);  // drivers off

    StepGen::ch[0].mask = 1UL << Pins::STEP_L;
    StepGen::ch[1].mask = 1UL << Pins::STEP_R;
    setSpeed(0, 0);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    StepGen::timer = timerBegin(1000000);
    timerAttachInterrupt(StepGen::timer, &StepGen::isr);
    timerAlarm(StepGen::timer, 1000000 / Hw::TICK_HZ, true, 0);
#else
    StepGen::timer = timerBegin(1, 80, true);
    timerAttachInterrupt(StepGen::timer, &StepGen::isr, true);
    timerAlarmWrite(StepGen::timer, 1000000 / Hw::TICK_HZ, true);
    timerAlarmEnable(StepGen::timer);
#endif
  }

  void enable(bool on) { digitalWrite(Pins::ENABLE, on ? LOW : HIGH); }

  // steps/s, signed, positive = robot forward
  void setSpeed(float left, float right) {
    apply(0, left, Pins::DIR_L);
    apply(1, right, Pins::DIR_R);
  }

  // reverse the turning direction of a motor (dashboard setting); takes effect immediately
  void setInvert(bool left, bool right) {
    if (left != _inv[0])  { _inv[0] = left;  _dirDirty[0] = true; }
    if (right != _inv[1]) { _inv[1] = right; _dirDirty[1] = true; }
  }

  int32_t stepsLeft() const { return StepGen::ch[0].count; }
  int32_t stepsRight() const { return StepGen::ch[1].count; }
  void    resetCounts() { StepGen::ch[0].count = 0; StepGen::ch[1].count = 0; }

 private:
  // the very first setSpeed() (and any change of 'invert') always writes the DIR pin
  bool _fwd[2] = {false, false};
  bool _inv[2] = {Hw::INVERT_LEFT, Hw::INVERT_RIGHT};
  bool _dirDirty[2] = {true, true};

  void apply(int i, float rate, uint8_t dirPin) {
    bool  fwd = rate >= 0.0f;
    float mag = fabsf(rate);
    if (mag > Hw::MAX_STEP_RATE) mag = Hw::MAX_STEP_RATE;
    if (fwd != _fwd[i] || _dirDirty[i]) {
      _dirDirty[i] = false;
      digitalWrite(dirPin, (fwd != _inv[i]) ? HIGH : LOW);
      StepGen::ch[i].dir = fwd ? 1 : -1;
      _fwd[i] = fwd;
    }
    // Every step uses one extra 'pulse' tick in the ISR where the accumulator does not
    // advance, so the real rate is f*u/(1+u). Solve for u to get exactly 'mag' steps/s.
    const float f = (float)Hw::TICK_HZ;
    StepGen::ch[i].inc = (mag < 1.0f) ? 0u : (uint32_t)(4294967296.0f * mag / (f - mag));
  }
};
