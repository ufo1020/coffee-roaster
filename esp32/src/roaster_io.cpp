#include <Arduino.h>
#include "config.h"
#include "roaster_io.h"

// Heater is driven by a zero-cross SSR (Delixi CDG1-1DA). The SSR switches at
// mains zero-cross on its own, so we get clean proportional power with plain
// SLOW PWM: over a HEATER_PWM_PERIOD_MS window, hold the gate ON for heaterPct%
// of the window and OFF the rest. The SSR turns each edge into whole mains
// cycles. No zero-cross detector or phase control needed.
//
// The window phase is tracked with millis() so it's independent of how often
// roasterIoApply() is called (it's called at CONTROL_LOOP_HZ).

// --- Fan ESC (servo signal) ---------------------------------------------------
// The blower is a brushless motor on an RC ESC (see config.h). We drive its "S"
// pad with a 50 Hz servo pulse via the LEDC peripheral: pick a timer resolution,
// then set the duty so the high time is FAN_PULSE_MIN_US..FAN_PULSE_MAX_US.
//
// 16-bit @ 50 Hz gives a 20000 us frame across 65535 counts => ~0.305 us/count,
// plenty of resolution for a 1000 us throttle band.
static constexpr uint8_t  FAN_LEDC_BITS = 16;
static constexpr uint32_t FAN_LEDC_MAX  = (1u << FAN_LEDC_BITS) - 1;

static uint32_t fanUsToDuty(uint32_t us) {
  // duty = pulse_us / frame_us * full_scale ; frame_us = 1e6 / FAN_ESC_HZ.
  return (uint32_t)((uint64_t)us * FAN_ESC_HZ * FAN_LEDC_MAX / 1000000ULL);
}

static void fanWriteUs(uint32_t us) {
  if (PIN_FAN_PWM < 0) return;
  ledcWrite(PIN_FAN_PWM, fanUsToDuty(us));
}

static uint32_t fanPctToUs(uint8_t pct) {
  if (pct > 100) pct = 100;
  // 0 % -> hard OFF (below the ESC's stall point, motor stopped).
  if (pct == 0) return FAN_PULSE_MIN_US;
  // 1..100 % -> spread across the ESC's actual usable band so the whole slider
  // maps to real airflow instead of wasting the bottom (dead zone) and top
  // (everything above ~1850 us is identical). 1 % sits at the lowest steady
  // speed, 100 % at true max.
  return FAN_RUN_MIN_US +
         (uint32_t)(pct - 1) * (FAN_PULSE_MAX_US - FAN_RUN_MIN_US) / 99;
}

void roasterIoBegin() {
  if (PIN_HEATER_SSR >= 0) { pinMode(PIN_HEATER_SSR, OUTPUT); digitalWrite(PIN_HEATER_SSR, LOW); }
  if (PIN_FAN_PWM    >= 0) {
    // Start the ESC signal and hold min throttle so the ESC arms before we ever
    // command real speed. Blocks briefly in setup() — fine, nothing else runs yet.
    ledcAttach(PIN_FAN_PWM, FAN_ESC_HZ, FAN_LEDC_BITS);
    fanWriteUs(FAN_PULSE_MIN_US);
    delay(FAN_ARM_MS);
  }
  if (PIN_STATUS_LED >= 0) { pinMode(PIN_STATUS_LED, OUTPUT); }
  if (PIN_BUZZER     >= 0) { pinMode(PIN_BUZZER, OUTPUT); }
  // TODO: attachInterrupt(PIN_ZERO_CROSS, ...) once wired.
}

void roasterIoApply(const ControlOutputs& o) {
  uint8_t heat = o.heaterPct > 100 ? 100 : o.heaterPct;
  uint8_t fan  = o.fanPct    > 100 ? 100 : o.fanPct;

  // SAFETY INTERLOCK (air roaster): the heater may never run without airflow.
  // If the fan is below HEATER_MIN_FAN_PCT, force the heater fully OFF here —
  // the lowest possible level, so it holds no matter what the control layer or
  // master commanded. This is the hardware truth: heater alone => scorch/fire.
  if (fan < HEATER_MIN_FAN_PCT) heat = 0;

  // --- heater ---  slow-PWM burst control (SSR is internally zero-cross).
  if (PIN_HEATER_SSR >= 0) {
    uint32_t phase = millis() % HEATER_PWM_PERIOD_MS;
    bool on = phase < (uint32_t)heat * HEATER_PWM_PERIOD_MS / 100;
    digitalWrite(PIN_HEATER_SSR, (heat > 0 && on) ? HIGH : LOW);
  }

  // --- fan ---  RC ESC servo pulse: map 0..100 % -> min..max pulse width.
  if (PIN_FAN_PWM >= 0)
    fanWriteUs(fanPctToUs(fan));
}
