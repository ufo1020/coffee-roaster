#include <Arduino.h>
#include "config.h"
#include "roaster_io.h"
#include "comm.h"     // to report what actually reached the pins, not what was asked

// Heater is driven by a zero-cross SSR (Delixi CDG1-1DA). The SSR switches at
// mains zero-cross on its own, so we get clean proportional power with plain
// SLOW PWM: over a HEATER_PWM_PERIOD_MS window, hold the gate ON for heaterPct%
// of the window and OFF the rest. The SSR turns each edge into whole mains
// cycles. No zero-cross detector or phase control needed.
//
// The gate is driven by roasterIoHeaterTick(), called on EVERY loop() pass (~1 kHz),
// not by roasterIoApply() (CONTROL_LOOP_HZ). Until FW 2026.10.20 the gate was only
// rewritten at 10 Hz against a 250 ms window, so it could only be on for 0/1/2/3/4/5
// of five 100 ms slots: real power came in 20 % steps (60/80/100 %) chosen by a
// slowly drifting phase offset. 75 % and 85 % could both deliver 80 % until the
// offset drifted ~30 s later, which the operator saw as "power changes 30 s late".

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
    // Define the signal line BEFORE the LEDC peripheral is attached. Until then the
    // pin is an undriven input, and whatever the ESC samples on a floating line —
    // or any duty the timer happens to come up with before the first fanWriteUs() —
    // is a throttle command we never intended. A steady LOW is not a valid servo
    // frame at all, so the ESC simply sees "no signal" and stays idle, which is the
    // one state we can be sure is safe.
    pinMode(PIN_FAN_PWM, OUTPUT);
    digitalWrite(PIN_FAN_PWM, LOW);
    // Now start the real signal and hold min throttle so the ESC arms before we
    // ever command speed. Blocks briefly in setup() — fine, nothing else runs yet.
    ledcAttach(PIN_FAN_PWM, FAN_ESC_HZ, FAN_LEDC_BITS);
    fanWriteUs(FAN_PULSE_MIN_US);
    delay(FAN_ARM_MS);
  }
  if (PIN_STATUS_LED >= 0) { pinMode(PIN_STATUS_LED, OUTPUT); }
  if (PIN_BUZZER     >= 0) { pinMode(PIN_BUZZER, OUTPUT); }
  // TODO: attachInterrupt(PIN_ZERO_CROSS, ...) once wired.
}

// millis() at which the APPLIED fan first reached HEATER_MIN_FAN_PCT, or 0 while
// it is below it. Drives the spin-up lead below. Static so it survives across
// calls and can never be reset by anything upstream.
static uint32_t gFanUpSinceMs = 0;

// Heater % that passed the interlock and spin-up lead; the gate follows it.
static volatile uint8_t gHeatApplied = 0;

void roasterIoHeaterTick() {
  if (PIN_HEATER_SSR < 0) return;
  const uint8_t heat = gHeatApplied;
  const uint32_t phase = millis() % HEATER_PWM_PERIOD_MS;
  const bool on = phase < (uint32_t)heat * HEATER_PWM_PERIOD_MS / 100;
  digitalWrite(PIN_HEATER_SSR, (heat > 0 && on) ? HIGH : LOW);
}

void roasterIoApply(const ControlOutputs& o) {
  uint8_t heat = o.heaterPct > 100 ? 100 : o.heaterPct;
  uint8_t fan  = o.fanPct    > 100 ? 100 : o.fanPct;

  // SOFT START (see FAN_START_PCT): never step the blower up from a dead stop, or
  // the ESC can fail to sync and leave the motor stationary while we believe it is
  // running. Break away gently, then ramp. Everything below — interlock, spin-up
  // lead, telemetry — then works off the speed we are ACTUALLY commanding.
  {
    static uint8_t  gFanApplied = 0;
    static uint32_t gFanLastMs  = 0;
    const uint32_t now = millis();
    const uint32_t dt  = gFanLastMs ? (uint32_t)(now - gFanLastMs) : 0;
    gFanLastMs = now;

    if (fan == 0) {
      gFanApplied = 0;                       // stopped: forget the ramp
    } else if (gFanApplied == 0) {
      gFanApplied = fan < FAN_START_PCT ? fan : FAN_START_PCT;   // break away
    } else if (fan > gFanApplied) {
      const uint32_t step = (uint32_t)FAN_SLEW_PCT_PER_S * dt / 1000u;
      gFanApplied = ((uint32_t)(fan - gFanApplied) <= step) ? fan
                                                            : (uint8_t)(gFanApplied + step);
    } else {
      gFanApplied = fan;                     // slowing down is always immediate
    }
    fan = gFanApplied;
  }

  // SAFETY INTERLOCK (air roaster): the heater may never run without airflow.
  // If the fan is below HEATER_MIN_FAN_PCT, force the heater fully OFF here —
  // the lowest possible level, so it holds no matter what the control layer or
  // master commanded. This is the hardware truth: heater alone => scorch/fire.
  if (fan < HEATER_MIN_FAN_PCT) heat = 0;

  // SPIN-UP LEAD (see HEATER_FAN_LEAD_MS): the interlock is satisfied the instant
  // a fan PERCENTAGE arrives, but air does not move for another second or two. Keep
  // the heater off until the fan has genuinely been up that long, so the element can
  // never energise into a blower that is still starting. Only waits when airflow was
  // actually below the threshold — ramping 30 % -> 80 % never waits.
  {
    const uint32_t now = millis();
    if (fan >= HEATER_MIN_FAN_PCT) {
      // 0 is the "fan is down" sentinel, so skip it on the ~49-day wrap.
      if (gFanUpSinceMs == 0) gFanUpSinceMs = (now == 0) ? 1 : now;
      // unsigned subtraction, so this stays correct across millis() rollover
      if ((uint32_t)(now - gFanUpSinceMs) < HEATER_FAN_LEAD_MS) heat = 0;
    } else {
      gFanUpSinceMs = 0;
    }
  }

  // --- heater ---  slow-PWM burst control (SSR is internally zero-cross). Store
  // the level; roasterIoHeaterTick() times the gate. Tick once now so a cut (0)
  // reaches the pin immediately, not on the next loop pass.
  gHeatApplied = heat;
  roasterIoHeaterTick();

  // --- fan ---  RC ESC servo pulse: map 0..100 % -> min..max pulse width.
  if (PIN_FAN_PWM >= 0)
    fanWriteUs(fanPctToUs(fan));

  // Report what the PINS got, not what the control layer asked for. control.cpp
  // fills these in before this function runs, from the pre-interlock intent — so a
  // roast log could show "heater 80 / fan 80" through an interlock cut, a spin-up
  // hold or a soft-start ramp, none of which happened at the hardware. Overwrite
  // with the truth: this is the only place that knows it, and both the UI and the
  // master's own airflow check read these values.
  commSetInput(IREG_HEATER_ACT, heat);
  commSetInput(IREG_FAN_ACT,    fan);
}
