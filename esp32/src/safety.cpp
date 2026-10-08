#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "safety.h"

// Independent safety layer. In Path 2 the master (HA / Node-RED / script) runs
// the PID and just sends heater %, so the ESP cannot trust it — these guards
// cut heat regardless of what the master commands. Firmware can't protect
// against its own crash; a hardware thermal fuse in series with the SSR load is
// the ultimate backstop.

static uint32_t gLastHeartbeatMs = 0;
static uint16_t gBadProbeCount   = 0;
static bool     gOverTemp        = false;   // latched over-temp heat hold (hysteresis)

// Runaway tracker: sample air at the start of each observation window while the
// heater is high; if air hasn't risen enough by the window's end, trip.
static uint32_t gRunawayWindowMs = 0;
static float    gRunawayStartAir  = NAN;

void safetyBegin() {
  gLastHeartbeatMs = millis();
  gBadProbeCount   = 0;
  gOverTemp        = false;
  gRunawayWindowMs = 0;
  gRunawayStartAir  = NAN;
}

void safetyNoteHeartbeat() {
  gLastHeartbeatMs = millis();
}

// Throttled logging so a persistent fault doesn't spam the serial log.
static void logThrottled(const char* msg) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs > 5000) { Serial.println(msg); lastMs = millis(); }
}

SafetyVerdict safetyCheck(const Temps& t, uint8_t heaterPct) {
  const uint32_t now = millis();

  // 1) Comms watchdog: no master heartbeat within timeout -> cut heat.
  if (now - gLastHeartbeatMs > SAFETY_COMMS_TIMEOUT_MS) {
    logThrottled("[safety] master heartbeat lost — cutting heat");
    return SAFETY_FAILSAFE;
  }

  // 2) Dead probe: NaN air for too many consecutive reads -> cut heat.
  //    (A dumb actuator must never heat blind. Counts up on NaN, resets on a
  //    good read.) Runs before the over-temp check, which needs a valid reading.
  if (isnan(t.air)) {
    if (++gBadProbeCount >= SAFETY_BADPROBE_MAX) {
      logThrottled("[safety] air probe dead/unwired — cutting heat");
      return SAFETY_FAILSAFE;
    }
    // Not yet at the debounce threshold: don't over-temp/runaway check on NaN,
    // but also don't allow the runaway window to use a stale reading. A NaN glitch
    // must NOT release a latched over-temp hold, or heat would come back early.
    gRunawayStartAir = NAN;
    return gOverTemp ? SAFETY_HEAT_HOLD : SAFETY_OK;   // brief glitch tolerated
  }
  gBadProbeCount = 0;

  // 3) Over-temp heat hold, latched with hysteresis: trip at SAFETY_MAX_AIR_C, stay
  //    held until air falls below SAFETY_AIR_RESUME_C. Only the heater is cut; the
  //    fan keeps following the master (see config.h for why not fan 100 %).
  if (gOverTemp) {
    if (t.air < SAFETY_AIR_RESUME_C) {
      gOverTemp = false;
      Serial.printf("[safety] air %.1f < %.1f — heat allowed again\n", t.air, SAFETY_AIR_RESUME_C);
    }
  } else if (t.air >= SAFETY_MAX_AIR_C) {
    gOverTemp = true;
    Serial.printf("[safety] air %.1f >= %.1f — holding heat off\n", t.air, SAFETY_MAX_AIR_C);
  }
  if (gOverTemp) return SAFETY_HEAT_HOLD;

  // 4) Thermal runaway: if we're driving the heater hard but air isn't rising,
  //    the probe has likely left the airflow (reads cool while the element
  //    cooks). Observe over a window; require a minimum rise. ONLY armed below
  //    SAFETY_RUNAWAY_MAX_ARM_C: above that the probe is clearly in hot air and
  //    a flat reading just means "settled on target" (over-temp still guards overshoot).
  if (t.air >= SAFETY_RUNAWAY_MAX_ARM_C) {
    gRunawayWindowMs = 0;               // disarm near the setpoint; reset tracker
    gRunawayStartAir  = NAN;
  } else if (heaterPct >= SAFETY_RUNAWAY_HEAT_PCT) {
    if (gRunawayWindowMs == 0 || isnan(gRunawayStartAir)) {
      gRunawayWindowMs = now;          // (re)start the window
      gRunawayStartAir  = t.air;
    } else if (now - gRunawayWindowMs >= SAFETY_RUNAWAY_WINDOW_MS) {
      const float rise = t.air - gRunawayStartAir;
      if (rise < SAFETY_RUNAWAY_MIN_RISE_C) {
        Serial.printf("[safety] runaway: heater %u%% but air rose only %.1f C in %lus — cutting heat\n",
                      heaterPct, rise, (unsigned long)(SAFETY_RUNAWAY_WINDOW_MS / 1000));
        return SAFETY_FAILSAFE;
      }
      gRunawayWindowMs = now;          // rise ok — slide the window forward
      gRunawayStartAir  = t.air;
    }
  } else {
    gRunawayWindowMs = 0;              // heater not high: reset the tracker
    gRunawayStartAir  = NAN;
  }

  return SAFETY_OK;
}
