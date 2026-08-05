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

// Runaway tracker: sample BT at the start of each observation window while the
// heater is high; if BT hasn't risen enough by the window's end, trip.
static uint32_t gRunawayWindowMs = 0;
static float    gRunawayStartBt  = NAN;

void safetyBegin() {
  gLastHeartbeatMs = millis();
  gBadProbeCount   = 0;
  gRunawayWindowMs = 0;
  gRunawayStartBt  = NAN;
}

void safetyNoteHeartbeat() {
  gLastHeartbeatMs = millis();
}

// Throttled logging so a persistent fault doesn't spam the serial log.
static void logThrottled(const char* msg) {
  static uint32_t lastMs = 0;
  if (millis() - lastMs > 5000) { Serial.println(msg); lastMs = millis(); }
}

bool safetyOk(const Temps& t, uint8_t heaterPct) {
  const uint32_t now = millis();

  // 1) Comms watchdog: no master heartbeat within timeout -> cut heat.
  if (now - gLastHeartbeatMs > SAFETY_COMMS_TIMEOUT_MS) {
    logThrottled("[safety] master heartbeat lost — cutting heat");
    return false;
  }

  // 2) Dead probe: NaN BT for too many consecutive reads -> cut heat.
  //    (A dumb actuator must never heat blind. Counts up on NaN, resets on a
  //    good read.) Runs before the over-temp check, which needs a valid BT.
  if (isnan(t.bt)) {
    if (++gBadProbeCount >= SAFETY_BADPROBE_MAX) {
      logThrottled("[safety] BT probe dead/unwired — cutting heat");
      return false;
    }
    // Not yet at the debounce threshold: don't over-temp/runaway check on NaN,
    // but also don't allow the runaway window to use a stale BT.
    gRunawayStartBt = NAN;
    return true;   // brief NaN glitch tolerated
  }
  gBadProbeCount = 0;

  // 3) Over-temp: hard BT cutoff regardless of commanded %.
  if (t.bt >= SAFETY_MAX_BT_C) {
    Serial.printf("[safety] BT %.1f >= %.1f — cutting heat\n", t.bt, SAFETY_MAX_BT_C);
    return false;
  }

  // 4) Thermal runaway: if we're driving the heater hard but BT isn't rising,
  //    the probe has likely left the bean mass (reads cool while the element
  //    cooks). Observe over a window; require a minimum rise. ONLY armed below
  //    SAFETY_RUNAWAY_MAX_ARM_C: above that the probe is clearly in hot beans and
  //    a flat BT just means "settled on target" (over-temp still guards overshoot).
  if (t.bt >= SAFETY_RUNAWAY_MAX_ARM_C) {
    gRunawayWindowMs = 0;               // disarm near the setpoint; reset tracker
    gRunawayStartBt  = NAN;
  } else if (heaterPct >= SAFETY_RUNAWAY_HEAT_PCT) {
    if (gRunawayWindowMs == 0 || isnan(gRunawayStartBt)) {
      gRunawayWindowMs = now;          // (re)start the window
      gRunawayStartBt  = t.bt;
    } else if (now - gRunawayWindowMs >= SAFETY_RUNAWAY_WINDOW_MS) {
      const float rise = t.bt - gRunawayStartBt;
      if (rise < SAFETY_RUNAWAY_MIN_RISE_C) {
        Serial.printf("[safety] runaway: heater %u%% but BT rose only %.1f C in %lus — cutting heat\n",
                      heaterPct, rise, (unsigned long)(SAFETY_RUNAWAY_WINDOW_MS / 1000));
        return false;
      }
      gRunawayWindowMs = now;          // rise ok — slide the window forward
      gRunawayStartBt  = t.bt;
    }
  } else {
    gRunawayWindowMs = 0;              // heater not high: reset the tracker
    gRunawayStartBt  = NAN;
  }

  return true;
}
