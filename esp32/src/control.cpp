#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"
#include "control.h"
#include "comm.h"

static Temps        gTemps{ NAN, NAN };
static RoasterState gState = STATE_IDLE;

// --- RoR (rate of rise) tracking on BT ---
// Slope over a trailing ROR_WINDOW_MS window rather than a single sample delta:
// the MAX6675 jitter makes a one-step derivative unusable (see config.h). We keep
// a small ring of timestamped BT samples and take the slope from the oldest sample
// still inside the window to the newest. NaN reads are skipped (not buffered) so a
// brief dead-probe glitch doesn't poison the slope.
static float    gRoR = NAN;           // C/min
#define ROR_BUF_N   64                // 30 s @ 2 Hz = ~60 samples; headroom
static float    gRorBt[ROR_BUF_N];
static uint32_t gRorMs[ROR_BUF_N];
static uint8_t  gRorHead = 0;         // next write slot
static uint8_t  gRorCount = 0;        // valid samples in the ring

// --- PID (only used when HREG_MODE == 1) ---
static double pidIn = 0, pidOut = 0, pidSet = 0;
// TODO: tune Kp/Ki/Kd on the real roaster.
static PID pid(&pidIn, &pidOut, &pidSet, 2.0, 0.5, 1.0, DIRECT);

void controlBegin() {
  pid.SetOutputLimits(0, HEATER_MAX_PCT);
  pid.SetMode(AUTOMATIC);
}

void controlUpdateTemps(const Temps& t) {
  gTemps = t;

  // RoR = BT slope over a trailing ROR_WINDOW_MS window (see the ring above).
  const uint32_t nowMs = millis();
  if (!isnan(t.bt)) {
    gRorBt[gRorHead] = t.bt;
    gRorMs[gRorHead] = nowMs;
    gRorHead = (gRorHead + 1) % ROR_BUF_N;
    if (gRorCount < ROR_BUF_N) gRorCount++;

    // Find the oldest sample still within the window; slope from it to now.
    float    oldBt = NAN;
    uint32_t oldMs = 0;
    for (uint8_t i = 0; i < gRorCount; i++) {
      // walk back from the most-recent write
      const uint8_t idx = (gRorHead + ROR_BUF_N - 1 - i) % ROR_BUF_N;
      if (nowMs - gRorMs[idx] <= ROR_WINDOW_MS) { oldBt = gRorBt[idx]; oldMs = gRorMs[idx]; }
      else break;
    }
    const float dtMin = (nowMs - oldMs) / 60000.0f;
    gRoR = (!isnan(oldBt) && dtMin > 0) ? (t.bt - oldBt) / dtMin : NAN;
  }

  // Publish temps -> comm telemetry (C x10).
  commSetInput(IREG_BT,     isnan(t.bt)  ? 0 : (int16_t)lroundf(t.bt  * 10));
  commSetInput(IREG_ET,     isnan(t.et)  ? 0 : (int16_t)lroundf(t.et  * 10));
  commSetInput(IREG_BT_ROR, isnan(gRoR)  ? 0 : (int16_t)lroundf(gRoR * 10));
  commSetInput(IREG_STATE,  (uint16_t)gState);
}

Temps controlCurrentTemps() { return gTemps; }

ControlOutputs controlStep() {
  const uint16_t mode   = commGetHolding(HREG_MODE);
  const uint16_t fanSp  = commGetHolding(HREG_FAN_SP);

  ControlOutputs out{};
  out.fanPct = (uint8_t)constrain((int)fanSp, 0, 100);

  if (mode == 1) {                       // ---- local PID ----
    pidSet = commGetHolding(HREG_PID_TARGET) / 10.0;   // C x10 -> C
    pidIn  = isnan(gTemps.bt) ? pidSet : gTemps.bt;    // fail closed if no probe
    pid.Compute();
    out.heaterPct = (uint8_t)constrain((int)lround(pidOut), 0, HEATER_MAX_PCT);
  } else {                               // ---- manual passthrough ----
    out.heaterPct = (uint8_t)constrain((int)commGetHolding(HREG_HEATER_SP), 0, HEATER_MAX_PCT);
  }

  gState = out.heaterPct > 0 ? STATE_ROASTING : STATE_IDLE;
  commSetInput(IREG_HEATER_ACT, out.heaterPct);
  commSetInput(IREG_FAN_ACT,    out.fanPct);
  return out;
}

ControlOutputs controlFailSafe() {
  gState = STATE_FAULT;
  commSetInput(IREG_STATE, (uint16_t)gState);
  commSetInput(IREG_HEATER_ACT, 0);
  // Heater OFF. Keep fan at 100 % to cool the beans/element.
  return ControlOutputs{ /*heaterPct*/ 0, /*fanPct*/ 100 };
}
