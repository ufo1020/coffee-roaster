#pragma once
#include "thermocouples.h"

void safetyBegin();

// What the control layer must do this step. Independent of the master (Path 2: ESP
// is a dumb actuator, so it must guard itself).
enum SafetyVerdict {
  SAFETY_OK,          // apply the commanded outputs
  SAFETY_HEAT_HOLD,   // over-temp: heater 0, fan stays as commanded. Latched with
                      // hysteresis: trips at SAFETY_MAX_AIR_C, clears below
                      // SAFETY_AIR_RESUME_C.
  SAFETY_FAILSAFE,    // heater 0, fan 100 % to cool (controlFailSafe)
};

// Checks, in order:
//   * comms watchdog  — no heartbeat within SAFETY_COMMS_TIMEOUT_MS        -> FAILSAFE
//   * dead probe      — air NaN for SAFETY_BADPROBE_MAX consecutive reads  -> FAILSAFE
//   * over-temp       — air >= SAFETY_MAX_AIR_C (until < SAFETY_AIR_RESUME_C) -> HEAT_HOLD
//   * thermal runaway — heater high but air not rising (probe out of airflow) -> FAILSAFE
// `heaterPct` is what the control layer intends to apply this step (needed for
// the runaway check). Call once per control loop.
SafetyVerdict safetyCheck(const Temps& t, uint8_t heaterPct);

// Call whenever a valid master heartbeat is seen (MQTT heartbeat / set/* write).
void safetyNoteHeartbeat();
