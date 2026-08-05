#pragma once
#include "thermocouples.h"

void safetyBegin();

// Returns false if it is NOT safe to keep heating -> caller must fail safe.
// Independent of the master (Path 2: ESP is a dumb actuator, so it must guard
// itself). Trips on any of:
//   * comms watchdog  — no heartbeat within SAFETY_COMMS_TIMEOUT_MS
//   * over-temp       — BT >= SAFETY_MAX_BT_C
//   * dead probe      — BT NaN for SAFETY_BADPROBE_MAX consecutive reads
//   * thermal runaway — heater high but BT not rising (probe out of bean mass)
// `heaterPct` is what the control layer intends to apply this step (needed for
// the runaway check). Call once per control loop.
bool safetyOk(const Temps& t, uint8_t heaterPct);

// Call whenever a valid master heartbeat is seen (MQTT heartbeat / set/* write).
void safetyNoteHeartbeat();
