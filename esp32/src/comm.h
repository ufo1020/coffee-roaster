#pragma once
#include <stdint.h>

// Neutral comm interface between the control loop and the outside world.
// Backed by MQTT (mqtt_comm.cpp). control.cpp talks only to these helpers and
// stays transport-agnostic — the register indices (HREG_*/IREG_* in config.h,
// a holdover from the old Modbus map) are reused as stable internal keys.

void     commBegin();     // connect WiFi-dependent transport, set safe defaults
void     commLoop();      // service the transport + publish due telemetry

// Input "registers" (firmware -> world): value is raw, same scaling as before
// (temps are C x10). commLoop() batches these into the published status JSON.
void     commSetInput(uint16_t reg, int16_t value);

// Holding "registers" (world -> firmware): last commanded setpoints.
uint16_t commGetHolding(uint16_t reg);
