#pragma once
#include "thermocouples.h"
#include "roaster_io.h"

// Roaster state machine (published as IREG_STATE in the MQTT status payload).
enum RoasterState {
  STATE_IDLE     = 0,
  STATE_ROASTING = 1,
  STATE_COOLING  = 2,
  STATE_FAULT    = 3,
};

void controlBegin();

// Feed fresh temps in; also updates RoR and publishes temps to input registers.
void controlUpdateTemps(const Temps& t);

// Compute one control step from current mode/setpoints (read from holding regs).
ControlOutputs controlStep();

// Safe outputs used when the safety layer trips (heater off, fan on to cool).
ControlOutputs controlFailSafe();

// Last temps seen by the control layer (for the safety check).
Temps controlCurrentTemps();
