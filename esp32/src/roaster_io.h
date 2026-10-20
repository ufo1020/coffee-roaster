#pragma once
#include <stdint.h>

// Commanded outputs to the roaster hardware.
struct ControlOutputs {
  uint8_t heaterPct;  // 0..100
  uint8_t fanPct;     // 0..100
};

void roasterIoBegin();                       // set all outputs to a SAFE state
void roasterIoApply(const ControlOutputs& o);  // at CONTROL_LOOP_HZ: decide levels
void roasterIoHeaterTick();                    // every loop() pass: time the SSR gate
