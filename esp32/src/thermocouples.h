#pragma once

// Air temp (the single probe, ~1 cm above the heater disc in the inlet jet; was
// called "BT" before FW 2026.10.08) and Environment Temp (ET), in degrees C.
// NAN means the probe read failed / is not wired.
struct Temps {
  float air;
  float et;
};

void  thermocouplesBegin();
// Sample the chip into the filter ring. Call at ~4 Hz from loop() (>= the MAX6675
// conversion time); decoupled from thermocouplesRead so the trimmed mean has enough
// real samples despite the 2 Hz telemetry rate.
void  thermocouplesSample();
// Return the current filtered air temp (trimmed mean of the ring). NAN if the probe is
// dead/unwired so the safety debounce still fires.
Temps thermocouplesRead();
// Last UNFILTERED chip read (pre-gate, pre-trim). Debug: compare raw vs filtered.
float thermocouplesRawAir();
