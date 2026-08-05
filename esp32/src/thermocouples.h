#pragma once

// Bean Temp (BT) and Environment Temp (ET) in degrees C.
// NAN means the probe read failed / is not wired.
struct Temps {
  float bt;
  float et;
};

void  thermocouplesBegin();
Temps thermocouplesRead();
