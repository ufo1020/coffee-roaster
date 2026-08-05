#include <Arduino.h>
#include <max6675.h>
#include "config.h"
#include "thermocouples.h"

// Software-CS MAX6675 on the shared hardware SPI bus. Only BT is wired; ET is
// disabled (PIN_TC_CS_ET < 0) and always reads NAN. Constructed only if the
// pins are real; with placeholder pins (-1) reads return NAN.
static MAX6675* tcBT = nullptr;

void thermocouplesBegin() {
  if (PIN_TC_SCK < 0 || PIN_TC_MISO < 0) {
    Serial.println("[tc] SPI pins are placeholders — thermocouples disabled");
    return;
  }
  // MAX6675 has no begin(); the constructor sets up the pins.
  if (PIN_TC_CS_BT >= 0)
    tcBT = new MAX6675(PIN_TC_SCK, PIN_TC_CS_BT, PIN_TC_MISO);
  delay(250); // let the chip settle after power-up
}

// Filter the BT read: the MAX6675 on this rig scatters ±5-10 C sample-to-sample,
// which corrupts RoR and the runaway guard (a real slow rise reads as flat -> false
// FAULT, seen 2026-07-24). Two cheap stages over the TIMED reads (~2 Hz; the chip
// needs ~220 ms/conversion so bursting isn't possible):
//   1) median-of-3 on the last three raw reads -> rejects a lone spike outright
//   2) EMA on the median -> smooths the residual jitter
// NaN (open-thermocouple) is passed through unfiltered so the dead-probe debounce
// in safety.cpp still fires. Pattern mirrors the BBB temp sensor trimmed-mean fix.
#define TC_EMA_ALPHA 0.4f          // 1.0 = no smoothing; lower = smoother/laggier

static float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

Temps thermocouplesRead() {
  Temps t{ NAN, NAN };
  // readCelsius() returns NAN when the open-thermocouple bit is set, which the
  // safety/PID/RoR code already treats as a dead probe. ET is not wired → NAN.
  if (!tcBT) return t;

  const float raw = tcBT->readCelsius();
  if (isnan(raw)) { t.bt = NAN; return t; }   // dead-probe path: don't filter/latch

  static float r1 = NAN, r2 = NAN;   // previous two raw reads for the median window
  static float ema = NAN;            // smoothed output
  float med = (isnan(r1) || isnan(r2)) ? raw : median3(raw, r1, r2);
  r2 = r1; r1 = raw;
  ema = isnan(ema) ? med : (TC_EMA_ALPHA * med + (1.0f - TC_EMA_ALPHA) * ema);
  t.bt = ema;
  return t;
}
