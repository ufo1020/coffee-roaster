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

// Filter the BT read: the MAX6675 on this rig scatters ±5-10 C sample-to-sample
// (EMI pickup, often in short bursts), which corrupts RoR and the runaway guard (a
// real slow rise reads as flat -> false FAULT, seen 2026-07-24). We use a TRIMMED
// MEAN (same pattern as the BBB temp sensor fix): drop the extremes, average the
// rest — this rejects spikes *and* smooths the survivors, where a plain median just
// picks one middle sample and keeps its jitter.
//
// The MAX6675 needs ~170-220 ms per conversion, so it yields at most ~4-5 unique
// values/sec; reading faster just re-returns the same conversion. So we SAMPLE in a
// dedicated 4 Hz path (thermocouplesSample(), driven from loop()) into a ring buffer,
// and thermocouplesRead() — called at the 2 Hz telemetry rate — returns the trimmed
// mean of that ring. Two stages:
//   0) outlier gate — reject any raw sample that jumps > TC_MAX_STEP_C from the last
//      accepted value before it ever enters the ring. Real BT can't move that fast in
//      a 250 ms sample, but EMI can. Zero lag; capped at TC_MAX_REJECT consecutive so
//      a genuine step (bean charge) or a settled new level is eventually accepted.
//   1) trimmed mean — sort the window, drop TC_TRIM lowest + TC_TRIM highest, average
//      what's left. TC_WINDOW=8 @ 4 Hz -> 2 s of history (~1 s effective lag).
//
// NaN (open-thermocouple) flushes the ring and is passed straight through so the
// dead-probe debounce in safety.cpp still fires exactly as before.
#ifndef TC_SAMPLE_MS
#define TC_SAMPLE_MS   250         // ring sampling period (>= chip conversion time)
#endif
#ifndef TC_WINDOW
#define TC_WINDOW      8           // ring size: 8 @ 4 Hz = 2 s of BT history
#endif
#ifndef TC_TRIM
#define TC_TRIM        2           // drop this many lowest AND highest before averaging
#endif
#ifndef TC_MAX_STEP_C
#define TC_MAX_STEP_C  12.0f       // reject raw samples that jump more than this per sample
#endif
#ifndef TC_MAX_REJECT
#define TC_MAX_REJECT  4           // after this many consecutive rejects, accept anyway
#endif

static_assert(TC_WINDOW - 2 * TC_TRIM >= 1, "TC_TRIM too large: nothing left to average");

static float gRing[TC_WINDOW];     // accepted raw samples (oldest..newest not ordered)
static int   gCount   = 0;         // valid entries in gRing (fills to TC_WINDOW)
static int   gHead    = 0;         // next write index (circular)
static float gLast    = NAN;       // last accepted value (for the outlier gate)
static int   gRejects = 0;         // consecutive outlier rejections
static bool  gDeadRaw = false;     // most recent chip read was open-circuit (NaN)

// Sample the chip into the ring. Call at ~TC_SAMPLE_MS from loop(); reading faster
// than the conversion time gains nothing (same value returned).
void thermocouplesSample() {
  if (!tcBT) return;
  const float raw = tcBT->readCelsius();

  if (isnan(raw)) {                // open thermocouple: flush and mark dead
    gDeadRaw = true;
    gCount = 0; gHead = 0; gLast = NAN; gRejects = 0;
    return;
  }
  gDeadRaw = false;

  // 0) outlier gate before the ring — hold the last good value on an implausible
  //    jump, but give up after TC_MAX_REJECT in a row so a real new level lands.
  float use = raw;
  if (!isnan(gLast) && fabsf(raw - gLast) > TC_MAX_STEP_C && gRejects < TC_MAX_REJECT) {
    gRejects++;
    use = gLast;
  } else {
    gRejects = 0;
    gLast = raw;
  }

  gRing[gHead] = use;
  gHead = (gHead + 1) % TC_WINDOW;
  if (gCount < TC_WINDOW) gCount++;
}

Temps thermocouplesRead() {
  Temps t{ NAN, NAN };             // ET is not wired → NAN
  // dead probe: surface NaN immediately so safety.cpp's debounce fires (unchanged).
  if (!tcBT || gDeadRaw || gCount == 0) { t.bt = NAN; return t; }

  // 1) trimmed mean: copy the ring, insertion-sort, drop TC_TRIM from each end.
  float s[TC_WINDOW];
  const int n = gCount;
  for (int i = 0; i < n; i++) s[i] = gRing[i];
  for (int i = 1; i < n; i++) {
    float k = s[i]; int j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  // Trim only when the window is full enough to spare 2*TC_TRIM and still average
  // >=1; while filling, average everything we have.
  int trim = (n - 2 * TC_TRIM >= 1) ? TC_TRIM : 0;
  float sum = 0.0f; int cnt = 0;
  for (int i = trim; i < n - trim; i++) { sum += s[i]; cnt++; }
  t.bt = sum / cnt;
  return t;
}
