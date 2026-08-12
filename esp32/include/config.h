#pragma once
// =============================================================================
//  coffee-roaster ESP32 — build-time configuration
//
//  >>> ALL HARDWARE PINS BELOW ARE PLACEHOLDERS (-1). <<<
//  Set each to the real GPIO once the wiring is decided. Any pin left at -1 is
//  treated as "not wired" by the firmware and its feature is skipped.
// =============================================================================

// ---------------------------------------------------------------------------
//  Thermocouple — MAX6675 (K-type), Bean Temp only. Read-only SPI, no MOSI.
//  Only BT is wired; ET CS stays -1 (disabled) → ET reads NAN.
//  VSPI defaults; avoid strapping pins 0/2/12/15 and GPIO13 (PIN_FAN_PWM).
// ---------------------------------------------------------------------------
#define PIN_TC_SCK     18   // shared SPI clock (SCK)
#define PIN_TC_MISO    19   // SPI data-out (SO)
#define PIN_TC_CS_BT    5   // chip-select, Bean Temp probe
#define PIN_TC_CS_ET   -1   // Environment Temp probe not wired (disabled)

// ---------------------------------------------------------------------------
//  Heater — Delixi CDG1-1DA 25A SSR (DC input / AC load, zero-cross type).
//  The SSR does zero-cross switching internally, so we don't need a mains
//  zero-cross detector: driving the gate with SLOW PWM (whole-cycle burst
//  control, see HEATER_PWM_PERIOD_MS) lets the SSR snap each edge to the next
//  zero-cross on its own. Proportional power = fraction of the window ON.
//
//  Wiring: GPIO27 -> (optional NPN, else direct) -> SSR input+ ; SSR input- to
//  GND. 10k pull-down on the GPIO holds the heater OFF during boot/reset.
//  Non-inverting: HIGH = heater on.
// ---------------------------------------------------------------------------
#define PIN_HEATER_SSR   27  // SSR gate (output, non-inverting: HIGH = on)
#define PIN_ZERO_CROSS   -1  // not needed: SSR is internally zero-cross

// Slow-PWM window for burst (integral-cycle) control of the zero-cross SSR.
// This is NOT phase control — the SSR passes whole mains cycles, so power is set
// by the fraction of the window the gate is ON. Shorter window = faster bursts =
// less visible flicker on a bare lamp and smaller element temperature ripple,
// at the cost of coarser steps. 250 ms @ CONTROL_LOOP_HZ=10 gives ~2.5 % steps
// and blinks 4x/s. The bean mass + airflow thermally average the bursts regardless.
#define HEATER_PWM_PERIOD_MS  250

// ---------------------------------------------------------------------------
//  Airflow  (air/fluid-bed roaster — beans are tumbled by the blower, no drum)
//
//  Fan is a BRUSHLESS blower on an RC ESC (the round "H SKY" board potted in
//  the hub). It is NOT a plain PWM DC fan: it wants a servo pulse train, the
//  same signal the "SKY V3" dialer generated from its pot.
//    * Wire PIN_FAN_PWM -> the ESC's "S" (signal) pad, and tie GND together.
//    * We emit 50 Hz, PULSE_MIN..PULSE_MAX us; width maps to fan 0..100 %.
//    * At boot we hold PULSE_MIN for FAN_ARM_MS so the ESC arms (won't spin
//      until it sees min throttle first).
// ---------------------------------------------------------------------------
#define PIN_FAN_PWM    13    // ESC signal (servo pulse output) -> ESC "S" pad

// RC ESC servo-signal timing for the fan, calibrated to THIS ESC (H SKY):
//   * < ~1100 us : stopped
//   * ~1100 us   : cogging (stalls & restarts) — unusable
//   * ~1140 us   : lowest steady speed
//   * ~1850 us   : true max (no faster above this; 1850..2000 all identical)
// So the ESC's useful band is compressed into 1140..1850 us. We map fan % onto
// that band (see roaster_io.cpp): 0 % = fully OFF (below stall), and 1..100 %
// spreads across FAN_RUN_MIN_US..FAN_PULSE_MAX_US so the whole slider does
// something. The curve is still nonlinear (most change is in the low end) —
// that's the ESC, not the code.
// SAFETY INTERLOCK: on an air roaster the heater must NEVER run without airflow
// or the element overheats / beans scorch. The firmware refuses to drive the
// heater unless the fan is commanded at least this %. Enforced in roaster_io.cpp
// at the lowest level, so it holds regardless of control mode or what the master
// commands.
#define HEATER_MIN_FAN_PCT  20

#define FAN_ESC_HZ        50     // servo frame rate
#define FAN_PULSE_MIN_US  1000   // 0 % / OFF (also the arming level) — motor stopped
#define FAN_RUN_MIN_US    1140   // fan 1 %  = lowest steady speed (just above cogging)
// fan 100 %: the ESC's TRUE max is ~1850 us. 1350 was too weak on real HW (2026-07-24
// — 100 % airflow insufficient for roasting), so stepping the cap up toward 1850.
// fanPctToUs spreads 1..100 % across FAN_RUN_MIN_US..FAN_PULSE_MAX_US, so raising this
// widens the whole band (every % gets more air). Conservative first step: 1500; raise
// further (1650, 1800…) if 100 % is still weak. NB profile fan_curves are in %, so they
// auto-rescale — re-check curves after each bump via the export→review workflow.
#define FAN_PULSE_MAX_US  1850   // fan 100 % = ESC TRUE max (1850..2000 all identical)
#define FAN_ARM_MS        3000   // hold min throttle this long at boot to arm

// ---------------------------------------------------------------------------
//  Local UI / safety indicators (optional)
// ---------------------------------------------------------------------------
#define PIN_STATUS_LED 2     // onboard LED (GPIO2 on most ESP32 devkits): heartbeat / fault
#define PIN_BUZZER     -1    // TODO: alarm buzzer

// ---------------------------------------------------------------------------
//  Network
// ---------------------------------------------------------------------------
#define WIFI_HOSTNAME     "coffee-roaster-esp32"
// WiFi + MQTT credentials live in secrets.h (copy secrets.h.example). Not committed.

// ---------------------------------------------------------------------------
//  MQTT topics. Base MUST stay under coffee-roaster-esp32/ — the broker ACL
//  (docker/mosquitto/config/acl) scopes the 'roaster' user to that tree.
//
//  Firmware PUBLISHES:
//    <base>/status     JSON telemetry (bt, et, ror, heater, state) ~TELEMETRY_HZ
//    <base>/available  retained LWT: "online" / "offline"
//  Firmware SUBSCRIBES (any client — Artisan or HA — may write these):
//    <base>/set/heater      heater setpoint %   (MANUAL mode)
//    <base>/set/fan         fan setpoint %
//    <base>/set/mode        0=MANUAL, 1=PID
//    <base>/set/pid_target  PID target C (float, e.g. 210.5)
//    <base>/heartbeat       comms watchdog ping (any payload); set/* also count
// ---------------------------------------------------------------------------
#define MQTT_BASE         "coffee-roaster-esp32"
#define MQTT_CLIENT_ID    "coffee-roaster-esp32"
#define MQTT_TOPIC_STATUS     MQTT_BASE "/status"
#define MQTT_TOPIC_AVAIL      MQTT_BASE "/available"
#define MQTT_TOPIC_VERSION    MQTT_BASE "/version"   // retained; published on connect
#define MQTT_TOPIC_SET_WILD   MQTT_BASE "/set/+"
#define MQTT_TOPIC_HEARTBEAT  MQTT_BASE "/heartbeat"

// ---------------------------------------------------------------------------
//  Firmware version. No git in this tree, so this is a hand-bumped date-stamp
//  (bump it when you flash something you want to tell apart). __DATE__/__TIME__
//  is a free compiler stamp so every build is still uniquely identifiable even
//  if the version literal wasn't bumped. Published retained to <base>/version on
//  every (re)connect: any client (roaster-ui, Artisan, HA, mosquitto_sub) gets it
//  instantly on subscribe — that's the "query the firmware version" mechanism.
// ---------------------------------------------------------------------------
#ifndef FW_VERSION                         // overridable via -DFW_VERSION in platformio.ini
#define FW_VERSION        "2026.08.12"
#endif
#define FW_BUILD          (__DATE__ " " __TIME__)

// ---------------------------------------------------------------------------
//  Control / safety tuning (placeholder values — tune on real hardware)
// ---------------------------------------------------------------------------
#define CONTROL_LOOP_HZ        10     // PID / output update rate
#define TELEMETRY_HZ           2      // how often temps refresh the registers
// BT filter sampling: the MAX6675 needs ~170-220 ms/conversion, so sampling faster
// than this just re-reads the same value. 250 ms = ~4 Hz feeds the trimmed-mean ring
// (thermocouples.cpp) with real samples while telemetry still publishes at 2 Hz.
#define TC_SAMPLE_MS           250
// RoR window: a single sample-to-sample delta divided by ~0.5 s is dominated by
// MAX6675 jitter (measured raw RoR swung -372..+1600 C/min, 21% sign-flips on a
// real roast — 2026-07-26). Compute the slope over a sliding time window instead:
// 30 s collapses that to a clean curve (stdev ~28, ~2% flips) with minimal lag.
#define ROR_WINDOW_MS          30000  // BT slope measured over this trailing window
#define SAFETY_MAX_BT_C        230.0f // hard cutoff: cut heater above this BT
#define SAFETY_COMMS_TIMEOUT_MS 2000  // no master heartbeat -> fail safe (heater off).
                                      // Tight: master (HA/Node-RED/script) must ping
                                      // ~1 Hz. Path 2 = ESP is a dumb actuator, so a
                                      // dead master must cut heat fast.
#define HEATER_MAX_PCT         100    // clamp

// --- Independent overheat / fault guards (ESP never trusts the master) -------
// Dead-probe: BT NaN (unplugged/faulty MAX31855) for this many consecutive
// control-loop reads -> fail safe. At CONTROL_LOOP_HZ=10, 20 reads = ~2 s.
#define SAFETY_BADPROBE_MAX     20
// Thermal-runaway: if the heater is commanded at/above this % but BT fails to
// rise at least SAFETY_RUNAWAY_MIN_RISE_C within SAFETY_RUNAWAY_WINDOW_MS, the
// probe likely fell out of the bean mass (reads cool while the element cooks)
// -> fail safe. Tune once real thermal response is known.
#define SAFETY_RUNAWAY_HEAT_PCT     60
// 2026-07-24: 3 C / 30 s was too tight — it tripped FAULT on a normal light-roast
// approach to target (BT legitimately flattens near the setpoint while heater is
// still >=60%), plus the noisy MAX6675 scatters a real slow rise to ~0. Loosened
// to 2 C / 60 s: still catches a true probe-out-of-mass runaway (heater hard, BT
// dead-flat for a full minute) but tolerates a normal flattening ramp.
#define SAFETY_RUNAWAY_WINDOW_MS    60000   // 60 s observation window
#define SAFETY_RUNAWAY_MIN_RISE_C   2.0f    // expect >=2 C rise in that window
// 2026-07-26: the guard also false-tripped at the TOP of a good roast (light2 hit
// 197 C on target at 9 min, then FAULT): near the setpoint BT legitimately flattens
// (<2 C/60 s) while the PID holds heater >=60% settling onto target. The runaway it
// actually protects against is probe-out-of-mass, which reads COOL (~ambient) while
// the element cooks — that danger only exists at low BT. So ARM the guard only below
// this BT; above it the probe is demonstrably in hot beans and SAFETY_MAX_BT_C (230)
// covers a true overshoot. Set below the lowest realistic target so real roasts finish.
#define SAFETY_RUNAWAY_MAX_ARM_C    150.0f  // runaway guard only active while BT < this

// ---------------------------------------------------------------------------
//  Internal state keys (a holdover from the old Modbus register map; now just
//  array indices in mqtt_comm.cpp that map to MQTT topics — see MQTT_TOPIC_*).
//  Scaling: all temperatures are   degrees C x 10   (e.g. 2105 => 210.5 C)
// ---------------------------------------------------------------------------
// Telemetry (firmware -> world; published in <base>/status)
#define IREG_BT          0   // bean temp     (C x10)
#define IREG_ET          1   // env temp      (C x10)
#define IREG_BT_ROR      2   // bean RoR      (C/min x10)
#define IREG_HEATER_ACT  3   // heater actual (%)
#define IREG_STATE       4   // roaster state enum (see control.h)
#define IREG_FAN_ACT     5   // fan actual    (%)
#define IREG_COUNT       6

// Set to 1 only when an ET thermocouple is physically wired. When 0, ET is
// omitted from the status payload instead of publishing a fake 0 C.
#define HAS_ET_PROBE     0

// Setpoints (world -> firmware; written via <base>/set/*)
#define HREG_HEATER_SP   0   // heater setpoint (%) — used in MANUAL mode
#define HREG_FAN_SP      1   // fan setpoint    (%)
#define HREG_MODE        2   // 0=MANUAL, 1=PID
#define HREG_PID_TARGET  3   // PID target temp (C x10) — used in PID mode
#define HREG_COUNT       4   // (heartbeat is now its own topic, not a register)
