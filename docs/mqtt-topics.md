# roaster-ui ⟷ ESP32 — MQTT contract

The ESP32 and roaster-ui talk over **MQTT** (broker: mosquitto on atst). The ESP
publishes telemetry and subscribes to setpoints; roaster-ui is the controller
(host PID). Artisan, if attached, is an **optional read-only logger** on the same
telemetry — it no longer drives the roast.

- **Broker:** mosquitto on atst (`mqtt.lan`), port `1883`, user `roaster`
  (the broker ACL scopes this user to the `coffee-roaster-esp32/` tree).
- **Base topic:** `coffee-roaster-esp32/` (= `MQTT_BASE` in `esp32/include/config.h`).
- **Scaling:** temperatures in the status JSON are already in **°C** (the °C×10
  register scaling is internal to the firmware; it's converted before publish).

Keep this in sync with `esp32/include/config.h` (topics + `IREG_*`/`HREG_*`)
and `esp32/src/mqtt_comm.cpp` (payload shape).

## Firmware PUBLISHES

| Topic | Retained | Payload |
|-------|----------|---------|
| `<base>/status` | no | JSON telemetry, ~`TELEMETRY_HZ` (2 Hz) — see below |
| `<base>/available` | **yes** (LWT) | `online` / `offline` |
| `<base>/version` | **yes** | firmware stamp, e.g. `2026.08.05 (Aug  5 2026 14:23:01)` |

`<base>/status` JSON keys:

| Key | Meaning | Units |
|-----|---------|-------|
| `air` | Air temp: the single probe, ~1 cm above the heater disc in the inlet jet. Reads mostly hot air, not beans. **Was `bt` before FW 2026.10.08.** | °C |
| `air_raw` | Unfiltered `air` chip read (only with `DEBUG_RAW_AIR`; was `bt_raw`) | °C |
| `et` | Environment temp (only if `HAS_ET_PROBE`) | °C |
| `ror` | Rate of rise of `air` (firmware-computed, 30 s window) | °C/min |
| `heater` | Heater actual output | % |
| `fan` | Fan actual output | % |
| `state` | Roaster state enum: 0=idle 1=roast 2=cool 3=fault (any safety action, incl. the over-temp heat hold below) | enum |

## Firmware SUBSCRIBES

Any client may write these (roaster-ui does; Artisan/HA could).

| Topic | Meaning | Range / payload |
|-------|---------|-----------------|
| `<base>/set/heater` | Heater setpoint (MANUAL mode) | `0`..`100` |
| `<base>/set/fan` | Fan setpoint | `0`..`100` |
| `<base>/set/mode` | `0`=MANUAL passthrough, `1`=local PID | `0` / `1` |
| `<base>/set/pid_target` | PID target temp (local-PID mode) | float °C, e.g. `210.5` |
| `<base>/heartbeat` | comms watchdog ping | any payload |

**Heartbeat / safety:** any write to `set/*` OR `heartbeat` feeds the ESP's comms
watchdog. If the controller goes silent for `SAFETY_COMMS_TIMEOUT_MS`, the ESP
**cuts the heater** (fail-safe). roaster-ui publishes `heartbeat` continuously —
keep it running. This watchdog is independent of any host-side logic.

**What each safety guard does to the outputs** (all report `state` 3):

| Guard | Trip | Outputs | Clears |
|-------|------|---------|--------|
| comms watchdog | no `set/*`/`heartbeat` for `SAFETY_COMMS_TIMEOUT_MS` | heater 0, fan 100 % | next heartbeat |
| dead probe | `air` NaN for `SAFETY_BADPROBE_MAX` reads | heater 0, fan 100 % | good read |
| over-temp | `air` ≥ `SAFETY_MAX_AIR_C` (240 °C) | heater 0, **fan as commanded** | `air` < `SAFETY_AIR_RESUME_C` (230 °C) |
| thermal runaway | heater ≥ 60 % but `air` rises < 2 °C in 60 s (only while `air` < 150 °C) | heater 0, fan 100 % | next window |

Before FW 2026.10.08 the over-temp limit was a non-latching 230 °C that also forced the fan
to 100 %. It tripped repeatedly at first crack on normal roasts.

## Control model

roaster-ui runs the **PID on the host** and pushes `set/heater` + `set/fan`, so
the ESP normally stays in **MANUAL** (`mode 0`) passthrough. The ESP's local PID
(`mode 1`) exists as a fallback but is unused in the current setup. Regardless of
mode, the ESP's safety guards (over-temp, dead-probe, thermal-runaway, comms
timeout) always run and can override any commanded output.

## Attaching Artisan (optional)

Point Artisan at the broker as an MQTT source and read `<base>/status` `air` for
logging. Let Artisan compute its **own** ΔBT (Config → Curves → Delta) for
analysis — don't also feed it the firmware `ror`, or you'll double-smooth. Artisan
should **not** publish `set/*` in the current setup (roaster-ui owns control).

## Quick check without hardware

```bash
# watch telemetry
mosquitto_sub -h mqtt.lan -u roaster -P "$MQTT_PASS" -t 'coffee-roaster-esp32/#' -v

# query the running firmware version (retained → arrives immediately)
mosquitto_sub -h mqtt.lan -u roaster -P "$MQTT_PASS" -t coffee-roaster-esp32/version -C 1

# or simulate the whole ESP: tools/fake_esp.py (speaks this same contract)
MQTT_PASS=... python3 tools/fake_esp.py
```
