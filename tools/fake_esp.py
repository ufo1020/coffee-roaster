#!/usr/bin/env python3
"""Fake ESP roaster — feeds simulated temps to the roaster-ui over MQTT.

Speaks exactly what the firmware speaks on the wire so the whole control loop
closes and you can watch the UI drive it:

  publishes : <base>/available   = "online"   (retained, like the LWT)
              <base>/status       = {"bt","et","ror","heater","fan","state"} @ 1 Hz
  subscribes: <base>/set/heater   (0..100)   from the master control loop
              <base>/set/fan      (0..100)
              <base>/set/mode     (ignored; master owns the PID)
              <base>/heartbeat    (logged, so you can see the master is alive)

A lumped thermal model turns the commanded heater/fan into a moving BT, so
AUTO profiles + PID actually settle on target and MANUAL sliders visibly heat
and cool. Mirrors the firmware heater-needs-fan interlock (heat=0 if fan<20%).

Sim modes (--sim):
  realistic  (default) believable fast ramp; PID tracks the curve closely.
  disturb    injects periodic shoves + a wandering heater-efficiency bias so
             BT drifts OFF the target curve — watch the PID fight back.

Usage:
  MQTT_PASS=... python3 fake_esp.py        # creds via env (see MQTT_USER/MQTT_PASS)
  MQTT_HOST=127.0.0.1 python3 fake_esp.py  # override host via env
  python3 fake_esp.py --sim disturb        # kick BT off-curve to stress the PID
  python3 fake_esp.py --start-temp 25 --fault   # start in FAULT to test the banner
  python3 fake_esp.py --no-probe           # publish bt=null (dead-probe look)
"""
import argparse
import json
import math
import os
import signal
import time

import paho.mqtt.client as mqtt

BASE = os.getenv("ROASTER_BASE", "coffee-roaster-esp32")
T_STATUS    = f"{BASE}/status"
T_AVAIL     = f"{BASE}/available"
T_SET_HEAT  = f"{BASE}/set/heater"
T_SET_FAN   = f"{BASE}/set/fan"
T_SET_MODE  = f"{BASE}/set/mode"
T_HEARTBEAT = f"{BASE}/heartbeat"

# must match firmware / app interlock: heater is forced 0 below this fan %
HEATER_MIN_FAN_PCT = 20

# --- lumped thermal model (deg C, seconds) --------------------------------
#   dBT = K_H*(heater/100)*eff - (K_LOSS + K_FAN*fan/100)*(BT - ambient)
# Tuned so full heat @ fan 40% equilibrates ~245 C (headroom above the 230 C
# cutoff) with a punchy-but-believable ramp: ~40 C/min early, tapering as BT
# rises. Crucially K_H now BEATS losses well past roast temp, so BT actually
# REACHES target (the old K_H=0.35 stalled ~195 C — that was the "too slow").
AMBIENT = 22.0
K_H     = 1.2      # heater authority (deg/s at 100%)
K_LOSS  = 0.0030   # passive loss to ambient
K_FAN   = 0.0060   # extra loss per unit fan
NOISE_C = 0.3      # +/- sensor noise band (deg C), realistic thermocouple jitter


class FakeESP:
    def __init__(self, a):
        self.heater = 0          # last commanded (pre-interlock)
        self.fan = 0
        self.bt = a.start_temp
        self.state = 3 if a.fault else 0   # 0 IDLE 1 ROASTING 2 COOLING 3 FAULT
        self.no_probe = a.no_probe
        self.sim = a.sim
        self.hist = []           # (t, bt) for RoR
        self.hb = 0
        self.t = 0.0             # sim clock (s) — drives deterministic disturbances
        self.eff = 1.0           # heater efficiency (disturb mode wanders this)

        self.m = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-esp")
        self.m.username_pw_set(a.user, a.password)
        self.m.will_set(T_AVAIL, "offline", retain=True)   # LWT like the real one
        self.m.on_connect = self._on_connect
        self.m.on_message = self._on_message
        self.host, self.port = a.host, a.port

    def _on_connect(self, c, u, flags, rc, props=None):
        print(f"[fake-esp] connected rc={rc}, base={BASE}")
        c.subscribe([(T_SET_HEAT, 0), (T_SET_FAN, 0), (T_SET_MODE, 0), (T_HEARTBEAT, 0)])
        c.publish(T_AVAIL, "online", retain=True)

    def _on_message(self, c, u, msg):
        p = msg.payload.decode(errors="replace").strip()
        try:
            if msg.topic == T_SET_HEAT:
                self.heater = max(0, min(100, int(float(p))))
            elif msg.topic == T_SET_FAN:
                self.fan = max(0, min(100, int(float(p))))
            elif msg.topic == T_HEARTBEAT:
                self.hb += 1
        except ValueError:
            pass

    def _ror(self, now):
        """deg/min over the last ~30 s of BT history."""
        self.hist = [(t, b) for (t, b) in self.hist if now - t <= 30] + [(now, self.bt)]
        if len(self.hist) < 2:
            return 0.0
        (t0, b0), (t1, b1) = self.hist[0], self.hist[-1]
        return 0.0 if t1 == t0 else (b1 - b0) / (t1 - t0) * 60.0

    def step(self, dt):
        self.t += dt
        # firmware interlock: no airflow => no heat
        heat_eff = 0 if self.fan < HEATER_MIN_FAN_PCT else self.heater

        # disturb mode: make BT wander off the commanded curve so the PID has to
        # correct. Two deterministic (repeatable) disturbances:
        #   * eff  — slow heater-efficiency drift (sine, ~0.75..1.25), a load the
        #            PID sees only through its effect on temperature.
        #   * kick — periodic step shoves (e.g. a cold bean charge / draft).
        eff, kick = 1.0, 0.0
        if self.sim == "disturb":
            eff = 1.0 + 0.25 * math.sin(self.t / 40.0)
            # a downward shove every ~90 s lasting ~8 s
            if (self.t % 90.0) < 8.0:
                kick = -0.5
        self.eff = eff

        loss = (K_LOSS + K_FAN * self.fan / 100.0) * (self.bt - AMBIENT)
        self.bt += dt * (K_H * heat_eff / 100.0 * eff - loss + kick)
        if self.bt < AMBIENT:
            self.bt = AMBIENT
        if self.state != 3:  # don't leave FAULT on its own
            self.state = 1 if heat_eff > 0 else (2 if self.bt > 40 else 0)

    def bt_reading(self):
        """Published BT: true model temp + a little sensor noise (deterministic)."""
        if self.no_probe:
            return None
        noise = NOISE_C * math.sin(self.t * 2.3)   # cheap repeatable jitter
        return round(self.bt + noise, 1)

    def run(self):
        self.m.connect(self.host, self.port, keepalive=30)
        self.m.loop_start()
        last = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                dt, last = now - last, now
                self.step(dt)
                heat_eff = 0 if self.fan < HEATER_MIN_FAN_PCT else self.heater
                status = {
                    "bt": self.bt_reading(),
                    "et": None,
                    "ror": round(self._ror(now), 1),
                    "heater": heat_eff,     # actual, post-interlock (what the SSR sees)
                    "fan": self.fan,
                    "state": self.state,
                }
                self.m.publish(T_STATUS, json.dumps(status))
                dist = f" eff={self.eff:.2f}" if self.sim == "disturb" else ""
                print(f"\rBT={status['bt']}  RoR={status['ror']:+.1f}  "
                      f"heat={heat_eff:3d}%  fan={self.fan:3d}%  "
                      f"state={self.state}  hb={self.hb}{dist}   ", end="", flush=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.m.publish(T_AVAIL, "offline", retain=True)
            self.m.loop_stop()
            self.m.disconnect()
            print("\n[fake-esp] offline, bye")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.getenv("MQTT_HOST", "mqtt.lan"))
    ap.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    ap.add_argument("--user", default=os.getenv("MQTT_USER", "roaster"))
    ap.add_argument("--password", default=os.getenv("MQTT_PASS", ""))   # set via MQTT_PASS or --password (no baked-in secret)
    ap.add_argument("--sim", choices=["realistic", "disturb"], default="realistic",
                    help="realistic: fast ramp, PID tracks; disturb: drift off-curve to stress PID")
    ap.add_argument("--start-temp", type=float, default=AMBIENT, dest="start_temp")
    ap.add_argument("--fault", action="store_true", help="report FAULT (test banner/ready block)")
    ap.add_argument("--no-probe", action="store_true", dest="no_probe",
                    help="publish bt=null (dead-probe look)")
    a = ap.parse_args()
    print(f"[fake-esp] {a.host}:{a.port} user={a.user}  (Ctrl-C to stop)")
    FakeESP(a).run()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    main()
