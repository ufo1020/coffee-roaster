// coffee-roaster ESP32 controller — entry point.
//
// Roles:
//   * read BT/ET thermocouples
//   * publish them + accept setpoints over MQTT (roaster-ui is the controller)
//   * run the control loop (MANUAL passthrough or local PID) at CONTROL_LOOP_HZ
//   * enforce safety locally (over-temp + comms watchdog) -> fail-safe heater OFF
//
// Everything hardware-specific is gated on real pins being set in config.h;
// with the default placeholder pins (-1) this builds and runs but drives nothing.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>          // esp_wifi_set_ps — force power-save fully OFF
#include "config.h"
#include "secrets.h"
#include "thermocouples.h"
#include "roaster_io.h"
#include "control.h"
#include "safety.h"
#include "comm.h"
#include "ota.h"

static uint32_t lastControlUs = 0;
static uint32_t lastTelemetryUs = 0;
static uint32_t lastSampleUs = 0;

// WiFi event handler: the AP can deauth us or the link can glitch at any time.
// Without this the board would associate once at boot and then sit offline
// forever if the link ever dropped (Artisan + OTA both unreachable). Kick a
// reconnect whenever we lose the STA connection.
static void onWifiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // Re-assert power-save OFF on every (re)connect. The driver re-enables
      // modem-sleep across an association, so setting it once at boot isn't
      // enough — do it here, after the link is actually up, or unicast gets
      // buffered to DTIM intervals (huge latency + loss despite good signal).
      esp_wifi_set_ps(WIFI_PS_NONE);
      Serial.printf("[wifi] up: %s (ps=none)\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[wifi] disconnected — reconnecting");
      WiFi.reconnect();
      break;
    default:
      break;
  }
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.onEvent(onWifiEvent);
  WiFi.setAutoReconnect(true);     // driver-level retry, backs up onWifiEvent
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
  // Disable modem-sleep: default power-save buffers/drops incoming unicast
  // between DTIM beacons, which shows up as heavy ping/Modbus packet loss even
  // with a strong signal and a stable association. On arduino-esp32 3.x
  // WiFi.setSleep(false) alone is unreliable, so also call the IDF primitive.
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("\nWiFi up: %s\n", WiFi.localIP().toString().c_str());
}

// Heartbeat / status LED: blinks slowly when WiFi is up (alive & connected),
// fast when disconnected. Driven from loop() with no delay()s.
static void updateStatusLed() {
  if (PIN_STATUS_LED < 0) return;
  const bool connected = WiFi.status() == WL_CONNECTED;
  const uint32_t period = connected ? 1000 : 150;   // ms; fast blink = no WiFi
  static uint32_t lastToggleMs = 0;
  static bool on = false;
  const uint32_t nowMs = millis();
  if (nowMs - lastToggleMs >= period) {
    lastToggleMs = nowMs;
    on = !on;
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\ncoffee-roaster controller booting  fw %s (%s)\n", FW_VERSION, FW_BUILD);

  roasterIoBegin();        // outputs to a safe state FIRST (heater off)
  safetyBegin();
  thermocouplesBegin();
  connectWifi();
  otaBegin();              // wireless firmware updates (espota :3232)
  commBegin();             // MQTT: control setpoints + telemetry publish
  controlBegin();

  Serial.println("ready");
}

void loop() {
  otaHandle();             // check for a pending wireless update
  commLoop();              // service MQTT + publish telemetry
  updateStatusLed();       // heartbeat: slow blink = WiFi up, fast = disconnected

  // Periodic link-quality print (diagnostic): RSSI in dBm. > -67 good,
  // -67..-75 marginal, < -80 unreliable.
  static uint32_t lastRssiMs = 0;
  if (WiFi.status() == WL_CONNECTED && millis() - lastRssiMs > 3000) {
    lastRssiMs = millis();
    Serial.printf("[wifi] RSSI %d dBm\n", WiFi.RSSI());
  }

  const uint32_t now = micros();

  // BT sampling: feed the trimmed-mean ring at ~4 Hz (>= the MAX6675 conversion
  // time). Decoupled from telemetry so the filter has enough real samples to trim.
  if (now - lastSampleUs >= TC_SAMPLE_MS * 1000UL) {
    lastSampleUs = now;
    thermocouplesSample();
  }

  // Telemetry: refresh temps into the input registers (published over MQTT).
  if (now - lastTelemetryUs >= 1000000UL / TELEMETRY_HZ) {
    lastTelemetryUs = now;
    Temps t = thermocouplesRead();
    controlUpdateTemps(t);       // feeds RoR + publishes to input regs
  }

  // Control loop: MANUAL passthrough or PID, then apply outputs.
  if (now - lastControlUs >= 1000000UL / CONTROL_LOOP_HZ) {
    lastControlUs = now;
    ControlOutputs out = controlStep();   // decides heater/fan
    if (!safetyOk(controlCurrentTemps(), out.heaterPct)) {
      out = controlFailSafe();            // heater 0%, keep fan for cool-down
    }
    roasterIoApply(out);
  }

  // Yield 1 ms so the WiFi/lwIP service task gets CPU. Without this the loop
  // spins flat-out and starves networking, which shows up as multi-second,
  // bursty TCP/ICMP latency and heavy loss even on a strong link. Our control
  // loop is time-gated above (10 Hz), so a 1 ms sleep costs nothing.
  delay(1);
}
