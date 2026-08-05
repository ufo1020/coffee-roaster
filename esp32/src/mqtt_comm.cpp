#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "secrets.h"
#include "comm.h"
#include "safety.h"

// MQTT transport for the roaster. Replaces the Modbus server as the live
// control + telemetry path.
//
//   PUBLISH  <base>/status     JSON, ~TELEMETRY_HZ
//            <base>/available  retained LWT ("online"/"offline")
//   SUBSCRIBE <base>/set/+     setpoints (heater/fan/mode/pid_target)
//             <base>/heartbeat comms watchdog ping
//
// Any write to set/* OR heartbeat feeds safetyNoteHeartbeat(), so an actively
// commanding client keeps the watchdog alive even without a dedicated ping —
// but a well-behaved controller (Artisan/HA) should still publish heartbeat so
// heat is cut if it goes silent while merely holding a setpoint.

static WiFiClient   net;
static PubSubClient mq(net);

// --- register-style state, same indices as config.h (transport-agnostic keys)
static uint16_t gHolding[HREG_COUNT];   // world -> firmware setpoints
static int16_t  gInput[IREG_COUNT];     // firmware -> world telemetry

// pid_target arrives as a float string (C); we store it C x10 in HREG_PID_TARGET
// to match the old Modbus scaling that control.cpp already expects.
static void handleSet(const char* leaf, const char* payload) {
  int v = atoi(payload);
  if      (!strcmp(leaf, "heater"))     gHolding[HREG_HEATER_SP] = (uint16_t)constrain(v, 0, 100);
  else if (!strcmp(leaf, "fan"))        gHolding[HREG_FAN_SP]    = (uint16_t)constrain(v, 0, 100);
  else if (!strcmp(leaf, "mode"))       gHolding[HREG_MODE]      = v ? 1 : 0;
  else if (!strcmp(leaf, "pid_target")) gHolding[HREG_PID_TARGET]= (uint16_t)lroundf(atof(payload) * 10.0f);
  else { Serial.printf("[mqtt] unknown set leaf '%s'\n", leaf); return; }
  safetyNoteHeartbeat();   // an active command counts as "controller alive"
}

static void onMessage(char* topic, byte* payload, unsigned int len) {
  char buf[32];
  unsigned int n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  memcpy(buf, payload, n); buf[n] = '\0';

  if (!strcmp(topic, MQTT_TOPIC_HEARTBEAT)) { safetyNoteHeartbeat(); return; }

  // set/<leaf>: find the last path segment.
  const char* slash = strrchr(topic, '/');
  if (slash && strstr(topic, MQTT_BASE "/set/") == topic) handleSet(slash + 1, buf);
}

static void reconnect() {
  // Non-blocking-ish: try once per call; main loop keeps calling commLoop().
  static uint32_t lastTryMs = 0;
  if (mq.connected() || millis() - lastTryMs < 2000) return;
  lastTryMs = millis();

  Serial.print("[mqtt] connecting... ");
  // LWT: broker publishes "offline" (retained) if we drop without a clean quit.
  bool ok = mq.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                       MQTT_TOPIC_AVAIL, 0, true, "offline");
  if (!ok) { Serial.printf("failed rc=%d\n", mq.state()); return; }

  Serial.println("connected");
  mq.publish(MQTT_TOPIC_AVAIL, "online", true);   // retained
  // Retained version stamp so any client learns the running build the moment it
  // subscribes (survives our reconnects too). "<version> (<build date/time>)".
  {
    char ver[64];
    snprintf(ver, sizeof(ver), "%s (%s)", FW_VERSION, FW_BUILD);
    mq.publish(MQTT_TOPIC_VERSION, ver, true);
  }
  mq.subscribe(MQTT_TOPIC_SET_WILD);
  mq.subscribe(MQTT_TOPIC_HEARTBEAT);
}

void commBegin() {
  // Safe defaults so nothing runs until a client commands it.
  for (int i = 0; i < HREG_COUNT; i++) gHolding[i] = 0;   // MANUAL, all off
  for (int i = 0; i < IREG_COUNT; i++) gInput[i]   = 0;

  mq.setServer(MQTT_HOST, MQTT_PORT);
  mq.setCallback(onMessage);
  mq.setBufferSize(256);   // status JSON is small; default 256 is enough
  reconnect();
}

static void publishStatus() {
  if (!mq.connected()) return;
  JsonDocument doc;
  doc["bt"]     = gInput[IREG_BT]     / 10.0f;
#if HAS_ET_PROBE
  doc["et"]     = gInput[IREG_ET]     / 10.0f;   // only when a probe is wired
#endif
  doc["ror"]    = gInput[IREG_BT_ROR] / 10.0f;
  doc["heater"] = gInput[IREG_HEATER_ACT];
  doc["fan"]    = gInput[IREG_FAN_ACT];
  doc["state"]  = gInput[IREG_STATE];
  char out[192];
  size_t n = serializeJson(doc, out, sizeof(out));
  mq.publish(MQTT_TOPIC_STATUS, (const uint8_t*)out, n, false);
}

void commLoop() {
  if (!mq.connected()) reconnect();
  mq.loop();

  static uint32_t lastPubMs = 0;
  const uint32_t periodMs = 1000 / TELEMETRY_HZ;
  if (millis() - lastPubMs >= periodMs) { lastPubMs = millis(); publishStatus(); }
}

void commSetInput(uint16_t reg, int16_t value) {
  if (reg < IREG_COUNT) gInput[reg] = value;
}

uint16_t commGetHolding(uint16_t reg) {
  return reg < HREG_COUNT ? gHolding[reg] : 0;
}
