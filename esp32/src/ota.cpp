#include <Arduino.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "secrets.h"
#include "ota.h"
#include "roaster_io.h"

void otaBegin() {
  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);   // must match --auth in platformio.ini

  // SAFETY: an update pauses the control loop, so force the roaster to a safe
  // state (heater OFF) before flashing. Never leave the element on mid-update.
  ArduinoOTA.onStart([]() {
    roasterIoApply(ControlOutputs{ /*heaterPct*/ 0, /*fanPct*/ 0 });
    Serial.println("[ota] update starting — outputs safe");
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[ota] error %u\n", e); });
  ArduinoOTA.onEnd([]() { Serial.println("[ota] done, rebooting"); });

  ArduinoOTA.begin();
  Serial.println("[ota] ready (espota, :3232)");
}

void otaHandle() {
  ArduinoOTA.handle();
}
