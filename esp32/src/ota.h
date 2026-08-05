#pragma once

// ArduinoOTA (espota) wireless firmware updates over WiFi.
// Call otaBegin() after WiFi is up, and otaHandle() every loop.
void otaBegin();
void otaHandle();
