#ifndef REMOTE_LOGGER_H
#define REMOTE_LOGGER_H

#include "WiFi.h"
#include "ArduinoHttpClient.h"
#include "TFminiLidar.h"

extern WiFiServer server;

void connectToWiFi();
void handleClient(TFMPlus* frontLidar, TFMPlus* rearLidar);
void updateRFIDData(const String& tagID, unsigned long timestamp);

#endif
