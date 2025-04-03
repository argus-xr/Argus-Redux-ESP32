#ifndef NETWORK_H
#define NETWORK_H

#include <WiFi.h>
#include <WiFiManager.h>

extern WiFiUDP udp;

void connectWiFi();

#endif // NETWORK_H