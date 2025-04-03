
#include <Arduino.h>
#include "network.h"

WiFiUDP udp;

void connectWiFi() {
    Serial.println("🔧 Starting WiFi setup...");
  
    WiFi.mode(WIFI_STA); // Station mode
    delay(1000);         // Let WiFi stack settle
    Serial.println("✅ WiFi STA mode set");
  
    WiFiManager wm;
    wm.setConfigPortalTimeout(180); // Timeout for captive portal (3 minutes)
  
    // Optional: Uncomment to force setup mode every boot during development
    // wm.resetSettings();
  
    bool connected = wm.autoConnect("ArgusSetup");
  
    if (!connected) {
      Serial.println("⛔ Failed to connect or configure WiFi. Rebooting...");
      delay(3000);
      ESP.restart();
    }
  
    Serial.println("✅ WiFi connected!");
    Serial.print("📍 IP Address: ");
    Serial.println(WiFi.localIP());
  }