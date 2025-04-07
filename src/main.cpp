#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <soc/rtc.h>
#include <esp_heap_caps.h>
#include "camera.h"
#include "main.h"
#include <Preferences.h>
#include "network.h"
#include "config.h"
#include "imu.h"
#include "sensormanager.h"

// --- FUNCTION DEFINITIONS ---

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
    vTaskDelay(pdMS_TO_TICKS(4000)); // allow time for serial output before boot stuff

    Serial.begin(115200);
    Serial.println("Main: HELLO THERE");

    Network::startNetworkTasks();

    SensorManager::instance.init();

    Serial.println("Main: Setup complete.");
}

void loop() {
    SensorManager::instance.processSensorData();
}
