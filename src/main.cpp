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

// --- GLOBAL VARIABLES ---
IMU imu;
CameraClass camera; // Create an instance of the CameraClass

// --- FUNCTION DEFINITIONS ---

uint16_t readBatteryMv() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = raw / 4095.0f * 3.3f * 2.0f * BATTERY_CALIBRATION_FACTOR;
    return static_cast<uint16_t>(voltage * 1000);
}

void sendPacket(camera_fb_t *frame) {
    // Check if the host is discovered before sending
    if (!Network::isHostDiscovered) {
        Serial.println("⚠️ Host not discovered, skipping packet send.");
        if (frame) {
            xSemaphoreGive(camera.getFrameHandledSemaphore()); // Signal that the frame has been handled, even though it wasn't sent
        }
        return;
    }

    Network::startMessage(Network::MessageType::SENSOR_DATA);

    PacketHeader header;
    if (frame) {
        header.cameraTimestampStart = camera.getFrameTimestampStart();
        header.cameraTimestampEnd = camera.getFrameTimestampEnd();
        header.imageSize = frame->len;
    }
    else {
        header.cameraTimestampStart = 0;
        header.cameraTimestampEnd = 0;
        header.imageSize = 0;
    }
    
    header.batteryMv = readBatteryMv();
    uint8_t imuCount = imu.getSampleCount();
    IMUSample imuBuffer[imuCount];
    imuCount = imu.getSamples(imuBuffer, imuCount); // Get the actual number of samples copied
    header.imuCount = imuCount;

    Network::encodeStruct(header);
    Network::writePayloadChunk((uint8_t*)imuBuffer, imuCount * sizeof(IMUSample));
    if (frame) {
        Network::writePayloadChunk(frame->buf, frame->len);
    }
    Network::endMessage();

    if (frame) {
        xSemaphoreGive(camera.getFrameHandledSemaphore()); // Signal that the frame has been handled
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
    vTaskDelay(pdMS_TO_TICKS(4000)); // allow time for serial output before boot stuff

    Serial.begin(115200);
    Serial.println("HELLO THERE");

    Serial.println("Connecting to WiFi...");
    Network::startNetworkTasks();

    Serial.println("Setting up IMU...");
    imu.init();
    Serial.println("IMU setup complete.");

    camera.initCamera();

    Serial.println("Starting camera task...");
    camera.start();
    Serial.println("Camera task started.");

    if (camera.getFrameReadySemaphore() == NULL) {
        Serial.println("Error creating camera semaphore");
    }
    if (camera.getFrameHandledSemaphore() == NULL) {
        Serial.println("Error creating camera handled semaphore");
    }

    Serial.println("Setup complete.");
}

void loop() {
    if (xSemaphoreTake(camera.getFrameReadySemaphore(), 0) == pdTRUE) { // Check if a frame is ready without blocking
        sendPacket(camera.getCapturedFrame());
    } else if (imu.getSampleCount() >= MAX_IMU_SAMPLES/2) {
        sendPacket(nullptr);
    }
}
