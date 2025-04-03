// Platform: ESP32-Cam using PlatformIO
// Features: MPU6050 IMU buffering, camera capture, timestamped UDP packets, battery monitoring, image+IMU bundling with fallback, multi-core camera capture, raw int16_t IMU data

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <MPU6050.h>
#include <soc/rtc.h>
#include <esp_heap_caps.h>
#include "camera.h"
#include "main.h"
#include <Preferences.h>
#include "network.h"
#include "config.h"

// --- GLOBAL VARIABLES ---
IMUSample imuBuffer[MAX_IMU_SAMPLES];
uint8_t imuIndex = 0;
MPU6050 imu;

// --- FUNCTION DEFINITIONS ---


void setupIMU() {
    if (Wire.begin()) {
        imu.initialize();
        if (!imu.testConnection()) {
            Serial.println("MPU6050 connection failed");
        } else {
            Serial.println("MPU6050 connected");
        }
    } else {
        Serial.println("Wire.begin() failed");
    }
}

uint16_t readBatteryMv() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = raw / 4095.0f * 3.3f * 2.0f * BATTERY_CALIBRATION_FACTOR;
    return static_cast<uint16_t>(voltage * 1000);
}

void bufferIMUSample() {
    if (imuIndex >= MAX_IMU_SAMPLES) {
        return; // Buffer full, don't add more
    }
    int16_t ax, ay, az, gx, gy, gz;
    imu.getAcceleration(&ax, &ay, &az);
    imu.getRotation(&gx, &gy, &gz);

    IMUSample &sample = imuBuffer[imuIndex];
    sample.timestampUs = esp_timer_get_time();
    sample.accel[0] = ax;
    sample.accel[1] = ay;
    sample.accel[2] = az;
    sample.gyro[0] = gx;
    sample.gyro[1] = gy;
    sample.gyro[2] = gz;
    imuIndex++;
}

void sendPacket(camera_fb_t *frame) {
    Network::startMessage(Network::MessageType::SENSOR_DATA);

    PacketHeader header;
    if (frame) {
        header.cameraTimestampStart = Camera::frameTimestampStart;
        header.cameraTimestampEnd = Camera::frameTimestampEnd;
        header.imageSize = frame->len;
    }
    else {
        header.cameraTimestampStart = 0;
        header.cameraTimestampEnd = 0;
        header.imageSize = 0;
    }
    
    header.batteryMv = readBatteryMv();
    header.imuCount = imuIndex;

    Network::writePayloadChunk((uint8_t*)&header, sizeof(header));
    Network::writePayloadChunk((uint8_t*)imuBuffer, imuIndex * sizeof(IMUSample));
    if (frame) {
        Network::writePayloadChunk(frame->buf, frame->len);
    }
    Network::endMessage();

    imuIndex = 0; // Reset IMU buffer index after sending
    xSemaphoreGive(Camera::frameHandled); // Signal that the frame has been handled
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
    vTaskDelay(pdMS_TO_TICKS(4000)); // allow time for serial output before boot stuff

    Serial.begin(115200);
    Serial.println("HELLO THERE");

    Serial.println("Connecting to WiFi...");
    Network::startNetworkTasks();
    Serial.println("WiFi connected!");

    /*Serial.println("Setting up IMU...");
    setupIMU();
    Serial.println("IMU setup complete.");*/

#if CAMERA_ENABLED
    Serial.println("Initializing camera...");
    Camera::initCamera();
    Serial.println("Camera initialized!");

    Serial.println("Starting camera task...");
    xTaskCreatePinnedToCore(Camera::cameraTask, "Camera Task", 8192, NULL, 5, NULL, 0); // Camera task on core 0, priority 5
    Serial.println("Camera task started.");

    Serial.println("Creating camera semaphore...");
    Camera::frameReady = xSemaphoreCreateBinary(); // Create the binary semaphore
    if (Camera::frameReady == NULL) {
        Serial.println("Error creating camera semaphore");
    } else {
        Serial.println("Camera semaphore created.");
    }
#endif

    Serial.println("Setup complete.");
}

void loop() {
    //bufferIMUSample();
    
    if (xSemaphoreTake(Camera::frameReady, 0) == pdTRUE) { // Check if a frame is ready without blocking
        sendPacket(Camera::capturedFrame);
    }
    else if (imuIndex >= MAX_IMU_SAMPLES) {
        sendPacket(nullptr);
    }
}
