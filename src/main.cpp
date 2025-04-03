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

// --- GLOBAL VARIABLES ---
IMUSample imu_buffer[MAX_IMU_SAMPLES];
uint8_t imu_index = 0;
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
  float voltage = raw / 4095.0f * 3.3f * 2.0f;
  return static_cast<uint16_t>(voltage * 1000);
}

void bufferIMUSample() {
  if (imu_index >= MAX_IMU_SAMPLES) return; // Buffer full, don't add more
  int16_t ax, ay, az, gx, gy, gz;
  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  IMUSample &sample = imu_buffer[imu_index];
  sample.timestamp_us = esp_timer_get_time();
  sample.accel[0] = ax;
  sample.accel[1] = ay;
  sample.accel[2] = az;
  sample.gyro[0] = gx;
  sample.gyro[1] = gy;
  sample.gyro[2] = gz;
  imu_index++;
}

void sendPacket(camera_fb_t *frame) {
  PacketHeader header;
  if (frame) {
    header.camera_timestamp_start = Camera::frame_timestamp_start;
    header.camera_timestamp_end = Camera::frame_timestamp_end;
  }
  else {
    header.camera_timestamp_start = 0;
    header.camera_timestamp_end = 0;
  }
  header.battery_mv = readBatteryMv();
  header.imu_count = imu_index;
  header.image_size = frame ? frame->len : 0;

  udp.beginPacket(DEST_IP, DEST_PORT);
  udp.write((uint8_t*)&header, sizeof(header));
  udp.write((uint8_t*)imu_buffer, imu_index * sizeof(IMUSample));
  if (frame) {
    udp.write(frame->buf, frame->len);
  }
  udp.endPacket();

  imu_index = 0; // Reset IMU buffer index after sending
  xSemaphoreGive(Camera::frame_handled); // Signal that the frame has been handled
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  delay(4000); // allow time for serial output before boot stuff

  Serial.begin(115200);
  Serial.println("HELLO THERE");

  Serial.println("Connecting to WiFi...");
  connectWiFi();
  Serial.println("WiFi connected!");

  /*Serial.println("Setting up IMU...");
  setupIMU();
  Serial.println("IMU setup complete.");*/

#if CAMERA_ENABLED
  Serial.println("Initializing camera...");
  Camera::initCamera();
  Serial.println("Camera initialized!");

  Serial.println("Starting camera task...");
  xTaskCreatePinnedToCore(Camera::cameraTask, "Camera Task", 8192, NULL, 2, NULL, 0); // Camera task on core 0, priority 2
  Serial.println("Camera task started.");

  Serial.println("Creating camera semaphore...");
  Camera::frame_ready = xSemaphoreCreateBinary(); // Create the binary semaphore
  if (Camera::frame_ready == NULL) {
    Serial.println("Error creating camera semaphore");
  } else {
    Serial.println("Camera semaphore created.");
  }
#endif

  Serial.println("Setup complete.");
}

void loop() {
  /*bufferIMUSample();
  
  if (xSemaphoreTake(Camera::frame_ready, 0) == pdTRUE) { // Check if a frame is ready without blocking
    sendPacket(Camera::captured_frame);
  }
  else if (imu_index >= MAX_IMU_SAMPLES) {
    sendPacket(nullptr);
  }*/
}
