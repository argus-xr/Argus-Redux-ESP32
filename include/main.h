#ifndef MAIN_H
#define MAIN_H

#include <WiFi.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// IMU data structure (raw int16_t)
struct __attribute__((packed)) IMUSample {
  uint32_t timestamp_us;
  int16_t accel[3];
  int16_t gyro[3];
};

// Packet header structure
struct __attribute__((packed)) PacketHeader {
  uint32_t camera_timestamp_start;
  uint32_t camera_timestamp_end;
  uint16_t battery_mv;
  uint8_t imu_count;
  uint32_t image_size;
};

void sendPacket(camera_fb_t *frame);

extern IMUSample imu_buffer[MAX_IMU_SAMPLES];
extern uint8_t imu_index;

#endif // MAIN_H
