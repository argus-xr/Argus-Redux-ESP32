#ifndef MAIN_H
#define MAIN_H

#include <WiFi.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// IMU data structure (raw int16_t)
struct __attribute__((packed)) IMUSample {
    uint32_t timestampUs;
    int16_t accel[3];
    int16_t gyro[3];
};

// Packet header structure
struct __attribute__((packed)) PacketHeader {
    uint32_t cameraTimestampStart;
    uint32_t cameraTimestampEnd;
    uint16_t batteryMv;
    uint8_t imuCount;
    uint32_t imageSize;
};

extern IMUSample imuBuffer[MAX_IMU_SAMPLES];
extern uint8_t imuIndex;

#endif // MAIN_H
