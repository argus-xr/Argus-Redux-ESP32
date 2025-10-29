#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU6050.h>
#include <esp_timer.h>
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "sensors/sensorbase.h"

// Structure to hold a single IMU sample
struct __attribute__((packed)) IMUSample {
    uint64_t timestampUs;
    int16_t accel[3];
    int16_t gyro[3];
};

struct __attribute__((packed)) IMUHeader {
    uint8_t sampleCount;
};

class IMU : public SensorBase {
public:
    IMU(uint8_t id);
    ~IMU();
    void init();
    uint8_t getSamples(IMUSample* buffer, uint8_t requestedCount);
    uint8_t getSampleCount();
    void start();
    void stop();
    void writeSensorDataToPacket();

private:
    uint8_t sensorID;
    MPU6050 imu;
    void bufferIMUSample();
    void imuTask();
    static void imuTaskEntryPoint(void *param);
    
    // Buffer and synchronization
    IMUSample imuBuffer[MAX_IMU_SAMPLES];
    SemaphoreHandle_t mutex;
    volatile uint8_t sampleCount;
    TaskHandle_t imuTaskHandle;
    bool isRunning;
    bool logNextSuccessfulRead;
};

#endif // IMU_H
