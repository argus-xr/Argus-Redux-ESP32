#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU6050.h>
#include <esp_timer.h>
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "sensors/sensorbase.h"

// Structure to hold a single IMU sample
typedef struct {
    uint64_t timestampUs;
    int16_t accel[3];
    int16_t gyro[3];
} IMUSample;

class IMU : public SensorBase {
public:
    IMU();
    ~IMU();
    void init();
    uint8_t getSamples(IMUSample* buffer, uint8_t requestedCount);
    uint8_t getSampleCount(); // New method to get the current sample count
    QueueHandle_t imuQueue;
    void start();
    void stop();
    SensorData getSensorData();

private:
    MPU6050 imu;
    void bufferIMUSample();
    void imuTask(); // Now a private member function
    static void imuTaskEntryPoint(void *param); // Static wrapper function
    TaskHandle_t imuTaskHandle;
    bool isRunning;
    bool logNextSuccessfulRead;
};

#endif // IMU_H
