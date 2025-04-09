#include "imu.h"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

IMU::IMU() : imuQueue(nullptr), imuTaskHandle(nullptr), isRunning(false) {

}

IMU::~IMU() {
    stop(); // Ensure the task is stopped before deleting the queue
    if (imuQueue) {
        vQueueDelete(imuQueue);
    }
}

void IMU::init() {
    if (Wire.begin(GPIO_NUM_15, GPIO_NUM_13)) {
        imu.initialize();
        if (!imu.testConnection()) {
            Serial.println("MPU6050 connection failed");
        } else {
            Serial.println("MPU6050 connected");
            imu.setSleepEnabled(false); // Prevent sleeping.
        }
    } else {
        Serial.println("Wire.begin() failed");
    }
    if (!imuQueue) {
        imuQueue = xQueueCreate(MAX_IMU_SAMPLES, sizeof(IMUSample));
        if (imuQueue == NULL) {
            Serial.println("Error creating IMU queue");
        }
    }
}

void IMU::imuTaskEntryPoint(void *param) {
    IMU* self = static_cast<IMU*>(param);
    self->imuTask();
}

void IMU::imuTask() {
    Serial.println("IMU task started");
    logNextSuccessfulRead = true;
    while (true) {
        if (isRunning) {
            bufferIMUSample();
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms if we should start again
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
    }
}

void IMU::bufferIMUSample() {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getAcceleration(&ax, &ay, &az);
    imu.getRotation(&gx, &gy, &gz);

    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
        // Error
        logNextSuccessfulRead = true;
        return;
    } else if (logNextSuccessfulRead) {
        Serial.println("IMU returned valid data");
        logNextSuccessfulRead = false;
    }

    IMUSample sample;
    sample.timestampUs = esp_timer_get_time();
    sample.accel[0] = ax;
    sample.accel[1] = ay;
    sample.accel[2] = az;
    sample.gyro[0] = gx;
    sample.gyro[1] = gy;
    sample.gyro[2] = gz;

    if (xQueueSend(imuQueue, &sample, 0) != pdTRUE) {
        Serial.println("IMU Queue full, dropping sample");
    }
}

uint8_t IMU::getSamples(IMUSample* buffer, uint8_t requestedCount) {
    uint8_t count = 0;
    IMUSample sample;
    while (count < requestedCount && xQueueReceive(imuQueue, &sample, 0) == pdTRUE) {
        buffer[count++] = sample;
    }
    return count; // Return the actual number of samples copied
}

uint8_t IMU::getSampleCount() {
    UBaseType_t count = uxQueueMessagesWaiting(imuQueue);
    return (uint8_t)count;
}

void IMU::start() {
    if (!imuTaskHandle) {
        xTaskCreatePinnedToCore(&IMU::imuTaskEntryPoint, "IMU Task", 4096, this, 5, &imuTaskHandle, 0);
    }
    isRunning = true;
}

void IMU::stop() {
    isRunning = false;
    if (imuTaskHandle) {
        vTaskDelete(imuTaskHandle);
        imuTaskHandle = nullptr;
    }
    if (imuQueue) {
        xQueueReset(imuQueue); // Clear the queue
    }
}
