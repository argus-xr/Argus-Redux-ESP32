#include "sensors/imu.h"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include "network.h"

IMU::IMU(uint8_t id) : mutex(nullptr), imuTaskHandle(nullptr), isRunning(false), sensorID(id), sampleCount(0) {
    mutex = xSemaphoreCreateMutex();
}

IMU::~IMU() {
    stop();
    if (mutex) {
        vSemaphoreDelete(mutex);
    }
}

void IMU::init() {
    if (Wire.begin(GPIO_NUM_15, GPIO_NUM_13)) {
        imu.initialize();
        if (!imu.testConnection()) {
            Serial.println("MPU6050 connection failed");
        } else {
            Serial.println("MPU6050 connected");
            imu.setSleepEnabled(false); // Prevent sleeping
        }
    } else {
        Serial.println("Wire.begin() failed");
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

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (sampleCount < MAX_IMU_SAMPLES) {
            imuBuffer[sampleCount++] = sample;
        } else {
            Serial.println("IMU buffer full, dropping sample");
        }
        xSemaphoreGive(mutex);
    } else {
        Serial.println("Failed to acquire mutex for IMU buffer");
    }
}

uint8_t IMU::getSamples(IMUSample* buffer, uint8_t requestedCount) {
    uint8_t count = 0;
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = (requestedCount < sampleCount) ? requestedCount : sampleCount;
        if (count > 0) {
            // Copy samples to the output buffer
            memcpy(buffer, imuBuffer, count * sizeof(IMUSample));
            
            // Shift remaining samples to the start of the buffer
            if (count < sampleCount) {
                memmove(imuBuffer, imuBuffer + count, (sampleCount - count) * sizeof(IMUSample));
            }
            sampleCount -= count;
        }
        xSemaphoreGive(mutex);
    }
    
    return count;
}

uint8_t IMU::getSampleCount() {
    uint8_t count = 0;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = sampleCount;
        xSemaphoreGive(mutex);
    }
    return count;
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
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sampleCount = 0;  // Clear the buffer
        xSemaphoreGive(mutex);
    }
}

void IMU::writeSensorDataToPacket() {
    if (!isRunning) {
        return; // Don't try to write data if we're stopped/stopping
    }

    uint8_t count;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    count = sampleCount;
    
    if (count == 0) {
        xSemaphoreGive(mutex);
        return; // No data to send
    }

    // Calculate payload size
    size_t payloadSize = sizeof(IMUHeader) + count * sizeof(IMUSample);

    // Start the network message
    if (!Network::startMessageToHost(Network::MessageType::SENSOR_DATA)) {
        Serial.printf("WARN: IMU (%u) Failed to start network message.\n", sensorID);
        xSemaphoreGive(mutex);
        return;
    }

    // Write packet header: SensorID, PayloadLength
    Network::writeInt<uint8_t>(sensorID);
    Network::writeInt<uint16_t>((uint16_t)payloadSize);

    // Write payload part 1: IMUHeader
    IMUHeader imuHeader;
    imuHeader.sampleCount = count;
    Network::writeStruct(imuHeader);

    // Write payload part 2: Samples
    Network::writePayloadChunk((uint8_t*)imuBuffer, count * sizeof(IMUSample));
    
    // Clear the buffer after sending
    sampleCount = 0;
    xSemaphoreGive(mutex);

    // Finalize and send the message (includes CRC)
    Network::endMessage();
    Serial.printf("INFO: IMU (%u) Sent packet with %u samples.\n", sensorID, count);
}
