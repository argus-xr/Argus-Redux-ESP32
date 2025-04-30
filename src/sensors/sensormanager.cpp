#include "sensors/SensorManager.h"
#include <Arduino.h>
#include "network.h"
#include "config.h"

// Define the static instance
SensorManager SensorManager::instance;

SensorManager::SensorManager() : imuRunning(false), cameraRunning(false) {
}

SensorManager::~SensorManager() {
    stopSensors();
}

void SensorManager::init() {
    Serial.println("SensorManager: Initializing sensors...");
    imu.init();
    camera.init();
    Serial.println("SensorManager: Sensors initialized.");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Give some time for the camera to initialize
}

void SensorManager::startSensors() {
    Serial.println("SensorManager: Starting sensors...");
    if (!imuRunning) {
        Serial.println("Starting IMU...");
        //imu.start();
        //imuRunning = true;
        Serial.println("IMU started.");
    } else {
        Serial.println("IMU already running.");
    }

    if (!cameraRunning) {
        Serial.println("Starting camera...");
        camera.start();
        cameraRunning = true;
        Serial.println("Camera started.");
    } else {
        Serial.println("Camera already running.");
    }
    Serial.println("SensorManager: Sensors started.");
}

void SensorManager::stopSensors() {
    Serial.println("SensorManager: Stopping sensors...");
    if (imuRunning) {
        Serial.println("Stopping IMU...");
        //imu.stop(); // Stop the IMU task
        //imuRunning = false;
        Serial.println("IMU stopped.");
    } else {
        Serial.println("IMU not running.");
    }

#if CAMERA_ENABLED
    if (cameraRunning) {
        Serial.println("Stopping camera...");
        camera.stop(); // Stop the camera task
        cameraRunning = false;
        Serial.println("Camera stopped.");
    } else {
        Serial.println("Camera not running.");
    }
#endif
    Serial.println("SensorManager: Sensors stopped.");
}

bool SensorManager::isImuRunning() const {
    return imuRunning;
}

bool SensorManager::isCameraRunning() const {
    return cameraRunning;
}

uint16_t SensorManager::readBatteryMv() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = raw / 4095.0f * 3.3f * 2.0f * BATTERY_CALIBRATION_FACTOR;
    return static_cast<uint16_t>(voltage * 1000);
}

void SensorManager::sendPacket(camera_fb_t *frame) {
    // Check if the host is discovered before sending
    if (!Network::isHostDiscovered) {
        if (frame) {
            Serial.println("SensorManager: Frame not sent, host not discovered.");
            xSemaphoreGive(camera.getFrameHandledSemaphore()); // Signal that the frame has been handled, even though it wasn't sent
        }
        return;
    }

    if (Network::startMessageToHost(Network::MessageType::SENSOR_DATA)) {
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
            //Network::writePayloadChunk(frame->buf, frame->len);
        }
        Network::endMessage();
    }

    if (frame) {
        Serial.println("Returning framebuffer");
        xSemaphoreGive(camera.getFrameHandledSemaphore()); // Signal that the frame has been handled
    }
}

void SensorManager::processSensorData() {
    if (xSemaphoreTake(camera.getFrameReadySemaphore(), 0) == pdTRUE) { // Check if a frame is ready without blocking
        camera_fb_t *frame = camera.getCapturedFrame();
        if (!frame) {
            Serial.println("SensorManager: No frame captured.");
            xSemaphoreGive(camera.getFrameHandledSemaphore());
        } else {
            sendPacket(frame);
            //xSemaphoreGive(camera.getFrameHandledSemaphore());
        }
    } else if (imu.getSampleCount() >= MAX_IMU_SAMPLES/2) {
        sendPacket(nullptr);
    }
}

IMU& SensorManager::getIMU() {
    return instance.imu;
}

CameraClass& SensorManager::getCamera() {
    return instance.camera;
}
