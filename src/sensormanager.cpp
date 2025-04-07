#include "SensorManager.h"
#include "IMU.h"
#include "camera.h"
#include <Arduino.h>

SensorManager::SensorManager(IMU& imu, CameraClass& camera) : imu(imu), camera(camera), imuRunning(false), cameraRunning(false) {
}

SensorManager::~SensorManager() {
    stopSensors();
}

void SensorManager::startSensors() {
    Serial.println("Starting sensors...");
    if (!imuRunning) {
        Serial.println("Starting IMU...");
        imu.init();
        imu.start(); // Start the IMU task
        imuRunning = true;
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
    Serial.println("Sensors started.");
}

void SensorManager::stopSensors() {
    Serial.println("Stopping sensors...");
    if (imuRunning) {
        Serial.println("Stopping IMU...");
        imu.stop(); // Stop the IMU task
        imuRunning = false;
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
    Serial.println("Sensors stopped.");
}

bool SensorManager::isImuRunning() const {
    return imuRunning;
}

bool SensorManager::isCameraRunning() const {
    return cameraRunning;
}
