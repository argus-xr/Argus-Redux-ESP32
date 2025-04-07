#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "IMU.h"
#include "camera.h"
#include "config.h"

// Packet header structure
struct __attribute__((packed)) PacketHeader {
    uint32_t cameraTimestampStart;
    uint32_t cameraTimestampEnd;
    uint16_t batteryMv;
    uint8_t imuCount;
    uint32_t imageSize;
};

class SensorManager {
public:
    SensorManager(); // Modified constructor
    ~SensorManager();
    void init();
    void startSensors();
    void stopSensors();
    bool isImuRunning() const;
    bool isCameraRunning() const;
    void sendPacket(camera_fb_t *frame);
    void processSensorData();

    // Static accessors for the IMU and Camera
    static IMU& getIMU();
    static CameraClass& getCamera();

    // Static instance of SensorManager
    static SensorManager instance;

private:
    IMU imu; // IMU is now a member
    CameraClass camera; // Camera is now a member
    bool imuRunning;
    bool cameraRunning;
    uint16_t readBatteryMv();
};

#endif // SENSOR_MANAGER_H
