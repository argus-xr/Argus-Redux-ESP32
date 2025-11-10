#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensors/IMU.h"
#include "sensors/camera.h"
#include "config.h"

// Packet header structure
struct __attribute__((packed)) SensorDataMessageHeader {
    uint32_t frameId;
    uint32_t cameraTimestampStart;
    uint32_t cameraTimestampEnd;
    uint16_t batteryMv;
    uint8_t imuCount;
    uint32_t imageSize;
};

// Image chunk message structure
// The actual chunk data will follow this struct in the UDP packet
struct __attribute__((packed)) ImageChunkMessageHeader {
    uint32_t frameId;
    uint32_t startByte;
    uint32_t length;
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
    void sendImageChunks(camera_fb_t *frame);
    uint32_t frameIdCounter;
};

#endif // SENSOR_MANAGER_H
