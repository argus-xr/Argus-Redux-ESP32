#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "IMU.h"
#include "camera.h"

class SensorManager {
public:
    SensorManager(IMU& imu, CameraClass& camera);
    ~SensorManager();
    void startSensors();
    void stopSensors();
    bool isImuRunning() const;
    bool isCameraRunning() const;

private:
    IMU& imu;
    CameraClass& camera;
    bool imuRunning;
    bool cameraRunning;
};

#endif // SENSOR_MANAGER_H
