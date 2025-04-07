#ifndef CAMERA_H
#define CAMERA_H

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Preferences.h>

class CameraClass {
public:
    CameraClass();
    ~CameraClass();
    void initCamera();
    void start();
    void stop(); // Add stopCameraTask() declaration
    void setFrameSize(framesize_t frameSize);
    framesize_t getFrameSize();

    camera_fb_t* getCapturedFrame() const;
    uint32_t getFrameTimestampStart() const;
    uint32_t getFrameTimestampEnd() const;

    SemaphoreHandle_t getFrameReadySemaphore() const;
    SemaphoreHandle_t getFrameHandledSemaphore() const;

private:
    camera_fb_t* capturedFrame;
    uint32_t frameTimestampStart;
    uint32_t frameTimestampEnd;
    uint32_t cameraTimeoutCount;
    framesize_t currentFrameSize;
    bool isRunning;
    TaskHandle_t cameraTaskHandle;

    SemaphoreHandle_t frameReady;
    SemaphoreHandle_t frameHandled;
    Preferences preferences;

    void cameraTask();
    static void cameraTaskEntryPoint(void *param);
    void cleanFrameBuffer(); // New function to clean the frame buffer
};

#endif // CAMERA_H
