#ifndef CAMERA_H
#define CAMERA_H

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace Camera {

    void initCamera();
    void cameraTask(void *param);

    extern camera_fb_t* capturedFrame;
    extern uint32_t frameTimestampStart;
    extern uint32_t frameTimestampEnd;

    extern SemaphoreHandle_t frameReady;
    extern SemaphoreHandle_t frameHandled;

    void setFrameSize(framesize_t frameSize);
    framesize_t getFrameSize();

} // namespace Camera

#endif // CAMERA_H
