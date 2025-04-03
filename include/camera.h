#ifndef CAMERA_H
#define CAMERA_H

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace Camera {

    void initCamera();
    void cameraTask(void *param);

    extern camera_fb_t* captured_frame;
    extern uint32_t frame_timestamp_start;
    extern uint32_t frame_timestamp_end;

    extern SemaphoreHandle_t frame_ready;
    extern SemaphoreHandle_t frame_handled;

} // namespace Camera

#endif // CAMERA_H
