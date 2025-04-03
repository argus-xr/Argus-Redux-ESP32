#include <Arduino.h>
#include <Preferences.h>
#include "camera.h"
#include "config.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h" // load up ESP32-Cam pins, but allow switching to other models easily if needed.

namespace Camera {

    camera_fb_t* capturedFrame = nullptr;
    uint32_t frameTimestampStart = 0;
    uint32_t frameTimestampEnd = 0;
    static uint32_t cameraTimeoutCount = 0;
    static framesize_t currentFrameSize = FRAMESIZE_QVGA;

    SemaphoreHandle_t frameReady; // Signal when a frame is ready to be handled
    SemaphoreHandle_t frameHandled; // Signal when a frame is handled by main.cpp
    Preferences preferences;

    void initCamera() {
    #if CAMERA_ENABLED
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = Y2_GPIO_NUM;
        config.pin_d1 = Y3_GPIO_NUM;
        config.pin_d2 = Y4_GPIO_NUM;
        config.pin_d3 = Y5_GPIO_NUM;
        config.pin_d4 = Y6_GPIO_NUM;
        config.pin_d5 = Y7_GPIO_NUM;
        config.pin_d6 = Y8_GPIO_NUM;
        config.pin_d7 = Y9_GPIO_NUM;
        config.pin_xclk = XCLK_GPIO_NUM;
        config.pin_pclk = PCLK_GPIO_NUM;
        config.pin_vsync = VSYNC_GPIO_NUM;
        config.pin_href = HREF_GPIO_NUM;
        config.pin_sccb_sda = SIOD_GPIO_NUM;
        config.pin_sccb_scl = SIOC_GPIO_NUM;
        config.pin_pwdn = PWDN_GPIO_NUM;
        config.pin_reset = RESET_GPIO_NUM;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size = currentFrameSize;
        config.jpeg_quality = 12;
        config.fb_count = 1;

        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x\n", err);
            setFrameSize(FRAMESIZE_QVGA);
        } else {
            Serial.println("Camera initialized");
        }
    #endif
    }

    void cameraTask(void *param) {
        while (true) {
            camera_fb_t *fb = nullptr;
            uint32_t waitStart = micros();
            while ((micros() - waitStart) < CAMERA_TIMEOUT_US) {
                fb = esp_camera_fb_get();
                if (fb) break;
                vTaskDelay(pdMS_TO_TICKS( 1 )); // Wait 1 ms
            }
            capturedFrame = fb;
            frameTimestampStart = waitStart;
            frameTimestampEnd = micros();
            if (fb == nullptr) {
                Serial.println("Camera timeout");
                cameraTimeoutCount++;
                if (cameraTimeoutCount > MAX_CAMERA_TIMEOUTS) {
                    Serial.println("Too many camera timeouts, resetting camera.");
                    esp_camera_deinit();
                    initCamera();
                    cameraTimeoutCount = 0;
                }
            } else {
                cameraTimeoutCount = 0;
            }
            xSemaphoreGive(frameReady); // Signal that a frame (or timeout) is ready
            xSemaphoreTake(frameHandled, portMAX_DELAY); // Wait until it's handled before we start on the next one
            if (capturedFrame) {
                esp_camera_fb_return(capturedFrame);
                capturedFrame = nullptr;
            }
        }
    }

    void setFrameSize(framesize_t frameSize) {
        preferences.begin("camera", false);
        preferences.putUChar("frameSize", frameSize);
        preferences.end();
        currentFrameSize = frameSize;
        esp_camera_deinit();
        initCamera();
    }

    framesize_t getFrameSize() {
        preferences.begin("camera", true);
        currentFrameSize = (framesize_t)preferences.getUChar("frameSize", FRAMESIZE_QVGA);
        preferences.end();
        return currentFrameSize;
    }

} // namespace Camera
