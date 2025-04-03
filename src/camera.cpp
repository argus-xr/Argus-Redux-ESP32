#include <Arduino.h>
#include "camera.h"
#include "config.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h" // load up ESP32-Cam pins, but allow switching to other models easily if needed.

namespace Camera {

    camera_fb_t* captured_frame = nullptr;
    uint32_t frame_timestamp_start = 0;
    uint32_t frame_timestamp_end = 0;
    
    SemaphoreHandle_t frame_ready; // Signal when a frame is ready to be handled
    SemaphoreHandle_t frame_handled; // Signal when a frame is handled by main.cpp

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
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;

        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x\n", err);
        } else {
            Serial.println("Camera initialized");
        }
    #endif
    }

    void cameraTask(void *param) {
        while (true) {
            camera_fb_t *fb = nullptr;
            uint32_t wait_start = micros();
            while ((micros() - wait_start) < CAMERA_TIMEOUT_US) {
                fb = esp_camera_fb_get();
                if (fb) break;
                vTaskDelay(pdMS_TO_TICKS( 1 )); // Wait 1 ms
            }
            captured_frame = fb;
            frame_timestamp_start = wait_start;
            frame_timestamp_end = micros();
            if (fb == nullptr) {
                Serial.println("Camera timeout");
            }
            xSemaphoreGive(frame_ready); // Signal that a frame (or timeout) is ready
            xSemaphoreTake(frame_handled, portMAX_DELAY); // Wait until it's handled before we start on the next one
            if (captured_frame) {
                esp_camera_fb_return(captured_frame);
                captured_frame = nullptr;
            }
        }
    }

} // namespace Camera
