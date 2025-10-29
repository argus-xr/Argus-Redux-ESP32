#include <Arduino.h>
#include <Preferences.h>
#include "sensors/camera.h"
#include "config.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h" // load up ESP32-Cam pins, but allow switching to other models easily if needed.

CameraClass::CameraClass() : capturedFrame(nullptr), frameTimestampStart(0), frameTimestampEnd(0),
    cameraTimeoutCount(0), currentFrameSize(FRAMESIZE_QVGA), isRunning(false), initialized(false) {
    frameReady = xSemaphoreCreateBinary();
    frameHandled = xSemaphoreCreateBinary();
    mutex = xSemaphoreCreateMutex();
    cameraTaskHandle = nullptr;
}

CameraClass::~CameraClass() {
    stop();
    if (frameReady) {
        vSemaphoreDelete(frameReady);
    }
    if (frameHandled) {
        vSemaphoreDelete(frameHandled);
    }
    if (mutex) {
        vSemaphoreDelete(mutex);
    }
}

void CameraClass::init() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (initialized) {
        Serial.println("Camera already initialized, skipping init.");
        xSemaphoreGive(mutex);
        return;
    }
    Serial.println("Initializing camera...");
    initialized = true;
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
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        setFrameSize(FRAMESIZE_QVGA);
    } else {
        Serial.println("Camera initialized!");
    }
    xSemaphoreGive(mutex);
}

void CameraClass::start() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (!cameraTaskHandle) {
        xTaskCreatePinnedToCore(CameraClass::cameraTaskEntryPoint, "Camera Task", 8192, this, 5, &cameraTaskHandle, 1);
    }
    isRunning = true;
    xSemaphoreGive(mutex);
}

void CameraClass::stop() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    isRunning = false;
    cleanFrameBuffer(); // Clean the frame buffer when stopping
    if (cameraTaskHandle) {
        vTaskDelete(cameraTaskHandle);
        cameraTaskHandle = nullptr;
    }
    xSemaphoreGive(mutex);
}

void CameraClass::cameraTaskEntryPoint(void *param) {
    CameraClass *self = static_cast<CameraClass *>(param);
    self->cameraTask();
}

void CameraClass::writeSensorDataToPacket() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (capturedFrame) {
        Network::writeInt<uint8_t>(1); // Sensor ID for camera
        Network::writeInt<uint16_t>(1); // Amount of frames we're sending
        Network::writeInt<uint16_t>(capturedFrame->len);
        Network::writePayloadChunk(capturedFrame->buf, capturedFrame->len);
    }
    xSemaphoreGive(mutex);
}

void CameraClass::cameraTask() {
    while (true) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        bool running = isRunning;
        xSemaphoreGive(mutex);

        if (running) {
            camera_fb_t *fb = nullptr;
            uint32_t waitStart = micros();
            while ((micros() - waitStart) < CAMERA_TIMEOUT_US) {
                fb = esp_camera_fb_get();
                if (fb) {
                    Serial.println("Got frame");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10)); // Wait 10 ms
            }

            xSemaphoreTake(mutex, portMAX_DELAY);
            capturedFrame = fb;
            frameTimestampStart = waitStart;
            frameTimestampEnd = micros();
            
            if (fb == nullptr) {
                Serial.println("Camera timeout");
                cameraTimeoutCount++;
                if (cameraTimeoutCount > MAX_CAMERA_TIMEOUTS) {
                    Serial.println("Too many camera timeouts, resetting camera.");
                    esp_camera_deinit();
                    initialized = false;
                    init();
                    cameraTimeoutCount = 0;
                }
            } else {
                cameraTimeoutCount = 0;
                xSemaphoreGive(frameReady); // Signal that a frame (or timeout) is ready
                xSemaphoreGive(mutex);
                
                Serial.println("Waiting for frame to be handled");
                xSemaphoreTake(frameHandled, portMAX_DELAY); // Wait until it's handled before we start on the next one
                Serial.println("Frame handled");
                
                xSemaphoreTake(mutex, portMAX_DELAY);
            }
            
            if (capturedFrame) {
                Serial.println("Clearing framebuffer");
                esp_camera_fb_return(capturedFrame);
                capturedFrame = nullptr;
            }
            xSemaphoreGive(mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms if we should start again
        }
    }
}

void CameraClass::setFrameSize(framesize_t frameSize) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    preferences.begin("camera", false);
    preferences.putUChar("frameSize", frameSize);
    preferences.end();
    currentFrameSize = frameSize;
    esp_camera_deinit();
    init();
    xSemaphoreGive(mutex);
}

framesize_t CameraClass::getFrameSize() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    preferences.begin("camera", true);
    currentFrameSize = (framesize_t)preferences.getUChar("frameSize", FRAMESIZE_QVGA);
    preferences.end();
    framesize_t result = currentFrameSize;
    xSemaphoreGive(mutex);
    return result;
}

void CameraClass::cleanFrameBuffer() {
    // Mutex should already be held when calling this private method
    if (capturedFrame) {
        esp_camera_fb_return(capturedFrame);
        capturedFrame = nullptr;
    }
}

camera_fb_t* CameraClass::getCapturedFrame() const {
    xSemaphoreTake(mutex, portMAX_DELAY);
    camera_fb_t* result = capturedFrame;
    xSemaphoreGive(mutex);
    return result;
}

uint32_t CameraClass::getFrameTimestampStart() const {
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint32_t result = frameTimestampStart;
    xSemaphoreGive(mutex);
    return result;
}

uint32_t CameraClass::getFrameTimestampEnd() const {
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint32_t result = frameTimestampEnd;
    xSemaphoreGive(mutex);
    return result;
}

SemaphoreHandle_t CameraClass::getFrameReadySemaphore() const {
    return frameReady;
}

SemaphoreHandle_t CameraClass::getFrameHandledSemaphore() const {
    return frameHandled;
}