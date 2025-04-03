#pragma once

// --- Network Configuration ---
#define DEST_PORT 4210

// --- Camera Configuration ---
#define CAMERA_ENABLED 1
#define CAMERA_TIMEOUT_US 1000000 // 1 second
#define MAX_CAMERA_TIMEOUTS 10
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA // Or FRAMESIZE_UXGA, etc.
#define CAMERA_MODEL_AI_THINKER

// --- IMU Configuration ---
#define MAX_IMU_SAMPLES 16

// --- Battery Configuration ---
#define BATTERY_PIN 34
#define BATTERY_CALIBRATION_FACTOR 1.0f // Adjust this value as needed
