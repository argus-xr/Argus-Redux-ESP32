# Argus VR Tracker ESP32 Firmware AI Assistant Guide

This guide helps AI coding assistants understand the key patterns and workflows of the Argus VR Tracker ESP32 firmware codebase.

## Project Architecture

### Core Components
- **SensorManager (`src/sensors/sensormanager.*`)**: Singleton that manages and coordinates all sensor operations
- **IMU (`src/sensors/IMU.*`)**: Handles inertial measurement data collection
- **Camera (`src/sensors/camera.*`)**: Manages camera frame capture
- **Network (`src/network.*`)**: Handles WiFi connectivity and UDP communication

### Key Data Flows
1. Sensors (IMU + Camera) collect data independently in background tasks
2. Main loop processes sensor data via `SensorManager::processSensorData()`
3. Data is packaged with timestamps and checksums
4. Packets are sent to host PC via UDP

## Development Workflows

### Build & Upload
```bash
# Build for ESP32-CAM
pio run -e esp32cam

# Upload firmware
pio run -e esp32cam -t upload

# Monitor serial output
pio device monitor -b 115200
```

### Hardware Requirements
- ESP32-CAM or ESP32-S3-CAM module
- Separate power supply needed during development
- FTDI adapter for programming (RX/TX/GND only, no power)

## Key Patterns

### Singleton Pattern
```cpp
// Access singleton instances like this:
SensorManager::instance.init();
SensorManager::getIMU().someMethod();
```

### Task Management
- Background tasks run on FreeRTOS
- Use `xSemaphoreGive()` for signaling between tasks
- Camera and IMU tasks run concurrently

### Error Handling
- Brown-out detector disabled in setup
- Network connection failures trigger auto-reboot
- Use `Serial.printf()` for debug messages

## Configuration

### Key Files
- `platformio.ini`: Build settings and dependencies
- `include/config.h`: Hardware pins and system constants
- `include/camera_pins.h`: Camera-specific pin definitions

### Dependencies
- jrowberg/i2cdevlib-MPU6050 (v1.0.0+)
- tzapu/WiFiManager (v2.0.11-beta+)

## Testing

When making changes:
1. Test WiFi connection and captive portal
2. Verify IMU data stream
3. Check camera frame capture and transmission
4. Monitor serial output for errors
5. Test power stability under load

## Gotchas

1. **Power Management**: Always ensure sufficient power supply during development to avoid brownouts
2. **Memory Management**: Camera frames require significant heap memory
3. **Task Priority**: Network tasks can interfere with sensor timing
4. **Buffer Sizes**: UDP packets have size limits for camera frame data