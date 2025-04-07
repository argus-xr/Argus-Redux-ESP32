# Argus VR Tracker: ESP32-CAM/S3-CAM Firmware

## Overview

Argus is a low-cost, open-source VR tracking system that provides accurate and responsive positional tracking. This repository contains the firmware for the ESP32-CAM and ESP32-S3-CAM-based trackers, which act as sensor nodes in the Argus system.

**Key Features:**

*   **Affordable:** Built using readily available and inexpensive ESP32-CAM/S3-CAM modules.
*   **Hybrid Tracking:** Combines IMU and camera data for drift-free, responsive tracking.
*   **Wireless:** Connects to a host computer via Wi-Fi for real-time data.
*   **Low Latency:** Designed for minimal latency, ideal for VR applications.
*   **Open Source:** Fully open-source, encouraging community contributions and customization.

## System Architecture

The Argus system consists of two main components:

1.  **ESP32-CAM/S3-CAM Tracker (This Firmware):**
    *   Collects IMU and camera data.
    *   Transmits sensor data wirelessly to the host.
    *   Manages sensor initialization and operation.
    *   Handles basic network discovery and communication.

2.  **Host Computer (Separate Project):**
    *   Receives sensor data from the ESP32-CAM/S3-CAM tracker.
    *   Processes the data to calculate the tracker's position and orientation.
    *   Interfaces with VR applications.

## Program Flow

The ESP32-CAM/S3-CAM firmware operates in the following way:

### Initialization (`setup()`)

1.  Disables the brownout detector.
2.  Initializes serial communication for debugging.
3.  Connects to Wi-Fi using `WiFiManager` (creates a captive portal if needed).
4.  Initializes the IMU and camera modules.
5.  Starts the camera and IMU tasks.
6.  Creates semaphores for camera frame handling.

### Main Loop (`loop()`)

The main loop continuously checks for data to send to the host, prioritizing camera frames.

1.  **Camera Frame Ready:** If a new camera frame is ready (signaled by a semaphore), it's sent to the host along with the latest IMU data.
2.  **IMU Data Ready:** If no camera frame is ready, the loop checks if enough IMU samples have been collected. If so, it sends the IMU data to the host.

### Background Tasks

The following tasks run concurrently:

*   **Camera Task (`cameraTask()`):** Captures camera frames and signals when a frame is ready.
*   **IMU Task (`imuTask()`):** Continuously reads and buffers IMU data.
*   **Network Tasks:**
    *   **`connectWiFiTask()`:** Manages Wi-Fi connection.
    *   **`hostDiscoveryTask()`:** Sends out discovery messages to find the host.
    *   **`udpListenerTask()`:** Listens for UDP packets from the host.
    *   **`wifiMonitorTask()`:** Monitors the WiFi connection and reboots if it is lost.

### Data Transmission (`sendPacket()`)

*   Packages IMU and camera data into a UDP packet.
*   Includes a header with timestamps, battery voltage, and IMU sample count.
*   Uses a CRC8 checksum for data integrity.
*   Sends the packet to the host computer.

### Sensor Management (`SensorManager`)

*   Manages the starting and stopping of the IMU and Camera.
*   Provides a simple interface to control the sensors.

## Dependencies

This project is built using PlatformIO, which simplifies dependency management and the build process. The following libraries are automatically managed by PlatformIO and are defined in the `platformio.ini` file:

*   `jrowberg/i2cdevlib-MPU6050@^1.0.0`: For interfacing with the MPU6050 IMU.
*   `tzapu/WiFiManager@^2.0.11-beta`: For managing Wi-Fi connections and creating a captive portal.

## Hardware Requirements

*   **ESP32-CAM or ESP32-S3-CAM Module:** The core of the tracker.
*   **MPU6050 IMU:** For inertial measurements.
*   **Battery and Charging Circuit:** Required for operation.
*   **USB to Serial Adapter:** For programming and debugging. An FT232RL FTDI breakout board is recommended.
    *   **Important:** The FTDI board alone cannot provide enough power to avoid brownouts. You will need to connect a separate USB cable to the ESP32-CAM/S3-CAM module's 5V and GND pins to provide sufficient power during development. The charging circuit could be used, as you'll need one anyway. I've been using a USB breakout board. This means you only connect the FTDI's RX/TX and gnd pins, not the +5V or +3.3V pins!

## Quick Start

Follow these steps to get started:

1.  **Install PlatformIO:** Install the PlatformIO extension for VSCode (or your preferred IDE).
2.  **Clone the Repository:** Clone this repository to your local machine.
3.  **Open the Project:** Open the cloned folder in VSCode. PlatformIO should automatically detect the project.
4.  **Build the Firmware:** In the PlatformIO sidebar, under "Project Tasks," select your target environment (e.g., `esp32cam`) and click "Build."
5.  **Upload the Firmware:** Once the build is successful, click "Upload" in the same "Project Tasks" section.
6.  **Connect the Hardware:** Connect the ESP32-CAM/S3-CAM to the IMU, and connect the battery.
7.  **Monitor Serial Output (Optional):** If you want to see debug messages, click "Monitor" in the "Project Tasks" section.

## To-Do

*   **Calibration:** Implement IMU and camera calibration routines.
*   **Advanced Data Processing:** Explore more sophisticated data fusion techniques.
*   **Power Management:** Optimize power consumption for longer battery life.
*   **Error Handling:** Improve error handling and recovery.
*   **Configuration:** Allow for more flexible configuration options.

## Contributing

Contributions to the Argus project are welcome! Please feel free to submit pull requests or open issues on GitHub.

## License

License undecided for now.
