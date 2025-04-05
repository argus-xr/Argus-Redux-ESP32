#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "network.h"
#include "config.h"
#include "main.h"

namespace Network {

    // Constants - Definition
    const int UDP_PORT = 4210;
    const char* REPLY_MSG = "ARGUS_REPLY";
    const unsigned long HOST_TIMEOUT_MS = 5000;
    const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
    const unsigned long DISCOVERY_INTERVAL_MS = 1000;
    const int MAX_UDP_PACKET_SIZE = 512;
    const uint8_t MAX_CRC_ERRORS = 10;

    WiFiUDP udp;
    IPAddress hostIP;
    bool isHostDiscovered = false;

    static unsigned long lastHostPacketTime = 0;
    static unsigned long lastHeartbeatTime = 0;
    static unsigned long lastDiscoveryTime = 0;

    static bool messageInProgress = false;
    static uint8_t messageCrc = 0;
    static SemaphoreHandle_t udpMutex = nullptr;
    static IPAddress broadcastIP;
    static uint32_t crcErrorCount = 0;

    // --- Decoding Buffer ---
    static const uint8_t* decodeBuffer = nullptr;
    static size_t decodeBufferSize = 0;
    static size_t decodeIndex = 0;

    void startMessage(MessageType type) {
        if (messageInProgress) return;
        if (udpMutex) xSemaphoreTake(udpMutex, portMAX_DELAY);

        if (isHostDiscovered) {
            udp.beginPacket(hostIP, UDP_PORT);
        } else {
            udp.beginPacket(broadcastIP, UDP_PORT);
        }

        uint8_t typeBuf[5];
        int typeLen = encodeVarInt(static_cast<uint32_t>(type), typeBuf);
        udp.write(typeBuf, typeLen);
        messageCrc = crc8(typeBuf, typeLen);

        messageInProgress = true;
    }

    void writePayloadChunk(const uint8_t* data, size_t length) {
        if (!messageInProgress) return;
        udp.write(data, length);
        messageCrc ^= crc8(data, length);
    }

    void endMessage() {
        if (!messageInProgress) return;
        udp.write(&messageCrc, 1);
        udp.endPacket();
        messageInProgress = false;
        messageCrc = 0;
        if (udpMutex) xSemaphoreGive(udpMutex);
    }

    void handleMessage(MessageType type, const uint8_t* payload, size_t length, IPAddress remoteIP) {
        decodeBuffer = payload;
        decodeBufferSize = length;
        decodeIndex = 0;

        switch (type) {
            case MessageType::DISCOVERY:
                Serial.print("📡 Discovery message received from: ");
                Serial.println(remoteIP);
                char discoveryString[128];
                if (!isHostDiscovered && decodeString(discoveryString, sizeof(discoveryString)) && strcmp(discoveryString, REPLY_MSG) == 0) {
                    hostIP = remoteIP;
                    isHostDiscovered = true;
                    lastHostPacketTime = millis();
                    Serial.print("🌟 Host discovered: ");
                    Serial.println(hostIP);
                }
                break;
            case MessageType::HEARTBEAT:
                Serial.println("💓 Heartbeat received");
                break;
            case MessageType::SETUP_CONFIG:
                Serial.println("🛠️ Setup/config received");
                // Handle setup/config data here
                char configString[128];
                if (decodeString(configString, sizeof(configString))) {
                    Serial.print("Received config string: ");
                    Serial.println(configString);
                } else {
                    Serial.println("Failed to decode config string.");
                }
                break;
            case MessageType::SENSOR_DATA:
                Serial.println("📦 Sensor data received");
                // Handle sensor data here
                break;
            default:
                Serial.printf("📦 Unknown message type: %u, %u bytes\n", static_cast<uint32_t>(type), length);
                break;
        }
    }

    void wifiMonitorTask(void* pvParams) {
        while (true) {
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("🚨 WiFi lost. Rebooting.");
                ESP.restart();
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    void hostDiscoveryTask(void* pvParams) {
        while (true) {
            if (!isHostDiscovered && millis() - lastDiscoveryTime > DISCOVERY_INTERVAL_MS) {
                Serial.println("🔍 Sending discovery message");
                startMessage(MessageType::DISCOVERY);
                encodeString(REPLY_MSG);
                endMessage();
                lastDiscoveryTime = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void udpListenerTask(void* pvParams) {
        while (true) {
            int len = udp.parsePacket();
            if (len > 0) {
                Serial.println("📦 Packet received");
                uint8_t packet[MAX_UDP_PACKET_SIZE];
                int n = udp.read(packet, sizeof(packet));
                if (n < 2) continue;

                uint8_t crc = packet[n - 1];
                if (crc8(packet, n - 1) != crc) {
                    Serial.println("❌ CRC fail");
                    crcErrorCount++;
                    if (crcErrorCount > MAX_CRC_ERRORS) {
                        Serial.println("Too many CRC errors, resetting connection.");
                        isHostDiscovered = false;
                        crcErrorCount = 0;
                    }
                    continue;
                } else {
                    crcErrorCount = 0;
                }

                WiFiUDP inStream = udp;
                uint32_t typeRaw = 0;
                if (!decodeVarInt(inStream, typeRaw)) continue;
                MessageType type = static_cast<MessageType>(typeRaw);

                size_t payloadLen = inStream.available();
                uint8_t* payload = (payloadLen > 0) ? (uint8_t*)malloc(payloadLen) : nullptr;

                if (payload) inStream.read(payload, payloadLen);

                lastHostPacketTime = millis();
                handleMessage(type, payload, payloadLen, udp.remoteIP());
                if (payload) free(payload);
            }

            if (isHostDiscovered && millis() - lastHostPacketTime > HOST_TIMEOUT_MS) {
                Serial.println("⚠️ Host timeout — rediscovering");
                isHostDiscovered = false;
            }

            if (isHostDiscovered && millis() - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
                startMessage(MessageType::HEARTBEAT);
                endMessage();
                lastHeartbeatTime = millis();
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void connectWiFiTask(void* pvParams) {
        WiFi.mode(WIFI_STA);
        vTaskDelay(pdMS_TO_TICKS(1000));
        WiFiManager wm;
        wm.setConfigPortalTimeout(180);
        if (!wm.autoConnect("ArgusSetup")) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            ESP.restart();
        }
        Serial.println("✅ WiFi connected");

        udp.begin(UDP_PORT);
        Serial.println("Started UDP");

        broadcastIP = ~WiFi.subnetMask() | WiFi.gatewayIP();
        Serial.println("Broadcast IP set");

        if (!udpMutex) udpMutex = xSemaphoreCreateMutex();
        Serial.println("UDP mutex created");

        xTaskCreatePinnedToCore(hostDiscoveryTask, "HostDiscovery", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(udpListenerTask, "UDPListener", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(wifiMonitorTask, "WiFiMonitor", 2048, nullptr, 1, nullptr, 1);
        Serial.println("Tasks created");
        
        vTaskDelete(NULL); // Delete the task after setup
        return;
    }

    void startNetworkTasks() {
        xTaskCreatePinnedToCore(connectWiFiTask, "WiFiManager", 4096, nullptr, 1, nullptr, 1);
    }

    // --- Encoding helpers ---

    uint8_t crc8(const uint8_t* data, size_t len) {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; ++j)
                crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
        return crc;
    }

    int encodeVarInt(uint32_t value, uint8_t* buffer) {
        int i = 0;
        do {
            uint8_t byte = value & 0x7F;
            value >>= 7;
            if (value) byte |= 0x80;
            buffer[i++] = byte;
        } while (value && i < 5);
        return i;
    }

    bool decodeVarInt(WiFiUDP& stream, uint32_t& outVal) {
        outVal = 0;
        uint8_t byte;
        int shift = 0;
        for (int i = 0; i < 5; ++i) {
            if (stream.read(&byte, 1) != 1) return false;
            outVal |= (uint32_t)(byte & 0x7F) << shift;
            if (!(byte & 0x80)) return true;
            shift += 7;
        }
        return false;
    }

    template <typename T>
    void encodeInt(T value) {
        uint8_t bytes = sizeof(T);
        for (int i = 0; i < bytes; i++) {
            uint8_t byte = (value >> (i * 8)) & 0xFF;
            udp.write(&byte, 1);
            messageCrc ^= byte;
        }
    }

    template <typename T>
    bool decodeInt(T& outVal) {
        if (decodeIndex + sizeof(T) > decodeBufferSize) return false;
        outVal = 0;
        for (int i = 0; i < sizeof(T); i++) {
            outVal |= ((T)decodeBuffer[decodeIndex++]) << (i * 8);
        }
        return true;
    }

    bool decodeStruct(void* outStruct, size_t structSize) {
        if (decodeIndex + structSize > decodeBufferSize) return false;
        memcpy(outStruct, decodeBuffer + decodeIndex, structSize);
        decodeIndex += structSize;
        return true;
    }

    void encodeString(const char* str) {
        if (!messageInProgress) return;
        if (str == nullptr) return;

        const char* ptr = str;
        while (*ptr != '\0') {
            udp.write((uint8_t*)ptr, 1);
            messageCrc ^= *ptr;
            ptr++;
        }
        udp.write((uint8_t)0); // Null terminator
        messageCrc ^= 0;
    }

    bool decodeString(char* outStr, size_t maxLen) {
        if (outStr == nullptr || maxLen == 0) return false;
        size_t i = 0;
        while (decodeIndex < decodeBufferSize && i < maxLen - 1) {
            if (decodeBuffer[decodeIndex] == 0) {
                decodeIndex++;
                outStr[i] = 0;
                return true;
            }
            outStr[i++] = decodeBuffer[decodeIndex++];
        }
        outStr[i] = 0; // Ensure null termination even if maxLen is reached
        return false;
    }
}
