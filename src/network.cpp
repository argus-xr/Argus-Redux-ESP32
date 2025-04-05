#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic> // Include for atomic types
#include "network.h"
#include "config.h"
#include "main.h"

namespace Network {

    // Constants - Definition
    const int UDP_PORT = 4210;
    const char* DISCOVERY_MSG = "ARGUS_DISCOVERY";
    const char* REPLY_MSG = "ARGUS_REPLY";
    const unsigned long HOST_TIMEOUT_MS = 60000;
    const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
    const unsigned long DISCOVERY_INTERVAL_MS = 1000;
    const int MAX_UDP_PACKET_SIZE = 512;
    const uint8_t MAX_CHECKSUM_ERRORS = 10;

    WiFiUDP udp;
    IPAddress hostIP;
    std::atomic<bool> isHostDiscovered{false}; // Initialize as atomic and false

    static unsigned long lastHostPacketTime = 0;
    static unsigned long lastHeartbeatTime = 0;
    //static unsigned long lastDiscoveryTime = 0; // Removed

    static bool messageInProgress = false;
    static uint8_t messageCrc = 0;
    static SemaphoreHandle_t udpMutex = nullptr;
    static IPAddress broadcastIP;
    static uint32_t checksumErrorCount = 0;

    // --- Decoding Buffer ---
    static const uint8_t* decodeBuffer = nullptr;
    static size_t decodeBufferSize = 0;
    static size_t decodeIndex = 0;

    // CRC8 Lookup Table: 0xD5
    static const uint8_t CRC8_TABLE[256] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,   0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,   0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,   0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,   0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,   0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,   0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,   0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,   0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,   0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,   0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,   0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,   0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,   0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,   0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,   0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,   0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
    };

    void startMessage(MessageType type) {
        if (messageInProgress) return;
        if (udpMutex) xSemaphoreTake(udpMutex, portMAX_DELAY);

        messageInProgress = true;

        if (isHostDiscovered) {
            udp.beginPacket(hostIP, UDP_PORT);
        } else {
            udp.beginPacket(broadcastIP, UDP_PORT);
        }

        messageCrc = 0; // Reset checksum
        encodeInt<uint8_t>(static_cast<uint8_t>(type)); // Encode the message type as a VarInt and update the checksum
    }

    void writePayloadChunk(const uint8_t* data, size_t length) {
        if (!messageInProgress) return;
        for (size_t i = 0; i < length; i++) {
            udp.write(&data[i], 1);
            updateCrc(data[i]);
        }
    }

    void endMessage() {
        if (!messageInProgress) return;
        udp.write(&messageCrc, 1);
        udp.endPacket();

        messageInProgress = false;
        if (udpMutex) xSemaphoreGive(udpMutex);
    }

    void handleMessage(MessageType type, const uint8_t* payload, size_t length, IPAddress remoteIP) {
        decodeBuffer = payload;
        decodeBufferSize = length;
        decodeIndex = 0;

        switch (type) {
            case MessageType::HELLO:
                Serial.print("📡 Hello message received from: ");
                Serial.println(remoteIP);
                if (!isHostDiscovered) {
                    Serial.print("📡 Hello message accepted!");
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
            if (!isHostDiscovered) {
                Serial.println("🔍 Sending discovery message");
                startMessage(MessageType::DISCOVERY);
                encodeString(DISCOVERY_MSG);
                endMessage();
            }
            vTaskDelay(pdMS_TO_TICKS(DISCOVERY_INTERVAL_MS)); // Wait for the interval
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

                uint8_t receivedChecksum = packet[n - 1];
                uint8_t calculatedChecksum = calculateChecksum(packet, n - 1);
                if (calculatedChecksum != receivedChecksum) {
                    Serial.println("❌ Checksum fail");
                    checksumErrorCount++;
                    if (checksumErrorCount > MAX_CHECKSUM_ERRORS) {
                        Serial.println("Too many checksum errors, resetting connection.");
                        isHostDiscovered = false;
                        checksumErrorCount = 0;
                    }
                    continue;
                } else {
                    checksumErrorCount = 0;
                }

                uint32_t typeRaw = 0;
                decodeBuffer = packet;
                decodeBufferSize = n - 1; // Exclude checksum byte
                decodeIndex = 0;
                if (!decodeVarInt(typeRaw)) continue;
                
                MessageType type = static_cast<MessageType>(typeRaw);

                size_t payloadLen = decodeBufferSize - decodeIndex;
                uint8_t* payload = (payloadLen > 0) ? (uint8_t*)malloc(payloadLen) : nullptr;

                if (payload) {
                    memcpy(payload, decodeBuffer + decodeIndex, payloadLen);
                }

                lastHostPacketTime = millis();
                handleMessage(type, payload, payloadLen, udp.remoteIP());
                if (payload) free(payload);
            }

            if (isHostDiscovered && millis() - lastHostPacketTime > HOST_TIMEOUT_MS) {
                Serial.println("⚠️ Host timeout — rediscovering");
                isHostDiscovered = false; // Atomic write
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
    uint8_t crc8_update(uint8_t crc, uint8_t data) {
        return CRC8_TABLE[crc ^ data];
    }

    uint8_t calculateChecksum(const uint8_t* data, size_t len) {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; ++i) {
            crc = crc8_update(crc, data[i]);
        }
        return crc;
    }
    
    void updateCrc(uint8_t data) {
        messageCrc = crc8_update(messageCrc, data);
    }

    void encodeVarInt(uint32_t value) {
        do {
            uint8_t byte = value & 0x7F;
            value >>= 7;
            if (value) byte |= 0x80;
            writePayloadChunk(&byte, 1);
        } while (value);
    }

    bool decodeVarInt(uint32_t& outVal) {
        outVal = 0;
        uint8_t byte;
        int shift = 0;
        for (int i = 0; i < 5; ++i) {
            if (decodeIndex >= decodeBufferSize) return false;
            byte = decodeBuffer[decodeIndex++];
            outVal |= (uint32_t)(byte & 0x7F) << shift;
            if (!(byte & 0x80)) return true;
            shift += 7;
        }
        return false;
    }

    template <typename T>
    void encodeInt(T value) {
        writePayloadChunk(reinterpret_cast<const uint8_t*>(&value), sizeof(T));
    }

    template <typename T>
    bool decodeInt(T& outVal) {
        if (decodeIndex + sizeof(T) > decodeBufferSize) return false;
        memcpy(&outVal, decodeBuffer + decodeIndex, sizeof(T));
        decodeIndex += sizeof(T);
        return true;
    }

    
    template <typename T>
    void encodeStruct(T value) {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(&value);
        writePayloadChunk(data, sizeof(T));
    }

    template <typename T>
    bool decodeStruct(T& outStruct) {
        if (decodeIndex + sizeof(T) > decodeBufferSize) return false;
        memcpy(&outStruct, decodeBuffer + decodeIndex, sizeof(T));
        decodeIndex += sizeof(T);
        return true;
    }

    void encodeString(const char* str) {
        if (!messageInProgress) return;
        if (str == nullptr) return;

        const char* ptr = str;
        while (*ptr != '\0') {
            writePayloadChunk((uint8_t*)ptr, 1);
            ptr++;
        }
        writePayloadChunk((uint8_t*)"\0", 1); // Null terminator
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
