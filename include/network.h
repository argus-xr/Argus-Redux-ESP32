#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include <atomic> // Include for atomic types

namespace Network {

    // Constants
    extern const int UDP_PORT;
    extern const char* REPLY_MSG;
    extern const unsigned long HOST_TIMEOUT_MS;
    extern const unsigned long HEARTBEAT_INTERVAL_MS;
    extern const unsigned long DISCOVERY_INTERVAL_MS;
    extern const int MAX_UDP_PACKET_SIZE;
    extern const uint8_t MAX_CRC_ERRORS;

    enum MessageType : uint8_t {
        UNKNOWN			= 0x00,
        DISCOVERY		= 0x01,
        HELLO			= 0x02,
        HEARTBEAT		= 0x03,
        SETUP_CONFIG    = 0x04,
        SENSOR_DATA     = 0x05,
    };

    extern std::atomic<bool> isHostDiscovered; // Make it atomic
    extern IPAddress hostIP;

    void startNetworkTasks();

    void startMessage(MessageType type);
    void writePayloadChunk(const uint8_t* data, size_t length);
    void endMessage();

    void handleMessage(MessageType type, const uint8_t* payload, size_t length, IPAddress remoteIP);


    // --- Encoding helpers ---
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    uint8_t calculateChecksum(const uint8_t* data, size_t len);
    void updateCrc(uint8_t data);
    void encodeVarInt(uint32_t value);
    bool decodeVarInt(uint32_t& outVal);

    template <typename T>
    void encodeInt(T value);

    template <typename T>
    bool decodeInt(T& outVal);

    // Moved decodeBuffer, decodeBufferSize, and decodeIndex declarations here
    extern const uint8_t* decodeBuffer;
    extern size_t decodeBufferSize;
    extern size_t decodeIndex;

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

    void encodeString(const char* str);

    bool decodeString(char* outStr, size_t maxLen);
}
