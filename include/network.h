#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>

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
		HEARTBEAT		= 0x02,
        SETUP_CONFIG    = 0x03,
        SENSOR_DATA     = 0x04,
    };

    extern bool isHostDiscovered;
    extern IPAddress hostIP;

    void startNetworkTasks();

    void startMessage(MessageType type);
    void writePayloadChunk(const uint8_t* data, size_t length);
    void endMessage();

    void handleMessage(MessageType type, const uint8_t* payload, size_t length, IPAddress remoteIP);


    // --- Encoding helpers ---
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    uint8_t calculateChecksum(const uint8_t* data, size_t len);
    void updateChecksum(uint8_t data);
    void encodeVarInt(uint32_t value);
    bool decodeVarInt(WiFiUDP& stream, uint32_t& outVal);

    template <typename T>
    void encodeInt(T value);

    template <typename T>
    bool decodeInt(T& outVal);
    
    template <typename T>
    void encodeStruct(T value);

    template <typename T>
    bool decodeStruct(T& outStruct);
    
    void encodeString(const char* str);

    bool decodeString(char* outStr, size_t maxLen);
}
