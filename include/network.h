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
        DISCOVERY = 0x00,
        HEARTBEAT = 0x01,
        SETUP_CONFIG = 0x02,
        CAMERA_FRAME = 0x03,
        IMU_DATA = 0x04,
        SENSOR_DATA = 0x05,
    };

    extern bool isHostDiscovered;
    extern IPAddress hostIP;

    void startNetworkTasks();

    void startMessage(MessageType type);
    void writePayloadChunk(const uint8_t* data, size_t length);
    void endMessage();

    void handleMessage(MessageType type, const uint8_t* payload, size_t length, IPAddress remoteIP);

    uint8_t crc8(const uint8_t* data, size_t len);
    int encodeVarInt(uint32_t value, uint8_t* buffer);
    bool decodeVarInt(WiFiUDP& stream, uint32_t& outVal);

    template <typename T>
    void encodeInt(T value);

    template <typename T>
    bool decodeInt(T& outVal);

    bool decodeStruct(void* outStruct, size_t structSize);
}
