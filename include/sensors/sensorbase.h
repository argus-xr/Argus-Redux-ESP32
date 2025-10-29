#pragma once
#include <cstdint>
#include "network.h"

class SensorBase
{
public:
    SensorBase() = default;

    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    // Writes sensor data directly to the buffer. Returns bytes written, or 0 if error/no data/buffer too small.
    // Format: [SensorID (1 byte)] [PayloadLength (2 bytes)] [Payload Data (PayloadLength bytes)]
    virtual void writeSensorDataToPacket() = 0;
 };