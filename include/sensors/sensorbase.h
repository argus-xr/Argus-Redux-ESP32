#pragma once
#include <cstdint>

struct SensorData
{
    uint32_t length;
    void* data;
};

class SensorBase
{
public:
    SensorBase() = default;

    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual SensorData getSensorData() = 0;
};