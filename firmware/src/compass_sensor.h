#pragma once

#include "logger.h"
#include "proto_gen/smartknob.pb.h"
#include <Adafruit_MMC56x3.h>

class CompassSensor
{
private:
    float currentHeading_ = 0.0f;
    Logger *logger_ = nullptr;
    Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
    TwoWire WireI2C = TwoWire(0);

public:
    float getCurrentHeading();

    void init();
    void setLogger(Logger *logger);
    void log(const char *msg);
};