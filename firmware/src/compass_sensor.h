#pragma once

#include "logger.h"
#include "proto_gen/smartknob.pb.h"

class CompassSensor
{
private:
    float currentHeading_ = 0.0f;
    Logger *logger_ = nullptr;

public:
    float getCurrentHeading();

    void init();
    void setLogger(Logger *logger);
    void log(const char *msg);
};