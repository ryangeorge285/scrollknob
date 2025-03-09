#include "compass_sensor.h"

void CompassSensor::init()
{
    log("Compass sensor initialized");
}

float CompassSensor::getCurrentHeading()
{
    currentHeading_ = ((int)(currentHeading_ + 1) % 360);
    return currentHeading_;
}

void CompassSensor::log(const char *msg)
{
    if (logger_ != nullptr)
    {
        logger_->log(msg);
    }
}

void CompassSensor::setLogger(Logger *logger)
{
    logger_ = logger;
}