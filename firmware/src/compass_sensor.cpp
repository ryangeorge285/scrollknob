#include "compass_sensor.h"
#include <Adafruit_MMC56x3.h>

void CompassSensor::init()
{
    WireI2C.begin(MMC5603_SDA, MMC5603_SCL, MMC5603_I2CSPEED);

    if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &WireI2C))
    { // I2C mode
        /* There was a problem detecting the MMC5603 ... check your connections */
        log("There was an error in the MMC5603 compass sensor");
        while (1)
            delay(10);
    }
    else
        log("MMC5603 compass initialised");

    mmc.printSensorDetails();
    mmc.setDataRate(1000);
    mmc.setContinuousMode(true);
}

float CompassSensor::getCurrentHeading()
{
    sensors_event_t event;
    mmc.getEvent(&event);
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "MMC5603 compass reading %f", atan2(event.magnetic.y, event.magnetic.x));
    log(buffer);
    return atan2(event.magnetic.y, event.magnetic.x);
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