#ifndef ADS1220_ADC_H
#define ADS1220_ADC_H
#include <ADS1220_WE.h>
#include <SPI.h>
#include "logger.h"
#include <Arduino.h>

class ADS1220Controller
{
private:
    ADS1220_WE ads0;
    ADS1220_WE ads1;
    Logger *logger_ = nullptr;

public:
    ADS1220Controller()
        : ads0(PIN_ADS1220_CS0, PIN_ADS1220_DRDY0, true),
          ads1(PIN_ADS1220_CS1, PIN_ADS1220_DRDY1, true) {}

    void init();
    void calibrate();
    float readStrainGauge(bool print = true);

    void setLogger(Logger *logger);
    void log(const char *msg);
};
#endif