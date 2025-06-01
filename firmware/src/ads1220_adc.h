#ifndef ADS1220_ADC_H
#define ADS1220_ADC_H
#include <ADS1220_WE.h>
#include <SPI.h>
#include "logger.h"
#include <Arduino.h>

class Configuration;

struct StrainRaw
{
    float A;
    float B;
    float C;
    float D;

    StrainRaw &operator+=(StrainRaw const &add)
    {
        this->A += add.A;
        this->B += add.B;
        this->C += add.C;
        this->D += add.D;
        return *this;
    }

    StrainRaw &operator/=(float divisor)
    {
        this->A /= divisor;
        this->B /= divisor;
        this->C /= divisor;
        this->D /= divisor;
        return *this;
    }

    StrainRaw &operator-=(StrainRaw const &sub)
    {
        this->A -= sub.A;
        this->B -= sub.B;
        this->C -= sub.C;
        this->D -= sub.D;
        return *this;
    }
};

struct StrainData
{
    float x, y;
    float force;
    int clickStatus;
};

class ADS1220Controller
{
private:
    ADS1220_WE ads0;
    ADS1220_WE ads1;
    Logger *logger_ = nullptr;

    StrainRaw strainOffset_;
    StrainRaw readStrainGauge(bool print = false);
    Configuration *configuration_ = nullptr; // New: Pointer to configuration object

    int restingFinger = 0;

public:
    ADS1220Controller()
        : ads0(PIN_ADS1220_CS0, PIN_ADS1220_DRDY0, true),
          ads1(PIN_ADS1220_CS1, PIN_ADS1220_DRDY1, true) {}

    void init();
    void calibrateZero();

    StrainData read(bool print = false);

    void setLogger(Logger *logger);
    void log(const char *msg);

    void setConfiguration(Configuration *config)
    {
        configuration_ = config;
    } // New: Method to set configuration
};
#endif