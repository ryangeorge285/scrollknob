#include "ads1220_adc.h"
#include "logger.h"
#include <ADS1220_WE.h>
#include <Arduino.h>

void ADS1220Controller::init()
{
    // SPI.begin(PIN_PMW3389_SCK, PIN_PMW3389_MISO, PIN_PMW3389_MOSI);
    SPI.setDataMode(SPI_MODE1); // ADS1220 wants Mode 1
    SPI.setBitOrder(MSBFIRST);
    ads0.setSPIClockSpeed(4000000);
    ads1.setSPIClockSpeed(4000000);

    if (!ads0.init())
    {
        log("ADS1220 0 not found!");
        while (1)
            ;
    }

    if (!ads1.init())
    {
        log("ADS1220 1 not found!");
        while (1)
            ;
    }

    ads0.bypassPGA(false);
    ads0.setGain(ADS1220_GAIN_128); // crank it up ×128
    ads0.setOperatingMode(ADS1220_TURBO_MODE);
    ads0.setConversionMode(ADS1220_CONTINUOUS);
    ads0.setDataRate(ADS1220_DR_LVL_3); // 45 SPS; adjust as needed

    ads1.bypassPGA(false);
    ads1.setGain(ADS1220_GAIN_128); // crank it up ×128
    ads1.setOperatingMode(ADS1220_TURBO_MODE);
    ads1.setConversionMode(ADS1220_CONTINUOUS);
    ads1.setDataRate(ADS1220_DR_LVL_3); // 45 SPS; adjust as needed
}

void ADS1220Controller::calibrate()
{
    SPI.setDataMode(SPI_MODE1);
    for (int i = 0; i < 100; i++)
    {
        readStrainGauge(true);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

float ADS1220Controller::readStrainGauge(bool print)
{
    SPI.setDataMode(SPI_MODE1);

    ads0.setCompareChannels(ADS1220_MUX_0_1);
    float mV1 = ads0.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);
    ads0.setCompareChannels(ADS1220_MUX_2_3);
    float mV2 = ads0.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);

    char bufAB[100];

    float StrainA;
    float StrainB;
    float StrainC;
    float StrainD;
    if (!(mV1 == 0 || mV2 == 0))
    {
        StrainB = mV1;
        StrainA = mV2;
        if (print)
        {
            snprintf(bufAB, sizeof(bufAB), "STRAIN_A:%f STRAIN_B:%f", StrainA, StrainB);
        }
    }

    ads1.setCompareChannels(ADS1220_MUX_0_1);
    mV1 = ads1.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);
    ads1.setCompareChannels(ADS1220_MUX_2_3);
    mV2 = ads1.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);

    if (!(mV1 == 0 || mV2 == 0))
    {
        StrainD = mV1;
        StrainC = mV2;
        if (print)
        {
            char bufCD[100];
            snprintf(bufCD, sizeof(bufCD), "STRAIN_C:%f STRAIN_D:%f", StrainC, StrainD);

            char buf[100];
            snprintf(buf, sizeof(buf), "%s %s", bufAB, bufCD);
            log(buf);
        }
    }

    return (StrainA + StrainB + StrainC + StrainD) / 4.0f;
}

void ADS1220Controller::log(const char *msg)
{
    if (logger_ != nullptr)
    {
        logger_->log(msg);
    }
}

void ADS1220Controller::setLogger(Logger *logger)
{
    logger_ = logger;
}