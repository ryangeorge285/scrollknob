#include "ads1220_adc.h"
#include "logger.h"
#include <ADS1220_WE.h>
#include <Arduino.h>
#include "configuration.h"          // Include configuration
#include "proto_gen/smartknob.pb.h" // For PB_PersistentConfiguration and PB_StrainRawOffset

void ADS1220Controller::init()
{
    // SPI.begin(PIN_PMW3389_SCK, PIN_PMW3389_MISO, PIN_PMW3389_MOSI);
    SPI.setDataMode(SPI_MODE1); // ADS1220 wants Mode 1
    SPI.setBitOrder(MSBFIRST);
    ads0.setSPIClockSpeed(4000000);
    ads1.setSPIClockSpeed(4000000);

    // Ensure ADS1 CS pin is configured as output and set HIGH to disable unused ADS1
    pinMode(PIN_ADS1220_CS1, OUTPUT);
    digitalWrite(PIN_ADS1220_CS1, HIGH);

    if (!ads0.init())
    {
        log("ADS1220 0 not found!");
        while (1)
            ;
    }

    // if (!ads1.init())
    // {
    //     log("ADS1220 1 not found!");
    //     while (1)
    //         ;
    // }

    ads0.bypassPGA(false);
    ads0.setGain(ADS1220_GAIN_128); // crank it up ×128
    ads0.setOperatingMode(ADS1220_TURBO_MODE);
    ads0.setConversionMode(ADS1220_CONTINUOUS);
    ads0.setDataRate(ADS1220_DR_LVL_3); // 45 SPS; adjust as needed

    // ads1.bypassPGA(false);
    // ads1.setGain(ADS1220_GAIN_128); // crank it up ×128
    // ads1.setOperatingMode(ADS1220_TURBO_MODE);
    // ads1.setConversionMode(ADS1220_CONTINUOUS);
    // ads1.setDataRate(ADS1220_DR_LVL_3); // 45 SPS; adjust as needed

    if (configuration_ != nullptr)
    {
        PB_PersistentConfiguration persistent_config = configuration_->get();
        if (persistent_config.has_strain_offset)
        {
            strainOffset_.A = persistent_config.strain_offset.A;
            strainOffset_.B = persistent_config.strain_offset.B;
            strainOffset_.C = persistent_config.strain_offset.C;
            strainOffset_.D = persistent_config.strain_offset.D;
            if (logger_)
            {
                char buf[200];
                snprintf(buf, sizeof(buf), "ADS: Loaded strain offset from config: A=%.3f, B=%.3f, C=%.3f, D=%.3f", strainOffset_.A, strainOffset_.B, strainOffset_.C, strainOffset_.D);
                log(buf);
            }
        }
        else
        {
            strainOffset_ = {0.0f, 0.0f, 0.0f, 0.0f}; // Default to zeros
            if (logger_)
            {
                log("ADS: No strain raw offset in config, using zeros.");
                calibrateZero();
            }
        }
    }
    else
    {
        strainOffset_ = {0.0f, 0.0f, 0.0f, 0.0f}; // Default to zeros if no config
        if (logger_)
        {
            log("ADS: Config not provided to ADS1220Controller, using zero strain offset.");
            calibrateZero();
        }
    }
}

void ADS1220Controller::calibrateZero()
{
    SPI.setDataMode(SPI_MODE1);

    strainOffset_ = {0.0, 0.0, 0.0, 0.0};
    StrainRaw strainOffset;

    for (int iter = 0; iter < 500; iter++)
    {
        strainOffset += readStrainGauge(true);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    strainOffset /= 500.0f;

    strainOffset_ = strainOffset;

    char buf[100];
    snprintf(buf, sizeof(buf), "STRAIN_OFFSET_A:%f STRAIN_OFFSET_B:%f STRAIN_OFFSET_C:%f STRAIN_OFFSET_D:%f", strainOffset_.A, strainOffset_.B, strainOffset_.C, strainOffset_.D);
    log(buf);

    // Save the new strain offset to configuration
    if (configuration_ != nullptr)
    {
        PB_StrainRawOffset pb_offset_to_save = PB_StrainRawOffset_init_default;
        pb_offset_to_save.A = strainOffset_.A;
        pb_offset_to_save.B = strainOffset_.B;
        pb_offset_to_save.C = strainOffset_.C;
        pb_offset_to_save.D = strainOffset_.D;
        if (configuration_->setStrainRawOffsetAndSave(pb_offset_to_save))
        {
            if (logger_)
                log("ADS: Saved new strain raw offset to configuration.");
        }
        else
        {
            if (logger_)
                log("ADS: Failed to save new strain raw offset to configuration.");
        }
    }
    else
    {
        if (logger_)
            log("ADS: Configuration pointer is null, cannot save strain offset.");
    }
}

StrainRaw ADS1220Controller::readStrainGauge(bool print)
{
    SPI.setDataMode(SPI_MODE1);

    ads0.setCompareChannels(ADS1220_MUX_0_1);
    float mV1 = ads0.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);
    ads0.setCompareChannels(ADS1220_MUX_2_3);
    float mV2 = ads0.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);

    float StrainA;
    float StrainB;
    float StrainC;
    float StrainD;
    if (!(mV1 == 0 || mV2 == 0))
    {
        StrainB = mV1;
        StrainA = mV2;
    }

    // ads1.setCompareChannels(ADS1220_MUX_0_1);
    // mV1 = ads1.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);
    // ads1.setCompareChannels(ADS1220_MUX_2_3);
    // mV2 = ads1.getRawData() * (2.048f / 8388608.0f / 128.0f * 1000.0f);

    // if (!(mV1 == 0 || mV2 == 0))
    // {
    //     StrainD = mV1;
    //     StrainC = mV2;
    // }

    StrainC = 0;
    StrainD = 0;

    StrainRaw result = StrainRaw{StrainA, StrainB, StrainC, StrainD};

    result -= strainOffset_;

    if (print)
    {
        char bufAB[100];
        snprintf(bufAB, sizeof(bufAB), "STRAIN_A:%f STRAIN_B:%f", result.A, result.B);
        // char bufCD[100];
        // snprintf(bufCD, sizeof(bufCD), "STRAIN_C:%f STRAIN_D:%f", result.C, result.D);

        // char buf[100];
        // snprintf(buf, sizeof(buf), "%s %s", bufAB, bufCD);
        log(bufAB);
    }

    return result;
}

StrainData ADS1220Controller::read(bool print)
{
    StrainRaw strain = readStrainGauge();
    StrainData data;

    data.x = -(strain.A - strain.B);
    // data.y = (strain.A - strain.C);
    //  x = rotation_cos * x - rotation_sin * y;
    //  y = rotation_sin * (StrainD - StrainB) + rotation_cos * y;
    data.force = sqrt(pow(strain.A, 2) + pow(strain.B, 2) + pow(strain.C, 2) + pow(strain.D, 2));

    if (data.force < .2)
    {
        restingFinger = 0;
        if (print)
        {
            char buf[100];
            snprintf(buf, sizeof(buf), "X:%f Y:%f Force:%f ClickStatus:%d", data.x, data.y, data.force, data.clickStatus);
            log(buf);
        }
    }
    else if (restingFinger != 1)
    {
        if (data.x < 0.1)
            restingFinger = 1;
        else
            restingFinger = 2;
        if (print)
        {
            char buf[100];
            snprintf(buf, sizeof(buf), "X:%f Y:%f Force:%f ClickStatus:%d", data.x, data.y, data.force, data.clickStatus);
            log(buf);
        }
    }

    data.clickStatus = restingFinger;

    return data;
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