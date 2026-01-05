#include "ads1220_adc.h"
#include "logger.h"
#include <ADS1220_WE.h>
#include <Arduino.h>
#include "configuration.h"          // Include configuration
#include "proto_gen/smartknob.pb.h" // For PB_PersistentConfiguration and PB_StrainRawOffset

bool ADS1220Controller::init()
{
    initialized_ = false;
    has_valid_sample_ = false;
    last_raw_ = {0.0f, 0.0f, 0.0f, 0.0f};
    last_data_ = {};
    current_mux_ = ADS1220_MUX_0_1;

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
        return false;
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

    // Prime the MUX so the first conversion is on the expected channel.
    ads0.setCompareChannels(current_mux_);

    initialized_ = true;

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
    return true;
}

void ADS1220Controller::calibrateZero()
{
    if (!initialized_)
    {
        if (logger_)
        {
            log("ADS: calibrateZero called before init");
        }
        return;
    }

    SPI.setDataMode(SPI_MODE1);

    strainOffset_ = {0.0, 0.0, 0.0, 0.0};
    StrainRaw strainOffset = {0.0f, 0.0f, 0.0f, 0.0f};
    size_t samples = 0;

    // Collect actual ADC samples without blocking other tasks.
    while (samples < 500)
    {
        StrainRaw sample;
        if (readStrainGauge(sample, false))
        {
            strainOffset += sample;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (samples == 0)
    {
        if (logger_)
        {
            log("ADS: calibrateZero failed (no samples)");
        }
        return;
    }

    strainOffset /= static_cast<float>(samples);

    strainOffset_ = strainOffset;
    last_raw_ = strainOffset;

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

bool ADS1220Controller::readStrainGauge(StrainRaw &result, bool print)
{
    if (!initialized_)
    {
        result = StrainRaw{0, 0, 0, 0};
        return false;
    }

    // If data isn't ready yet, don't block this task; just return the last value.
    if (digitalRead(PIN_ADS1220_DRDY0) == HIGH)
    {
        result = last_raw_;
        result -= strainOffset_;
        return false;
    }

    SPI.setDataMode(SPI_MODE1);

    const float scale = (2.048f / 8388608.0f / 128.0f * 1000.0f);
    const float mV = ads0.getRawData() * scale;

    switch (current_mux_)
    {
    case ADS1220_MUX_0_1:
        last_raw_.B = mV;
        break;
    case ADS1220_MUX_2_3:
        last_raw_.A = mV;
        break;
    default:
        break;
    }

    // Set up the next channel and let the conversion happen in the background.
    current_mux_ = (current_mux_ == ADS1220_MUX_0_1) ? ADS1220_MUX_2_3 : ADS1220_MUX_0_1;
    ads0.setCompareChannels(current_mux_);

    has_valid_sample_ = true;

    result = last_raw_;
    result -= strainOffset_;

    if (print)
    {
        char bufAB[100];
        snprintf(bufAB, sizeof(bufAB), "STRAIN_A:%f STRAIN_B:%f", result.A, result.B);
        log(bufAB);
    }

    return true;
}

bool ADS1220Controller::read(StrainData &data, bool print)
{
    if (!initialized_)
    {
        data = StrainData{};
        return false;
    }

    StrainRaw strain{};
    const bool new_sample = readStrainGauge(strain, print);

    if (!has_valid_sample_)
    {
        data = StrainData{};
        return false;
    }

    // If no new sample was consumed, reuse the previous reading.
    if (!new_sample)
    {
        data = last_data_;
        data.clickStatus = restingFinger;
        return false;
    }

    data.x = (strain.A - strain.B);
    // data.y = (strain.A - strain.C);
    //  x = rotation_cos * x - rotation_sin * y;
    //  y = rotation_sin * (StrainD - StrainB) + rotation_cos * y;
    data.force = sqrt(pow(strain.A, 2) + pow(strain.B, 2) + pow(strain.C, 2) + pow(strain.D, 2));

    if (data.force < .2f)
    {
        restingFinger = 0;
    }
    else if (restingFinger != 1)
    {
        restingFinger = (data.x < 0) ? 1 : 2;
    }

    data.clickStatus = restingFinger;
    last_data_ = data;

    return true;
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
