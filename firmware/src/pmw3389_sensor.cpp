#include "pmw3389_sensor.h"
#include <Arduino.h>
#include <SPI.h>
#include "pmw3389_firmware.h"

void PMW3389::init()
{
    pinMode(PIN_PMW3389_NC, OUTPUT);
    // pinMode(RESET_PIN, INPUT_PULLUP);

    SPI.begin(PIN_VSPI_SCK, PIN_VSPI_MISO, PIN_VSPI_MOSI);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);

    digitalWrite(PIN_PMW3389_NC, HIGH);
    digitalWrite(PIN_PMW3389_NC, LOW);
    digitalWrite(PIN_PMW3389_NC, HIGH);

    adns_write_reg(Shutdown, 0xB6);
    delay(300);
    digitalWrite(PIN_PMW3389_NC, LOW);

    delayMicroseconds(40);
    digitalWrite(PIN_PMW3389_NC, HIGH);

    delayMicroseconds(40);
    adns_write_reg(Power_Up_Reset, 0x5A);
    delay(50);

    // discard initial reads
    adns_read_reg(Motion);
    adns_read_reg(Delta_X_L);
    adns_read_reg(Delta_X_H);
    adns_read_reg(Delta_Y_L);
    adns_read_reg(Delta_Y_H);

    adns_upload_firmware();
    delay(10);
    setCPI(800); // default to 800 CPI
    adns_write_reg(Motion_Control, 0x01);
    log("Optical Chip Initialized");
    // DBG_PRINTLN("=== startup complete ===");
}

uint8_t PMW3389::adns_read_reg(uint8_t reg)
{
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_PMW3389_NC, LOW);

    SPI.transfer(reg & 0x7F);
    delayMicroseconds(35);
    uint8_t val = SPI.transfer(0);
    delayMicroseconds(1);
    digitalWrite(PIN_PMW3389_NC, HIGH);

    SPI.endTransaction();
    delayMicroseconds(19);
    // DBG_PRINTF("R 0x%02X -> 0x%02X\n", reg, val);
    return val;
}

void PMW3389::adns_write_reg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_PMW3389_NC, LOW);

    SPI.transfer(reg | 0x80);
    SPI.transfer(val);
    digitalWrite(PIN_PMW3389_NC, HIGH);

    SPI.endTransaction();
    delayMicroseconds(100);
    // DBG_PRINTF("W 0x%02X <- 0x%02X\n", reg, val);
}

void PMW3389::adns_upload_firmware()
{
    // DBG_PRINTLN("=== Uploading firmware ===");
    adns_write_reg(Config2, 0x00);
    adns_write_reg(SROM_Enable, 0x1D);
    delay(10);
    adns_write_reg(SROM_Enable, 0x18);

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_PMW3389_NC, LOW);

    SPI.transfer(SROM_Load_Burst | 0x80);
    delayMicroseconds(15);
    for (uint16_t i = 0; i < firmware_length; i++)
    {
        SPI.transfer(firmware_data[i]);
        // if ((i & 0x3FF) == 0) DBG_PRINTF("  uploaded %u/%u\n", i, firmware_length);
        delayMicroseconds(15);
    }
    digitalWrite(PIN_PMW3389_NC, HIGH);

    SPI.endTransaction();

    adns_read_reg(SROM_ID);
    log("PMW3389 firmware upload complete");
}

void PMW3389::setCPI(uint16_t cpi)
{
    unsigned cpival = cpi / 50;

    adns_write_reg(Resolution_L, (cpival & 0xFF));
    adns_write_reg(Resolution_H, ((cpival >> 8) & 0xFF));

    char buf[100];
    snprintf(buf, sizeof(buf), " -> %u CPI", cpival * 50);
    log(buf);
}

void PMW3389::enableBurst()
{
    adns_write_reg(Motion_Burst, 0x00);
}

void PMW3389::shutdown()
{
    // Configure motion detection to remain active during low power
    // Set the configuration register to enable motion interrupt
    adns_write_reg(Config2, 0x20); // Enable REST mode with motion detection

    // Put sensor in low power mode while keeping motion detection active
    adns_write_reg(Shutdown, 0xB0); // Low power mode with motion detection

    // Ensure CS is high (deselected)
    digitalWrite(PIN_PMW3389_NC, HIGH);

    // Make sure the motion pin is in input mode to detect interrupts
    pinMode(PIN_PMW3389_MOTION, INPUT_PULLUP);

    log("PMW3389 entered low power mode with motion detection enabled");
}

void PMW3389::log(const char *msg)
{
    if (logger_ != nullptr)
    {
        logger_->log(msg);
    }
}

void PMW3389::setLogger(Logger *logger)
{
    logger_ = logger;
}