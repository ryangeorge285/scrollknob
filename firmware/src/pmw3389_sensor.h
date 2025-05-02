#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "logger.h"

// ADNS register map
#define Product_ID 0x00
#define Inverse_Product_ID 0x3F
#define SROM_ID 0x2A
#define Motion 0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Config2 0x10
#define Resolution_L 0x0F
#define Resolution_H 0x10
#define SROM_Enable 0x13
#define Power_Up_Reset 0x3A
#define Shutdown 0x3B
#define Motion_Burst 0x50
#define SROM_Load_Burst 0x62

class PMW3389
{
private:
    bool inBurst = false;

    uint8_t adns_read_reg(uint8_t reg);
    void adns_write_reg(uint8_t reg, uint8_t val);
    void adns_upload_firmware();
    void setCPI(uint16_t cpi);

    Logger *logger_ = nullptr;

public:
    void init();
    void enableBurst();

    inline void readMouseMovement(bool &motion, int16_t &dx, int16_t &dy)
    {
        byte burst[12];

        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
        digitalWrite(PIN_PMW3389_NC, LOW);
        SPI.transfer(Motion_Burst);
        delayMicroseconds(35);
        SPI.transfer(burst, sizeof(burst));
        digitalWrite(PIN_PMW3389_NC, HIGH);
        SPI.endTransaction();

        motion = burst[0] & 0x80;
        bool offSurface = burst[0] & 0x08;
        int16_t x = (burst[3] << 8) | burst[2];
        int16_t y = (burst[5] << 8) | burst[4];
        dx += x;
        dy += y;

        if (motion || abs(dx) > 10 || abs(dy) > 10)
        {
            char buf[100];
            snprintf(buf, sizeof(buf), "Burst: motion=%d offSurface=%s dx=%d dy=%d SQUAL=%u",
                     motion, offSurface ? "true" : "false", dx, dy, burst[6]);
            log(buf);
        }
    }

    void setLogger(Logger *logger);
    void log(const char *msg);
};