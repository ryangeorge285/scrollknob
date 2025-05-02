#pragma once
#include <sensors/MagneticSensorSPI.h>

/** Configuration for 14‑bit AS5048A magnetic encoder over SPI */
MagneticSensorSPIConfig_s AS5048A_SPI = {
    // CPOL=0, CPHA=1 per datasheet
    .spi_mode = SPI_MODE1,
    // up to 10 MHz (we’ll pick 8 MHz here as a safe margin)
    .clock_speed = 8000000,
    // 14 bits of angular data
    .bit_resolution = 14,
    // address of the ANGLE register
    .angle_register = 0x3FFF,
    // index of the MSB of the 14‑bit result in the 16‑bit frame
    .data_start_bit = 14,
    // position of the R/W bit in the 16‑bit command word
    .command_rw_bit = 15,
    // position of the parity bit in the 16‑bit data word (LSB)
    .command_parity_bit = 0};