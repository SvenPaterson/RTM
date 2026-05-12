// Max31855Min.h — Minimal MAX31855 thermocouple driver (XPB-only).
// Replaces Adafruit_MAX31855 to save flash on the Nano Every. We only
// need the hot-junction °C value and a fault flag; the cold-junction
// reading and per-bit fault decoding from the Adafruit library are
// unused and drag in float math we don't need.
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

class Max31855Min {
public:
    explicit Max31855Min(uint8_t cs) : cs_(cs) {}

    /// @brief Configure CS pin (idle HIGH). SPI.begin() handled elsewhere.
    void begin() {
        pinMode(cs_, OUTPUT);
        digitalWrite(cs_, HIGH);
    }

    /// @brief Read hot-junction temperature in °C.
    /// @return Temperature in °C (resolution 0.25), or NAN on any fault
    ///         (D16 fault bit set: open / short-GND / short-VCC).
    float readCelsius() {
        digitalWrite(cs_, LOW);
        SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE0));
        uint32_t v = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            v = (v << 8) | (uint32_t)SPI.transfer(0x00);
        }
        SPI.endTransaction();
        digitalWrite(cs_, HIGH);

        // D16 = consolidated fault bit
        if (v & 0x00010000UL) return NAN;

        // Upper 14 bits (D31..D18) are signed hot-junction temp,
        // resolution 0.25 °C. Sign-extend via int16_t arithmetic shift.
        int16_t raw = (int16_t)(v >> 16);
        raw >>= 2;
        return (float)raw * 0.25f;
    }

private:
    uint8_t cs_;
};
