// LCDDriver.h - a pure hardware interface to NHD-0420D3Z-NSW-BBW-V3

#pragma once

#include <Arduino.h>
#include <SPI.h>

enum class TextAlign { LEFT, RIGHT, CENTER};

class LCDDriver {
    public:
        // init hardware
        LCDDriver(uint8_t csPin);
        bool begin();

        // display size
        static constexpr uint8_t kNumCols = 20;
        static constexpr uint8_t kNumRows = 4;

        // display control
        void displayOn();
        void displayOff();
        void setBrightness(uint8_t level = 8);
        void clearScreen();

        // send all pending changes to screen
        void flush();

        // text output functions
        void setLine(uint8_t row, const char* text, TextAlign align);

        inline void setLineLeft(uint8_t row, const char* text)      { setLine(row, text, TextAlign::LEFT); }
        inline void setLineRight(uint8_t row, const char* text)     { setLine(row, text, TextAlign::RIGHT); }
        inline void setLineCenter(uint8_t row, const char* text)    { setLine(row, text, TextAlign::CENTER); }
        void setLineLR(uint8_t row, const char* left, const char* right);

    private:

        const uint8_t LCD_CS_;

        const SPISettings spiSettings_{100000, MSBFIRST, SPI_MODE3};
        const uint8_t rowAddresses_[kNumRows] = {0x00, 0x40, 0x14, 0x54};

        char frontBuffer_[kNumRows][kNumCols + 1] = {};
        char sentBuffer_[kNumRows][kNumCols + 1] = {};
        bool isDirty_[kNumRows] = {true, true, true, true};

        // Helpers
        void sendCommand(uint8_t cmd, const uint8_t* params = nullptr, uint8_t paramLen = 0);
        void sendData(const char* data, size_t len);
        uint16_t getExecTime_us(uint8_t cmd) const;
        void blankLine(char* dst);
        uint8_t getStringLength(const char* s) const;

};

