#include "LCDDriver.h"

LCDDriver::LCDDriver(uint8_t csPin) : LCD_CS_(csPin) {
    // Constructor init list sets the pin
}

// public implimentation

bool LCDDriver::begin() {
    Serial.print("Initializing NHD-0420D3Z-NSW-BBW-V3 display... ");

    pinMode(LCD_CS_, OUTPUT);
    digitalWrite(LCD_CS_, HIGH);
    SPI.begin();
    delay(200);

    clearScreen();
    displayOn();
    setBrightness();

    setLineLR(0, "LCD Screen", "Ready");
    flush();
    delay(1000);

    Serial.println("DONE");
    return true;
}

void LCDDriver::displayOn() {
    sendCommand(0x41);
}

void LCDDriver::displayOff() {
    sendCommand(0x42);
}

void LCDDriver::setBrightness(uint8_t level) {
    // Clamp to valid range (1-8)
    level = (level < 1) ? 1 : (level > 8) ? 8 : level;
    sendCommand(0x53, &level, 1);
}

void LCDDriver::setLine(uint8_t row, const char* text, TextAlign align) {
    if (row >= kNumRows) return;
    
    blankLine(frontBuffer_[row]);
    uint8_t len = getStringLength(text);
    
    switch (align) {
        case TextAlign::LEFT:
            memcpy(frontBuffer_[row], text, len);
            break;
            
        case TextAlign::RIGHT:
            memcpy(frontBuffer_[row] + kNumCols - len, text, len);
            break;
            
        case TextAlign::CENTER: {
            uint8_t start = (kNumCols - len) / 2;
            memcpy(frontBuffer_[row] + start, text, len);
            break;
        }
            
    }
    
    if (memcmp(frontBuffer_[row], sentBuffer_[row], kNumCols)) {
        isDirty_[row] = true;
    }
}

void LCDDriver::setLineLR(uint8_t row, const char* left, const char* right) {
    if (row >= kNumRows) return;
    
    blankLine(frontBuffer_[row]);
    uint8_t rightLen = getStringLength(right);
    uint8_t leftMax = (rightLen < kNumCols) ? kNumCols - rightLen - 1 : 0;
    uint8_t leftLen = (leftMax ? (getStringLength(left) > leftMax ? leftMax : getStringLength(left)) : 0);
    
    memcpy(frontBuffer_[row], left, leftLen);
    memcpy(frontBuffer_[row] + kNumCols - rightLen, right, rightLen);
    
    if (memcmp(frontBuffer_[row], sentBuffer_[row], kNumCols)) {
        isDirty_[row] = true;
    }
}

void LCDDriver::flush() {
    for (uint8_t row = 0; row < kNumRows; ++row) {
        if (!isDirty_[row]) continue;

        // Move cursor to start of row
        sendCommand(0x45, &rowAddresses_[row], 1);

        // Send the 20 characters with required delay
        sendData(frontBuffer_[row], kNumCols);

        // Mark row as clean
        memcpy(sentBuffer_[row], frontBuffer_[row], kNumCols);
        isDirty_[row] = false;
    }
}

void LCDDriver::clearScreen() {
    // sendCommand knows that 0x51 needs a ≥1.5 ms pause
    sendCommand(0x51);

    // clear our “sent_” buffer so every line shows up dirty next flush
    for (uint8_t i = 0; i < kNumRows; ++i) {
        blankLine(sentBuffer_[i]);
    }
}

// private implimentation
void LCDDriver::sendCommand(uint8_t cmd, const uint8_t* params, uint8_t paramLen) {
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(spiSettings_);

    SPI.transfer(0xFE);
    SPI.transfer(cmd);

    for (uint8_t i = 0; i < paramLen; ++i) {
        SPI.transfer(params[i]);
    }

    uint16_t execTime = getExecTime_ms(cmd);
    if (execTime >= 1000) {
        delay(execTime / 1000);
    } else {
        delayMicroseconds(execTime);
    }

    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
}

void LCDDriver::sendData(const char* data, size_t len) {
    digitalWrite(LCD_CS_, LOW);
    SPI.beginTransaction(spiSettings_);
    
    for (size_t i = 0; i < len; ++i) {
        SPI.transfer(data[i]);
        delayMicroseconds(100);
    }
    
    SPI.endTransaction();
    digitalWrite(LCD_CS_, HIGH);
}

uint16_t LCDDriver::getExecTime_ms(uint8_t cmd) const {
    switch (cmd) {
        case 0x70: return 4000;   // Display Firmware
        case 0x46: case 0x47: case 0x48: case 0x51: return 1500;  // 1.5ms commands
        case 0x52: return 500;    // Contrast
        default: return 100;      // 0.1ms for others
    }
}

void LCDDriver::blankLine(char* dst) {
    memset(dst, ' ', kNumCols);
    dst[kNumCols] = '\0';
}

uint8_t LCDDriver::getStringLength(const char* s) const {
    uint8_t len = 0;
    while (len < kNumCols && s[len]) ++len;
    return len;
}