// MiniDebounce.h - Tiny pin debouncer for XPB switches.
// Replaces Bounce2 (~600B) with a struct-only equivalent of the API
// surface we actually use: attach/interval/update/read/changed/fell.
#pragma once

#include <Arduino.h>

class MiniDebounce {
public:
    void attach(uint8_t pin, uint8_t mode) {
        pin_ = pin;
        pinMode(pin_, mode);
        const uint8_t v = (uint8_t)digitalRead(pin_);
        state_     = v;
        lastRead_  = v;
        prevState_ = v;
        lastEdgeMs_ = millis();
    }
    void interval(uint16_t ms) { intervalMs_ = ms; }

    /// @return true if the debounced state changed this update().
    bool update() {
        const uint8_t v = (uint8_t)digitalRead(pin_);
        if (v != lastRead_) {
            lastRead_   = v;
            lastEdgeMs_ = millis();
        }
        prevState_ = state_;
        if ((uint16_t)(millis() - lastEdgeMs_) >= intervalMs_) {
            state_ = lastRead_;
        }
        return state_ != prevState_;
    }

    bool changed() const { return state_ != prevState_; }
    bool fell()    const { return changed() && state_ == LOW; }
    bool rose()    const { return changed() && state_ == HIGH; }
    int  read()    const { return state_; }

private:
    uint8_t  pin_         = 0;
    uint8_t  state_       = HIGH;
    uint8_t  prevState_   = HIGH;
    uint8_t  lastRead_    = HIGH;
    uint16_t intervalMs_  = 25;
    uint32_t lastEdgeMs_  = 0;
};
