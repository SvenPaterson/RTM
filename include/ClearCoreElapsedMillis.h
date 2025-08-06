// elapsedMillis.h (for ClearCore)
#pragma once
#include <ClearCore.h>

class elapsedMillis {
private:
    uint32_t ms;
public:
    elapsedMillis(void) { ms = Milliseconds(); }
    elapsedMillis(uint32_t val) { ms = Milliseconds() - val; }
    elapsedMillis(const elapsedMillis &orig) { ms = orig.ms; }
    operator uint32_t () const { return Milliseconds() - ms; }
    elapsedMillis & operator = (const elapsedMillis &rhs) { ms = rhs.ms; return *this; }
    elapsedMillis & operator = (uint32_t val) { ms = Milliseconds() - val; return *this; }
    elapsedMillis & operator -= (uint32_t val) { ms += val ; return *this; }
    elapsedMillis & operator += (uint32_t val) { ms -= val ; return *this; }
    elapsedMillis operator - (int val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned int val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (long val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned long val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator + (int val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned int val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
};

class elapsedMicros {
private:
    uint32_t us;
public:
    elapsedMicros(void) { us = Microseconds(); }
    elapsedMicros(uint32_t val) { us = Microseconds() - val; }
    elapsedMicros(const elapsedMicros &orig) { us = orig.us; }
    operator uint32_t () const { return Microseconds() - us; }
    elapsedMicros & operator = (const elapsedMicros &rhs) { us = rhs.us; return *this; }
    elapsedMicros & operator = (uint32_t val) { us = Microseconds() - val; return *this; }
    elapsedMicros & operator -= (uint32_t val) { us += val ; return *this; }
    elapsedMicros & operator += (uint32_t val) { us -= val ; return *this; }
    elapsedMicros operator - (int val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (unsigned int val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (long val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator + (int val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (unsigned int val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (long val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
};

class elapsedSeconds {
private:
    uint32_t s;
public:
    elapsedSeconds(void) { s = Milliseconds()/1000; }
    elapsedSeconds(uint32_t val) { s = (Milliseconds()/1000) - val; }
    elapsedSeconds(const elapsedSeconds &orig) { s = orig.s; }
    operator uint32_t () const { return (Milliseconds()/1000) - s; }
    elapsedSeconds & operator = (const elapsedSeconds &rhs) { s = rhs.s; return *this; }
    elapsedSeconds & operator = (uint32_t val) { s = (Milliseconds()/1000) - val; return *this; }
    elapsedSeconds & operator -= (uint32_t val) { s += val ; return *this; }
    elapsedSeconds & operator += (uint32_t val) { s -= val ; return *this; }
    elapsedSeconds operator - (int val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (unsigned int val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (long val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (unsigned long val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator + (int val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (unsigned int val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (long val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (unsigned long val) const { elapsedSeconds r(*this); r.s -= val; return r; }
};