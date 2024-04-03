#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

const bool isHighSpeedGearBox = true;
const uint16_t SPR = 200;

struct Step {
    bool turnOnHeat;
    uint16_t target_speed;
    bool is_CCW;
    uint16_t accel;
    uint32_t time;
};

const Step steps[] = {
    {true, 6000, false, 3000, 2},
    {true, 2000, false, 2000, 2},
    {true, 6000, false, 2000, 2},
    {true, 2000, false, 2000, 2},
    {true, 6000, false, 2000, 2},
    {true, 2000, false, 2000, 2},
    {true, 6000, false, 2000, 2},
    {true, 2000, false, 2000, 2},
    {true, 6000, false, 2000, 2},
    {true, 2000, false, 2000, 2},
    {true, 4000, false, 2000, 11},
    {true, 0, false, 4000, 11},
    {true, 6000, true, 3000, 2},
    {true, 2000, true, 2000, 2},
    {true, 6000, true, 2000, 2},
    {true, 2000, true, 2000, 2},
    {true, 6000, true, 2000, 2},
    {true, 2000, true, 2000, 2},
    {true, 6000, true, 2000, 2},
    {true, 2000, true, 2000, 2},
    {true, 6000, true, 2000, 2},
    {true, 2000, true, 2000, 2},
    {true, 4000, true, 2000, 11},
    {true, 0, true, 4000, 11},
};

#endif
