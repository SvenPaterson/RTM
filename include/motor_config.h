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
    {true, 1000, false, 500, 2},
    {true, 300, false, 350, 2},
    {true, 1000, false, 350, 2},
    {true, 300, false, 350, 2},
    {true, 1000, false, 350, 2},
    {true, 300, false, 350, 2},
    {true, 1000, false, 350, 2},
    {true, 300, false, 350, 2},
    {true, 1000, false, 350, 2},
    {true, 300, false, 350, 2},
    {true, 800, false, 500, 11},
    {true, 0, false, 800, 11},
    {true, 1000, true, 500, 2},
    {true, 300, true, 350, 2},
    {true, 1000, true, 350, 2},
    {true, 300, true, 350, 2},
    {true, 1000, true, 350, 2},
    {true, 300, true, 350, 2},
    {true, 1000, true, 350, 2},
    {true, 300, true, 350, 2},
    {true, 1000, true, 350, 2},
    {true, 300, true, 350, 2},
    {true, 800, true, 500, 11},
    {true, 0, true, 800, 11},
};

#endif
