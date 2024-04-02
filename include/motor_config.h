#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

const bool isHighSpeedGearBox = true;
const uint16_t SPR = 600;

struct Step {
    bool turnOnHeat;
    double target_speed;
    bool is_CCW;
    double accel;
    uint32_t time;
};

const Step steps[] = {
    {true, 3000, false, 1000, 13000},
    {true, 0, false, 1000, 8000},
    {true, 3000, true, 1000, 13000},
    {true, 0, true, 1000, 8000},
    {false, 1500, false, 1000, 13000},
    {false, 0, false, 1000, 8000},
    {false, 1500, true, 1000, 13000},
    {false, 0, true, 1000, 8000},
};

#endif
