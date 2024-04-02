#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

const bool isHighSpeedGearBox = false;
const uint16_t SPR = 800;

struct Step {
    bool turnOnHeat;
    double target_speed;
    bool is_CCW;
    double accel;
    uint32_t time;
};

const Step steps[] = {
    {false, 1000, true, 500, 12},
    {false, 0, true, 250, 5},
    {false, 1000, true, 500, 12},
    {false, 0, true, 250, 5},
    {false, 500, true, 500, 12},
    {false, 0, true, 250, 5},
    {false, 500, true, 500, 12},
    {false, 0, true, 250, 5},
};

#endif
