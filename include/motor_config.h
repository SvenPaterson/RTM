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
    {false, 2000, false, 500, 60},
    {false, 4000, false, 500, 60},
    {false, 6000, false, 500, 60},
    {false, 8000, false, 500, 60},
    {false, 2000, false, 500, 60},
    {false, 4000, false, 500, 60},
    {false, 6000, false, 500, 60},
    {false, 8000, false, 500, 60},
    {false, 2000, false, 500, 60},
    {false, 4000, false, 500, 60},
    {false, 6000, false, 500, 60},
    {false, 8000, false, 500, 60},
};

#endif
