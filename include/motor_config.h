#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

const uint16_t SPR = 600;

struct Step {
    bool turnOnHeat;
    bool hold_step;
    double hold_time;
    double target_position;
    double max_speed;
    double accel;
};

const Step steps[] = {
    {false, false, 0.0, 62.37, 94.5, 945},
    {false, true, 0.1, 62.37, 0.0, 0},
    {false, false, 0.0, -62.37, 94.5, 945},
    {false, true, 0.1, -62.37, 0.0, 0},
};

#endif
