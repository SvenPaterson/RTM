#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

const uint16_t SPR = 200;

struct Step {
    bool turnOnHeat;
    bool hold_step;
    double hold_time;
    double target_position;
    double max_speed;
    double accel;
};

const Step steps[] = {
    {true, true, 1.0, 180, 100, 500},
    {true, true, 3.0, -35, 300, 350},
    {true, true, 5.4, -90, 150, 350},
    {true, true, 10.0, 260, 50, 350},
    {true, true, 20.0, 50, 20, 350},
};

#endif
