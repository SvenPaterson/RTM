#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

// Steps per Revolution
const uint16_t SPR = 800; // Assuming 800 steps per revolution

// Struct to define each step
struct Step {
    bool is_CCW;         // Direction: true for counter-clockwise (CCW), false for clockwise (CW)
    double target_speed; // Target speed in RPM
    double accel;        // Acceleration in RPM/s
    double time;         // Duration of the step in seconds
};

// Updated Steps Array
const Step steps[] = {
    // 1. Baseline: 20s at 30 RPM with accel of 30 RPM/s CW, then stop
    {false, 30.0, 30.0, 19.0},
    {false, 0.0, 30.0, 1.0},

    // 2. 5 minutes at 1800 RPM with accel of 300 RPM/s CW, then stop
    {false, 1800.0, 300.0, 300.0},
    {false, 0.0, 600.0, 3.0},

    // 3. Run baseline again CW, then stop
    {false, 30.0, 30.0, 19.0},
    {false, 0.0, 30.0, 1.0},

    // 4. Sweep from 0 to 1800 to 0 RPM at 30 RPM/s CW for however long that takes
    {false, 1800.0, 30.0, 60.0}, // Sweep up to 1800 RPM
    {false, 0.0, 30.0, 60.0},    // Sweep down to 0 RPM

    // 5. Run baseline again CW
    {false, 30.0, 30.0, 19.0},
    {false, 0.0, 30.0, 1.0},

    // 6. Run baseline again CCW
    {true, 30.0, 30.0, 19.0},
    {true, 0.0, 30.0, 1.0},

    // 7. Run sweep again but CCW
    {true, 1800.0, 30.0, 60.0},  // Sweep up to 1800 RPM CCW
    {true, 0.0, 30.0, 60.0},     // Sweep down to 0 RPM CCW

    // 8. Run baseline again but CCW
    {true, 30.0, 30.0, 19.0},
    {true, 0.0, 30.0, 1.0}
};

#endif
