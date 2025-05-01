#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

// Steps per Revolution
#define MOTOR_MAX_VEL_RPM 3175 // 2760rpm for CPM-SDHP-N0563A-ELN
const uint16_t SPR = 3200;

#define MAX_SPEED 1800          // RPM
#define SWEEP_ACCEL 30          // RPM/sec
#define BREAKIN_DURATION 300     // sec
#define BASELINE_SPEED 30       // RPM
#define BASELINE_DURATION 20     // sec
#define DWELL_BETWEEN_STEPS 1   // sec

// Struct to define each step
struct Step {
    int32_t target_speed; 
    uint32_t accel;        // Acceleration in RPM/sec
    uint32_t dwell_time;   // amount of time to dwell after target speed is reached in sec
};

const Step torque_steps[] = {
    // 1. CW TORQUE BASELINE
    {BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, BASELINE_SPEED, DWELL_BETWEEN_STEPS},

    // 2. CW BREAKIN
    {MAX_SPEED, MAX_SPEED / 2, BREAKIN_DURATION},
    {0, MAX_SPEED / 2, DWELL_BETWEEN_STEPS},

    // 3. CW
    {BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, 30, DWELL_BETWEEN_STEPS},

    // 4. CW SWEEP
    {MAX_SPEED, SWEEP_ACCEL, DWELL_BETWEEN_STEPS},
    {0, SWEEP_ACCEL, DWELL_BETWEEN_STEPS},

    // 5. Run baseline again CW
    {BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, 30, DWELL_BETWEEN_STEPS},

    // 6. Run baseline again CCW
    {-BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, 30, DWELL_BETWEEN_STEPS},

    // 7. CCW SWEEP
    {-MAX_SPEED, SWEEP_ACCEL, DWELL_BETWEEN_STEPS},
    {0, SWEEP_ACCEL, DWELL_BETWEEN_STEPS},

    // 8. Run baseline again but CCW
    {-BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, 30, DWELL_BETWEEN_STEPS}
};

/* 
// DANA TEST PROTOCOL
const Step torque_steps[] = {
    {   10,    5, 60 },
    {   30,   15, 60 },
    {   45,   22, 60 },
    {   60,   30, 60 },
    {  100,   50, 60 },
    {  200,  100, 60 },
    {  300,  150, 60 },
    {  400,  200, 60 },
    {   60,   30, 60 },
    {   30,   15, 60 },
    {   10,    5, 60 },
    {   30,   15, 60 },
    {   60,   30, 60 },
    {  100,   50, 60 },
    {  500,  250, 60 },
    { 1000,  500, 60 },
    { 1500,  750, 60 },
    { 2000, 1000, 60 },
    { 2750, 1375, 60 },
}; 
*/

#endif