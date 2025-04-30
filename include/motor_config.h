#ifndef motor_config_h
#define motor_config_h

#include <cstdint>

/******* I/O PINS *******/
#define PRGM_RUN_BUS_PIN ConnectorDI6
#define LED_PIN ConnectorIO0
#define MOTOR_ENABLE_PIN ConnectorIO2
#define PRGM_RESET_BUS_PIN ConnectorDI7
#define SerialPort ConnectorUsb
#define SAFETY_PIN ConnectorDI8

//#define MAX_SPEED 1800          // RPM
//#define SWEEP_ACCEL 30          // RPM/sec
//#define BREAKIN_DURATION 300     // sec
//#define BASELINE_SPEED 30       // RPM
//#define BASELINE_DURATION 20     // sec
//#define DWELL_BETWEEN_STEPS 1   // sec

/* const Step torque_steps[] = {
    // 1. CW TORQUE BASELINE
    {BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, BASELINE_SPEED, DWELL_BETWEEN_STEPS},

    // 2. CW BREAKIN
    {MAX_SPEED, MAX_SPEED / 2, BREAKIN_DURATION},
    {0, MAX_SPEED / 2, DWELL_BETWEEN_STEPS},

    // 3. CW
    {BASELINE_SPEED, BASELINE_SPEED, BASELINE_DURATION},
    {0, BASELINE_SPEED, DWELL_BETWEEN_STEPS},

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
}; */

#endif