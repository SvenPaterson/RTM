#ifndef MC_h
#define MC_h

#include <Arduino.h>
#include <AccelStepper.h>
#include <map>
#include <Bounce2.h>

class MotorController {
    public:
        MotorController();
        ~MotorController();

        void begin(const std::map<String, uint8_t>& pinMappings);

        void enableMotor();
        /**
         * @brief Set the Max Speed in rpm
         * 
         * @param max_speed speed in rev per minute
         */
        void setMaxSpeed(const float& max_speed);

        /**
         * @brief Set the current speed in rpm
         * 
         * @param speed speed in rev per minute
         */
        void setSpeed(const float& speed);

        void setAcceleration(const float& accel);
        void moveTo(const float& target_pos);
        void runSpeed();

        void update();

    private:
        std::map<String, uint8_t> _pinMappings;
        uint8_t _supply_bus_pin, _dump_bus_pin, _run_bus_pin;
        uint8_t _reset_bus_pin, _loop_bus_pin, _motor_step_pin;
        uint8_t _motor_enable_pin, _motor_dir_pin, _torque_flag_bus_pin;
        uint8_t _LED_pin;

        AccelStepper _stepper;
        uint8_t _PPR = 200;

        Bounce _runBus;
        Bounce _resetBus;

        bool _running, _rst_request;

};

#endif //DC_h