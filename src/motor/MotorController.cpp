#include "MotorController.h"

MotorController::MotorController() :
    _stepper(AccelStepper::DRIVER, 0, 0),
    _runBus(),
    _resetBus()
{
}

MotorController::~MotorController() {

}

void MotorController::begin(const std::map<String, uint8_t>& pinMappings) {

    _pinMappings = pinMappings;
    _supply_bus_pin = pinMappings.at("AIR_SUPPLY_BUS_PIN");
    _dump_bus_pin = pinMappings.at("AIR_DUMP_BUS_PIN");
    _run_bus_pin = pinMappings.at("PRGM_RUN_BUS_PIN");
    _reset_bus_pin = pinMappings.at("PRGM_RESET_BUS_PIN");
    _loop_bus_pin = pinMappings.at("LOOP_BUS_PIN");
    _motor_step_pin = pinMappings.at("MOTOR_STEP_PIN");
    _motor_enable_pin = pinMappings.at("MOTOR_ENABLE_PIN");
    _motor_dir_pin = pinMappings.at("MOTOR_DIR_PIN");
    _torque_flag_bus_pin = pinMappings.at("TORQ_FLAG_BUS_PIN");

    pinMode(_motor_enable_pin, OUTPUT);
    pinMode(_torque_flag_bus_pin, OUTPUT);
    pinMode(_run_bus_pin, OUTPUT);
    pinMode(_reset_bus_pin, OUTPUT);

    pinMode(_supply_bus_pin, INPUT_PULLDOWN);
    pinMode(_dump_bus_pin, INPUT_PULLDOWN);
    pinMode(_loop_bus_pin, INPUT_PULLDOWN);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(_motor_enable_pin, HIGH);
    digitalWrite(_torque_flag_bus_pin, LOW);
    digitalWrite(_run_bus_pin, LOW);
    digitalWrite(_reset_bus_pin, LOW);
    
    _stepper = AccelStepper(1, _motor_step_pin, _motor_dir_pin);
    
    _runBus.attach(_run_bus_pin);
    _runBus.interval(100);
    _resetBus.attach(_reset_bus_pin);
    _resetBus.interval(100);

}

void MotorController::enableMotor() {
    digitalWrite(_motor_enable_pin, LOW);
}

void MotorController::setMaxSpeed(const float& max_speed) {
    _stepper.setMaxSpeed(max_speed * _PPR);
}

void MotorController::setSpeed(const float& speed) {
    _stepper.setSpeed(speed * _PPR);
}

void MotorController::setAcceleration(const float& accel) {
    _stepper.setAcceleration(accel * _PPR);
}

void MotorController::moveTo(const uint32_t& target_pos) {
    _stepper.moveTo(target_pos * _PPR);
}

void MotorController::runSpeed() {
    _stepper.runSpeed();
}