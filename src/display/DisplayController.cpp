#include "DisplayController.h"

#define RGB_WHITE 255, 255, 255

DisplayController::DisplayController() : _heaterPIDControl(&_input, &_output, &_setpoint_temp, _Kp, _Ki, _Kd, DIRECT)
{
    _setpoint_temp = 70.0;
    _heaterPIDControl.SetMode(AUTOMATIC);
    _heaterPIDControl.SetOutputLimits(0, 150);
}

void DisplayController::setHeaterPins(byte safety_pin, byte output_pin) {
    _safety_pin = safety_pin;
    _output_pin = output_pin;
    pinMode(_safety_pin, OUTPUT);
    pinMode(_output_pin, OUTPUT);
    digitalWrite(_safety_pin, LOW);
    digitalWrite(_output_pin, LOW);
}

void DisplayController::setTempSetpoint(double setpoint) {
    _setpoint_temp = setpoint;
}

void DisplayController::begin() {
    Serial.begin(115200);
    while (!Serial && (millis() < 4000)) {
    }

    // Initialize the I2C bus
    Wire.begin();
    Serial.println(" - i2c bus initialized");

    // Initialize the LCD screen
    lcd.begin(Wire);
    lcd.setBacklight(RGB_WHITE);
    lcd.setContrast(5);
    lcd.clear();
    Serial.println(" - lcd screen initialized");

    _seal_fault = !_sealTempSensor.begin(_SEAL_TC_ADDR);
    if (_seal_fault) _msg = "Seal TC Sensor Fault!";
    _sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sealTempSensor.setThermocoupleResolution(RES_14_BIT);

    _sump_fault = !_sumpTempSensor.begin(_SUMP_TC_ADDR);
    if (_sump_fault) _msg = "Sump TC Sensor Fault!";
    _sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sumpTempSensor.setThermocoupleResolution(RES_14_BIT);

    _press_fault = _pressSensor.begin();
    if (_press_fault) _msg = "Pressure Sensor Fault!"; 

}

double DisplayController::computeHeaterOutput() {
    _input = getSumpTemp();
    _heaterPIDControl.Compute();
    analogWrite(_output_pin, _output);
    return _output;
}

void DisplayController::update() {
    _seal_temp = _sealTempSensor.getThermocoupleTemp(false);
    _sump_temp = _sumpTempSensor.getThermocoupleTemp(false);
    _pressure = _pressSensor.readPressure();
}

double DisplayController::getSealTemp() {
    return _seal_temp;
}

double DisplayController::getSumpTemp() {
    return _sump_temp;
}

double DisplayController::getPressure() {
    return _pressure;
}