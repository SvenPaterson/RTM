#include "DisplayController.h"

// LCD screen parameters
#define SCREEN_REFRESH_INTERVAL 2000
#define RGB_WHITE 255, 255, 255
#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define MAX_CHARS_PER_LINE 20
#define ERROR_SCREEN_DURATION 5

// Pressure Sensor Parameters
#define PRESS_AVG_TIME 3000

DisplayController::DisplayController() : _heaterPIDControl(&_input, &_output, &_setpoint_temp, _Kp, _Ki, _Kd, DIRECT)
{
    _setpoint_temp = 70.0;
    _heaterPIDControl.SetMode(AUTOMATIC);
    _heaterPIDControl.SetOutputLimits(0, 150);
    Serial.println(" - PID loop initialized");
    _runSwitch = NULL;
    _resetSwitch = NULL;
}

void DisplayController::setHeaterPins(const uint8_t& safety_pin,
                                      const uint8_t& output_pin) {
    _safety_pin = safety_pin;
    _output_pin = output_pin;
    pinMode(_safety_pin, OUTPUT);
    pinMode(_output_pin, OUTPUT);
    digitalWrite(_safety_pin, LOW);
    digitalWrite(_output_pin, LOW);
}

void DisplayController::setSwitchPins(const uint8_t& run_pin,
                                      const uint8_t& reset_pin) {
    _run_pin = run_pin;
    _reset_pin = reset_pin;
    pinMode(_run_pin, INPUT_PULLDOWN);
    pinMode(_reset_pin, INPUT_PULLDOWN);
}

void DisplayController::setTestBusPins(const uint8_t& run_bus_pin,
                                       const uint8_t& reset_bus_pin,
                                       const uint8_t& loop_bus_pin) {
    _run_bus_pin = run_bus_pin;
    _reset_bus_pin = reset_bus_pin;
    _loop_bus_pin = loop_bus_pin;

    pinMode(_run_bus_pin, OUTPUT);
    pinMode(_reset_bus_pin, OUTPUT);
    pinMode(_loop_bus_pin, INPUT_PULLDOWN);

}

void DisplayController::setTorquePins(const uint8_t& torque_pin,
                                     const uint8_t& torque_bus_pin) {
    _read_torque_bus_pin = torque_bus_pin;
    _torque_pin = torque_pin;

    pinMode(_torque_pin, INPUT_PULLUP);
    pinMode(_read_torque_bus_pin, INPUT_PULLDOWN);
}

void DisplayController::setAirPins(const uint8_t& supply_bus_pin,
                                   const uint8_t& dump_bus_pin,
                                   const uint8_t& supply_valve_pin,
                                   const uint8_t& dump_valve_pin) {
    _supply_bus_pin = supply_bus_pin;
    _dump_bus_pin = dump_bus_pin;
    _supply_valve_pin = supply_valve_pin;
    _dump_valve_pin = dump_valve_pin;

    pinMode(_supply_bus_pin, INPUT_PULLDOWN);
    pinMode(_dump_bus_pin, INPUT_PULLDOWN);
    pinMode(_supply_valve_pin, OUTPUT);
    pinMode(_dump_valve_pin, OUTPUT);
}

void DisplayController::setTempSetpoint(const double& setpoint,
                                        const bool& units) {
    _setpoint_temp = setpoint;
    _temp_units = units;
    update();
    String msg = "Heater PID control initialized";
    messageScreen(msg);
    String input_str = tempToStr(getSumpTemp(), _temp_units);
    String setpoint_str = tempToStr(_setpoint_temp, _temp_units);
    printFourColumnRow(3, "Sump:", input_str, " Sp:", setpoint_str);
    Serial.println("- " + msg);
    delay(3000);
}

void DisplayController::begin() {
    String msg;
    Serial.begin(115200);
    while (!Serial && (millis() < 4000)) {
    }
    Serial.println("\nInitializing display controller...");

    // Initialize the I2C bus
    Wire.begin();
    Serial.println(" - i2c bus initialized");

    // Initialize the LCD screen
    lcd.begin(Wire);
    lcd.setBacklight(RGB_WHITE);
    lcd.setContrast(5);
    lcd.clear();
    Serial.println(" - lcd screen initialized");

    // Initialize seal thermocouple sensor
    msg = " - Seal TC sensor initialized";
    _seal_fault = !_sealTempSensor.begin(_SEAL_TC_ADDR);
    if (_seal_fault) {
        msg = "Seal TC Sensor Fault!";
        Serial.println(msg);
        errorScreen(msg, 5);
    }
    _sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sealTempSensor.setThermocoupleResolution(RES_14_BIT);
    Serial.println(msg);
    delay(100);

    // Initialize sump thermocouple sensor
    msg = " - Sump TC sensor initialized";
    _sump_fault = !_sumpTempSensor.begin(_SUMP_TC_ADDR);
    if (_sump_fault) {
        msg = "Sump TC Sensor Fault!";
        Serial.println(msg);
        errorScreen(msg);
    } 
    _sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sumpTempSensor.setThermocoupleResolution(RES_14_BIT);
    Serial.println(msg);
    delay(100);
    
    // Initialize pressure sensor
    msg = " - Pressure sensor initialized";
    _press_fault = !_pressSensor.begin();
    if (_press_fault) {
        msg = "Pressure Sensor Fault!";
        Serial.println(msg);
        errorScreen(msg);
    }
    Serial.println(msg);
    delay(100);

    // Initialize the Run / Reset Switch
    if (_runSwitch == NULL) {
        _runSwitch = new Bounce();
    }
    if (_resetSwitch == NULL) {
        _resetSwitch = new Bounce();
    }
    _runSwitch->attach(_run_pin);
    _runSwitch->interval(100);
    _resetSwitch->attach(_reset_pin);
    _resetSwitch->interval(100);
}

void DisplayController::computeHeaterOutput(const unsigned int& interval) {
    if (_PIDTimer > interval) {
        _input = _sump_temp;
        _heaterPIDControl.Compute();
        analogWrite(_output_pin, _output);
        _PIDTimer = 0;
    }
}

void DisplayController::turnOffHeaters() {
    analogWrite(_output_pin, 0);
    digitalWrite(_safety_pin, LOW);
}

void DisplayController::armHeaters() {
    digitalWrite(_safety_pin, HIGH);
}

void DisplayController::update() {
    _seal_temp = _sealTempSensor.getThermocoupleTemp(_temp_units);
    _sump_temp = _sumpTempSensor.getThermocoupleTemp(_temp_units);
    _abs_pressure = _pressSensor.readPressure();
    _rel_pressure = _abs_pressure - _press_offset;
    _runSwitch->update();
    _resetSwitch->update();
    bool sup_state = digitalRead(_supply_bus_pin) ? HIGH : LOW;
    digitalWrite(_supply_valve_pin, sup_state);
    bool dump_state = digitalRead(_dump_bus_pin) ? HIGH : LOW;
    digitalWrite(_dump_valve_pin, dump_state);
}

void DisplayController::setPressureOffset(const uint8_t& num_meas) {

    // NEED TO IMPLEMENT PRESSURE OPEN TO ATMOS FOR THIS //
    String status, msg = "Determining zero offset for pressure sensor:";
    Serial.println("- " + msg);
    elapsedMillis avgPressTimer, sampleTimer;
    messageScreen(msg);
    uint8_t sample_count = 0;
    unsigned int duration = (num_meas + 1) * 1000;
    while (avgPressTimer <= duration) {
        if (sampleTimer > 1000) {
            update();
            _press_offset += _abs_pressure;
            sample_count++;  
            sampleTimer = 0;
            status = "please wait... " + String(sample_count) + "s";
            Serial.println("    " + status);
            messageScreen(msg, false, status);
        }
    }
    _press_offset /= sample_count;
    status = "offset by " + String(_press_offset) + " psi";
    Serial.println("Pressure " + status);
    messageScreen(msg, false, status);
}

void DisplayController::readTorque() {
    uint32_t pwm_high_val = pulseIn(_torque_pin, HIGH, 100000);
    uint32_t pwm_low_val = pulseIn(_torque_pin, LOW, 100000);
    double pwm_duty_value = pwm_high_val / (pwm_high_val + pwm_low_val) * 100;

    if (pwm_duty_value < 96 && pwm_duty_value > 4) {
        double torque = ((pwm_duty_value - 50) * .3636);
        if (torque > 0) {
            _cw_torque = torque;
        }
        else {
            _ccw_torque = torque;
        }
    }
}

void DisplayController::runProgram() {
    digitalWrite(_run_bus_pin, HIGH);
}

void DisplayController::stopProgram() {
    digitalWrite(_run_bus_pin, LOW);
}

void DisplayController::resetProgram() {
    digitalWrite(_reset_bus_pin, HIGH);
}

double DisplayController::getSealTemp() {
    return _seal_temp;
}

double DisplayController::getSumpTemp() {
    return _sump_temp;
}

double DisplayController::getPressure() {
    return _abs_pressure;
}

bool DisplayController::getRunSwitch() {
    return _runSwitch->read();
}

bool DisplayController::getResetSwitch() {
    return _resetSwitch->read();
}

/////////////// IMPLEMENT SD CARD FUNCTIONALITY HERE!!! /////////////

/****** LCD SCREEN FUNCTION DEFINITIONS ******/

void DisplayController::messageScreen(const String& msg,
                                      const bool& cls,
                                      const String& status) {
    if (cls) {
        lcd.clear();
    }

    // Print the message with line wrapping
    int startPos = 0;
    int endPos = 0;
    int numLines = 0;
    int messageLength = msg.length();

    while (startPos < messageLength) {
        // Calculate the end position for the line
        endPos = startPos + MAX_CHARS_PER_LINE - 1;

        // Check if the end position goes beyond the message length
        if (endPos >= messageLength) {
        endPos = messageLength - 1;
        }
        else {
        // Find the last space character within the line's range
        while (endPos > startPos && msg[endPos] != ' ') {
            endPos--;
        }
        // adjust end pos to max limit if no space found
        if (endPos == startPos) {
            endPos = startPos + MAX_CHARS_PER_LINE - 1;
        }
        }

        lcd.setCursor(0, numLines);
        lcd.print(msg.substring(startPos, endPos + 1));
        numLines++;
        startPos = endPos + 1;
        }

        lcd.setCursor(0, 3);
        if (status != "") {
            lcd.print(rightJustifiedString(status));
        }
}

void DisplayController::testDoneScreen(const uint8_t& loop_count,
                                       const uint8_t total_loops) {
    //String status = "Total hours ran: " + String(_run_hours);
    if (_completeTimer >= 10000) {
        lcd.clear();
    }
    if (_completeTimer >= 1000) {
        messageScreen("[TEST COMPLETED]", true);
        if (_flasher) {
            lcd.setBacklight(RGB_GREEN);
        }
        else {
            lcd.setBacklight(RGB_WHITE);
        }
        _flasher = !_flasher;
        _completeTimer = 0;
    }
}

void DisplayController::errorScreen(const String& msg, const int& time) {
    messageScreen(msg);
    if (time==0) {
        while (true) {
            if (_errorTimer >= 1000) {
                if (_flasher) {
                    lcd.setBacklight(RGB_RED);
                }
                else {
                    lcd.setBacklight(RGB_WHITE);
                }
                _flasher = !_flasher;
                _errorTimer = 0;
            }
        }
    }
    else {
        for (int x=time; x>0; x--) {
        if (_errorTimer >= 1000) {
            if (_flasher) {
                lcd.setBacklight(RGB_RED);
            }
            else {
                lcd.setBacklight(RGB_WHITE);
            }
            _flasher = !_flasher;
            _errorTimer = 0;
            messageScreen(msg, false, "continue in " + String(x) + "s");
        }
        delay(1000);
    }
    lcd.setBacklight(RGB_WHITE);
    }
}

//unsigned long DisplayController::

String DisplayController::rightJustifiedString(const String& str) {
    /* Function returns a String with enough padding
    * prepended for the LCD screen to right justify the text
    * on an empty line
    */
    String paddedStr = str;
    while (paddedStr.length() < MAX_CHARS_PER_LINE) {
    paddedStr = " " + paddedStr;
    }
    return paddedStr;
}

void DisplayController::printFourColumnRow(const int& row, const String& str1,
                                           const String& str2, const String& str3, 
                                           const String& str4) {
    int width = MAX_CHARS_PER_LINE / 2;
    printRowPair(0, row, width, str1, str2);
    printRowPair(width, row, width, str3, str4);
}

void DisplayController::printRowPair(const int& col, const int& row,
                                     const int& width, const String& str1,
                                     const String& str2) {
    lcd.setCursor(col, row);
    lcd.print(str1);
    lcd.setCursor(str1.length()+col, row);
    lcd.print(padBetweenChars(width, str1, str2));
}

String DisplayController::padBetweenChars(const int& num_chars, 
                                          const String& str1,
                                          const String& str2) {
    unsigned int space = num_chars - (str1).length();
    String paddedStr = str2;
    while (paddedStr.length() < space) {
    paddedStr = " " + paddedStr;
    }
    return paddedStr;
}

String DisplayController::tempToStr(const double& temp,
                                    const bool& unit) {
  if (unit) {
    return String(temp, 0) + String((char)223) + "C";
  }
  else return String(temp, 0) + String((char)223) + "F";
}

void DisplayController::updateLCD(const String& test_status_str,
                                  const unsigned int& loop_count) {
    String loops_str = loop_count;
    String seal_temp_str = tempToStr(_seal_temp, _temp_units);
    String sump_temp_str = tempToStr(_sump_temp, _temp_units);
    String pressure = String(_rel_pressure);

    /**** DEBUGGING ONLY!!! ****/
    //pressure = "12.3";
    double _cw_torque = 1.26;
    double _ccw_torque = -0.98;
    /***************************/

    String torques_str = String(_ccw_torque) + " / " + String(_cw_torque);
    String set_point_str = tempToStr(_setpoint_temp, _temp_units);
    String date = String(month()) + "/" + String(day());
    String time = String(hour()) + ":" + String(minute());
    
    if (_screenTimer > SCREEN_REFRESH_INTERVAL) {
        _screenTimer = 0;
        if (_isScreenUpdate) {
            _isScreenUpdate = !_isScreenUpdate;
            printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status_str);
            printRowPair(0, 1, MAX_CHARS_PER_LINE, "Setpoint:", set_point_str);
            printFourColumnRow(2, "P:", pressure +"psi", " Loop:", loops_str);
            printFourColumnRow(3, "Seal:", seal_temp_str, " Sump:", sump_temp_str);
        }
        else {
            _isScreenUpdate = !_isScreenUpdate;
            // printRowPair(0, 0, MAX_CHARS_PER_LINE, "Date:", dateTimeStr());
            // print the remaining test time here
            printRowPair(0, 1, MAX_CHARS_PER_LINE, "Torque:", torques_str);
        }
    }
}