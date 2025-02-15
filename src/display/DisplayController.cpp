#include "DisplayController.h"
#include <avr/wdt.h>

// LCD screen parameters
#define SCREEN_REFRESH_INTERVAL 2000
#define RGB_WHITE 255, 255, 255
#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define RGB_OFF 0, 0, 0
#define MAX_CHARS_PER_LINE 20
#define ERROR_SCREEN_DURATION 5

// Pressure Sensor Parameters
#define PRESS_AVG_TIME 3000

DisplayController::DisplayController() : _heaterPIDControl(&_input, &_output, &_setpoint_temp, _Kp, _Ki, _Kd, DIRECT)
{
    _setpoint_temp = 70.0;
    _heaterPIDControl.SetMode(AUTOMATIC);
    _heaterPIDControl.SetOutputLimits(0, 150);
    _askingForHeat = true;
    Serial.println(" - PID loop initialized");
    _runSwitch = NULL;
    _resetSwitch = NULL;
}



void DisplayController::setTempSetpoint(const double& setpoint,
                                        const double& deltaT_safety,
                                        const bool& units) {
    _deltaT_safety = deltaT_safety;                                        
    _setpoint_temp = setpoint;
    _temp_units = units;
    if (_setpoint_temp > 250.0 && !_temp_units) {
        String err = "setpoint set too high! must be 250F/120C or less";
        errorScreen(err);
    }
    if (_setpoint_temp > 120.0 && _temp_units) {
        String err = "setpoint set too high! must be 250F/120C or less";
        errorScreen(err);
    }
    update(0); // this needs to be verified when test restart or powerloss!
    String msg = "Heater PID control initialized";
    messageScreen(msg);
    writeToLog(msg);
    String input_str_LCD = tempToStrLCD(getSumpTemp(), _temp_units);
    String setpoint_str_LCD = tempToStrLCD(_setpoint_temp, _temp_units);
    String setpoint_str_log = tempToStrLog(_setpoint_temp, _temp_units);
    printFourColumnRow(3, "Sump:", input_str_LCD, " Sp:", setpoint_str_LCD);
    writeToLog("Setpoint: " + setpoint_str_log, "INFO");
    delay(3000);
}

void DisplayController::setRecordInterval(const uint8_t& interval) {
    _record_interval = interval;
}

void DisplayController::setPressureOffset(const uint8_t& num_meas) {
    digitalWrite(_supply_valve_pin, LOW);
    digitalWrite(_dump_valve_pin, HIGH);
    String status, msg = "Determining zero offset for pressure sensor:";
    writeToLog(msg);
    elapsedMillis avgPressTimer, sampleTimer;
    messageScreen(msg);
    uint8_t sample_count = 0;
    _press_offset = 0;
    unsigned int duration = (num_meas + 1) * 1000;
    while (avgPressTimer <= duration) {
        if (sampleTimer > 1000) {
            update(0);
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
    writeToLog("Pressure " + status, "INFO");
    messageScreen(msg, false, status);
    delay(3000);
}

void DisplayController::begin(const std::map<String, uint8_t>& pinMappings) {
    /*
    {"SD_DETECT_PIN", SD_DETECT_PIN},
    {"LOOP_BUS_PIN", LOOP_BUS_PIN},
    {"MOTOR_HLFB_PIN", MOTOR_HLFB_PIN},
    {"HEAT_BUS_PIN", HEAT_BUS_PIN},
    {"HEAT_OUTPUT_PIN", HEAT_OUTPUT_PIN},
    {"HEAT_SAFETY_PIN", HEAT_SAFETY_PIN}, 
    {"AIR_SUPPLY_PIN", AIR_SUPPLY_PIN}, 
    {"AIR_DUMP_PIN", AIR_DUMP_PIN}, 
    {"PRGM_RUN_BUS_PIN", PRGM_RUN_BUS_PIN}, 
    {"TORQ_FLAG_BUS_PIN", TORQ_FLAG_BUS_PIN}, 
    {"AIR_SUPPLY_BUS_PIN", AIR_SUPPLY_BUS_PIN}, 
    {"AIR_DUMP_BUS_PIN", AIR_DUMP_BUS_PIN}, 
    {"RUN_SW_PIN", RUN_SW_PIN},
    {"RESET_SW_PIN", RESET_SW_PIN},
    {"SDA0_PIN", SDA0_PIN}, 
    {"SDL0_PIN", SDL0_PIN},
    {"PRGM_RESET_BUS_PIN", PRGM_RESET_BUS_PIN}
    */   

    // Initialize the teensy board time
    setSyncProvider(_getTeensyTime); 

    _pinMappings = pinMappings;
    _SD_detect_pin = pinMappings.at("SD_DETECT_PIN");
    _supply_bus_pin = pinMappings.at("AIR_SUPPLY_BUS_PIN");
    _dump_bus_pin = pinMappings.at("AIR_DUMP_BUS_PIN");
    _supply_valve_pin = pinMappings.at("AIR_SUPPLY_PIN");
    _dump_valve_pin = pinMappings.at("AIR_DUMP_PIN");
    _torque_pin = pinMappings.at("MOTOR_HLFB_PIN");
    _run_bus_pin = pinMappings.at("PRGM_RUN_BUS_PIN");
    _reset_bus_pin = pinMappings.at("PRGM_RESET_BUS_PIN");
    _loop_bus_pin = pinMappings.at("LOOP_BUS_PIN");
    _run_sw_pin = pinMappings.at("RUN_SW_PIN");
    _reset_sw_pin = pinMappings.at("RESET_SW_PIN");
    _heat_bus_pin = pinMappings.at("HEAT_BUS_PIN");
    _heat_safety_pin = pinMappings.at("HEAT_SAFETY_PIN");
    _heat_output_pin = pinMappings.at("HEAT_OUTPUT_PIN");
    
    pinMode(_SD_detect_pin, INPUT_PULLDOWN);
    pinMode(_supply_bus_pin, INPUT_PULLDOWN);
    pinMode(_dump_bus_pin, INPUT_PULLDOWN);
    pinMode(_supply_valve_pin, OUTPUT);
    pinMode(_dump_valve_pin, OUTPUT);
    pinMode(_torque_pin, INPUT_PULLUP);
    pinMode(_run_bus_pin, OUTPUT);
    pinMode(_reset_bus_pin, OUTPUT);
    pinMode(_loop_bus_pin, INPUT_PULLDOWN);
    pinMode(_run_sw_pin, INPUT_PULLDOWN);
    pinMode(_reset_sw_pin, INPUT_PULLDOWN);
    //pinMode(_heat_bus_pin, INPUT_PULLDOWN);
    pinMode(_heat_safety_pin, OUTPUT);
    pinMode(_heat_output_pin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(_heat_safety_pin, LOW);
    digitalWrite(_heat_output_pin, LOW);
    digitalWrite(_reset_bus_pin, LOW);
    digitalWrite(_run_bus_pin, LOW);
    digitalWrite(_dump_valve_pin, LOW);
    digitalWrite(_supply_valve_pin, LOW);
    digitalWrite(_heat_output_pin, LOW);
    digitalWrite(_heat_safety_pin, LOW);

    String msg ="\nInitializing display controller...";
    Serial.begin(115200);
    while (!Serial && (millis() < 4000)) {
    }
    Serial.println(msg);

    // Initialize the I2C bus
    Wire.begin();
    Serial.println(" - i2c bus initialized");

    // Initialize the LCD screen
    lcd.begin(Wire);
    lcd.setFastBacklight(RGB_WHITE);
    lcd.setContrast(5);
    lcd.clear();
    Serial.println(" - lcd screen initialized");

    // Initialize the SD Card
    _isSDCardInserted = digitalRead(_SD_detect_pin);
    if (_isSDCardInserted) {
        msg = " - SD card initialized";
        _isSDCardActive = SD.begin(BUILTIN_SDCARD);
        delay(100);
        if (!_isSDCardActive) {
            msg = "SD Card initialization fault!";
            errorScreen(msg);
        }
           // Create log file entry header
        msg = "\n\n-----------------------------------------";
        msg += "\nSystem start: " + getDateStr() + ", ";
        msg += getTimeStr() + "\nBegin Log:" + "---\n";
        writeToLog(msg, true);
    }
    else {
        msg = "No SD Card!    Insert and hit RESET switch";
        Serial.println(msg);
        errorScreen(msg);
    }
    
    // Read program file to get test parameters 
    readConfigFile();

    // Initialize seal thermocouple sensor
    msg = "Seal TC sensor initialized";
    _seal_fault = !_sealTempSensor.begin(_SEAL_TC_ADDR);
    if (_seal_fault) {
        msg = "Seal TC Sensor Fault on i2c bus at 0x" + String(_SEAL_TC_ADDR, HEX);
        Serial.println(msg);
        errorScreen(msg);
    }
    _sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sealTempSensor.setThermocoupleResolution(RES_14_BIT);
    writeToLog(msg);
    delay(100);

    // Initialize sump thermocouple sensor
    msg = "Sump TC sensor initialized";
    _sump_fault = !_sumpTempSensor.begin(_SUMP_TC_ADDR);
    if (_sump_fault) {
        msg = "Sump TC Sensor Fault on i2c bus at 0x" + String(_SUMP_TC_ADDR, HEX);
        Serial.println(msg);
        errorScreen(msg);
    } 
    _sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sumpTempSensor.setThermocoupleResolution(RES_14_BIT);
    writeToLog(msg);
    delay(100);
    
    // Initialize pressure sensor
    msg = "Pressure sensor initialized";
    _press_fault = !_pressSensor.begin();
    if (_press_fault) {
        msg = "Pressure Sensor Fault on i2c bus!";
        Serial.println(msg);
        errorScreen(msg);
    }
    writeToLog(msg);
    delay(100);

    // Initialize the Run / Reset Switch
    msg = "Run / Reset switch initialized";
    if (_runSwitch == NULL) {
        _runSwitch = new Bounce();
    }
    if (_resetSwitch == NULL) {
        _resetSwitch = new Bounce();
    }
    _runSwitch->attach(_run_sw_pin);
    _runSwitch->interval(10);
    _resetSwitch->attach(_reset_sw_pin);
    _resetSwitch->interval(10);
    writeToLog(msg);
    }




void DisplayController::update(const uint32_t& loop_count) {
    if (loop_count != _current_loop_count) {
        _current_loop_count = loop_count;
        writeToConfigFile();
    }
    _seal_temp = _sealTempSensor.getThermocoupleTemp(_temp_units);
    _sump_temp = _sumpTempSensor.getThermocoupleTemp(_temp_units);
    _abs_pressure = _pressSensor.readPressure();
    _rel_pressure = _abs_pressure - _press_offset;
    _runSwitch->update();
    _resetSwitch->update();
    _askingForHeat = digitalRead(_heat_bus_pin) ? HIGH : LOW;
    bool askingForPressure = digitalRead(_supply_bus_pin) ? HIGH : LOW;
    digitalWrite(_supply_valve_pin, askingForPressure);
    bool dump_state = digitalRead(_dump_bus_pin) ? HIGH : LOW;
    digitalWrite(_dump_valve_pin, dump_state);
    _isSDCardInserted = digitalRead(_SD_detect_pin);
    if (!_isSDCardInserted) {
        _isSDCardActive = false;
    }
}

void DisplayController::runProgram() {
    digitalWrite(_run_bus_pin, HIGH);
}

void DisplayController::resetTest() {
    _PIDTimer = 0;
    readConfigFile();
    _current_loop_count = 0;
    writeToConfigFile();
    writeToLog("RESET", "STATUS");
    lcd.clear();
    digitalWrite(_reset_bus_pin, HIGH);
    delay(10);
    digitalWrite(_reset_bus_pin, LOW);
    _hasHeaderBeenWritten = false;
    //writeToDataFile();
    setPressureOffset();
}

void DisplayController::stopProgram() {
    digitalWrite(_run_bus_pin, LOW);
}

void DisplayController::resetProgram() {
    digitalWrite(_reset_bus_pin, HIGH);
}

uint32_t DisplayController::getRequestedNumberOfLoops() {
    return _requested_loops;
}

uint32_t DisplayController::getCurrentLoopCount() {
    return _current_loop_count;
}




void DisplayController::computeHeaterOutput(const unsigned int& interval) {
    Serial.print("Setpoint temp: ");
    Serial.print(_setpoint_temp);
    Serial.print(", _askingForHeat: ");
    Serial.println(_askingForHeat);
    if (_setpoint_temp == 0 || !_askingForHeat) {
        turnOffHeaters();
    } 
    else {
        armHeaters();
        double deltaT = _sump_temp - _setpoint_temp;
        
        if (_PIDTimer > 3600000 && deltaT < -5) {
            errorScreen("Sump temp not rising, check heaters");
        }
        if (deltaT > _deltaT_safety) {
            errorScreen("Thermal runaway!");
        }
        if (_PIDTimer > interval) {
            _input = _sump_temp;
            _heaterPIDControl.Compute();
            analogWrite(_heat_output_pin, _output);
            _PIDTimer = 0;
        }
    }
}

void DisplayController::turnOffHeaters() {
    if (_areHeatersArmed) {
        digitalWrite(_heat_output_pin, LOW);
        digitalWrite(_heat_safety_pin, LOW);
        _areHeatersArmed = false;
    } 
}

void DisplayController::armHeaters() {
    if (!_areHeatersArmed) {
        _areHeatersArmed = true;
        digitalWrite(_heat_safety_pin, HIGH);
    }
}

double DisplayController::getSetpointTemp() {
    return _setpoint_temp;
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
    Serial.println("torque measured!");
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




void DisplayController::writeToLog(const String& msg, const String& type,
                                   const bool& first) {
    String log, error = "No SD Card! Can't write to log file";
    _isSDCardInserted = digitalRead(_SD_detect_pin);

    if (_isSDCardInserted && !_isSDCardActive) {
        _isSDCardActive = SD.begin(BUILTIN_SDCARD);
    }
    if (!_isSDCardInserted && Serial) {
        errorScreen(msg, 5);
        Serial.println(" - [INTENDED FOR LOGFILE] " + msg);
    }
    else if (_isSDCardActive) {
        _logFile = SD.open("log.txt", FILE_WRITE);
        if (_logFile) {
            if (first) log = msg;
            log = getDateStr() + " " + getTimeStr();
            log += " [" + type + "]\t" + msg + "\n";
            _logFile.print(log);
            _logFile.close();
            Serial.println(" - " + msg);
        }
        else {
            errorScreen(msg, 5);
        }
    }
}

void DisplayController::writeToDataFile() {
    if (_dataLoggerTimer > (_record_interval*1000)) {
        _isSDCardInserted = digitalRead(_SD_detect_pin);
        String error = "No SD Card! Can't write to data file";
        String unit = "F";
        if (_temp_units) unit = "C";
        if (_isSDCardInserted && !_isSDCardActive) {
            _isSDCardActive = SD.begin(BUILTIN_SDCARD);
        }
        if (_isSDCardActive) {
            _dataFile = SD.open("test_data.csv", FILE_WRITE);
            if (_dataFile) {
                if (!_hasHeaderBeenWritten) {
                    String header_str = "Datetime,Loop,Press(psi),";
                    header_str += "SealTemp(\xB0" + unit + "),SumpTemp(\xB0";
                    header_str += unit + "),CW_Torque(Nm),CCW_Torque(Nm)";
                    _dataFile.println(header_str);
                    writeToLog("data logging to test_data.csv started", "LOG");
                    _hasHeaderBeenWritten = true;
                }
                String data_str = getDateStr() + " " + getTimeStr() + ",";
                data_str += String(_current_loop_count) + "," + String(_rel_pressure);
                data_str += "," + String(_seal_temp) + "," + String(_sump_temp);
                data_str += "," + String(_cw_torque) + "," + String(_ccw_torque);
                _dataFile.println(data_str);
                _dataFile.close();
                _dataLoggerTimer = 0;
                
            }
            else {
                errorScreen(error, 5);
                _dataLoggerTimer = 0;
            } 
        }
        else {
            errorScreen(error, 5);
            _dataLoggerTimer = 0;
        }
    } 
}

void DisplayController::writeToConfigFile() {
    _isSDCardInserted = digitalRead(_SD_detect_pin);
    String error = "No SD Card! Can't write to config file";
    if (_isSDCardInserted && !_isSDCardActive) {
        _isSDCardActive = SD.begin(BUILTIN_SDCARD);
    }
    if (_isSDCardActive) {
        _restartFile = SD.open("config_file.txt", FILE_READ);
        String fileContent = "";
        int lastColonPosition = -1; // Initialize the position to -1 (not found)
        int currentPosition = 0;

        while (_restartFile.available()) {
            char c = _restartFile.read();
            if (c == ':') {
                lastColonPosition = currentPosition;
            }
            fileContent += c;
            currentPosition++;
        }
        _restartFile.close();

        // fileContent contains the entire content of the file
        // lastColonPosition holds the index of the last colon found.
        if (lastColonPosition != -1) {
            fileContent = fileContent.substring(0, lastColonPosition);
        }
        else {
            error = "config_file.txt incorrectly formatted or does not exist!";
            error += "recreate config file and restart test";
            errorScreen(error);
        }

        SD.remove("config_file.txt");
        _restartFile = SD.open("config_file.txt", FILE_WRITE);
        if (_restartFile) {
            _restartFile.print(fileContent);
            _restartFile.print(": ");
            _restartFile.println(_current_loop_count);
            _restartFile.close();
        } else {
            error = "Error writing to config_file.txt";
            errorScreen(error);
        }
    } else {
        errorScreen(error);
    }
}

void DisplayController::readConfigFile() {
    _isSDCardInserted = digitalRead(_SD_detect_pin);
    String error = "No SD Card! Can't read config file";
    if (_isSDCardInserted && !_isSDCardActive) {
        _isSDCardActive = SD.begin(BUILTIN_SDCARD);
    }
    if (_isSDCardActive) {
        _restartFile = SD.open("config_file.txt", FILE_READ);
        if (_restartFile) {
            // Read the lines and update the variables
            while (_restartFile.available()) {
                String line = _restartFile.readStringUntil('\n');
                line.trim(); // Remove leading/trailing whitespaces

                // Parse the line based on the delimiter ':'
                int delimiterIndex = line.indexOf(':');
                if (delimiterIndex >= 0) {
                    String key = line.substring(0, delimiterIndex);
                    String value = line.substring(delimiterIndex + 1);
                    value.trim(); // Remove leading/trailing whitespaces
                    
                    // Update the corresponding variable based on the key
                    if (key == "Required number of loops") {
                        _requested_loops = value.toInt();
                    }
                    else if (key == "Record data at interval(secs) of") {
                        _record_interval = value.toInt();
                    }
                    else if (key == "Temperature setpoint") {
                        _setpoint_temp = value.toFloat();
                    }
                    else if (key == "deltaT safety") {
                        _deltaT_safety = value.toFloat();
                    }
                    else if (key == "Temp units (1 -> degC, 0 -> degF)") {
                        _temp_units = value.toInt();
                    }
                    else if (key == "Test currently on loop") {
                        _current_loop_count = value.toInt();
                    }
                }
            }
            _restartFile.close();
        }
        else {
            error = "Error reading config_file.txt file";
            errorScreen(error);
        }
    }
    else {
        errorScreen(error);
    }
}




void DisplayController::updateLCD(const String& test_status) {
    String loops_str = _current_loop_count;
    String seal_temp_str = tempToStrLCD(_seal_temp, _temp_units);
    String sump_temp_str = tempToStrLCD(_sump_temp, _temp_units);
    String pressure = String(_rel_pressure);

    /**** DEBUGGING ONLY!!! ****/
    //pressure = "12.3";
    // double _cw_torque = 1.26;
    // double _ccw_torque = -0.98;
    /***************************/

    String torques_str = String(_ccw_torque) + " / " + String(_cw_torque);
    String set_point_str = tempToStrLCD(_setpoint_temp, _temp_units);
    String date = String(month()) + "/" + String(day());
    String time = String(hour()) + ":" + String(minute(), 2);
    if (_screenTimer > SCREEN_REFRESH_INTERVAL) {
        _screenTimer = 0;
        if (_isScreenUpdate) {
            _isScreenUpdate = !_isScreenUpdate;
            printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status);
            printFourColumnRow(1, "P:", pressure +"psi", " Loop:", loops_str);
            printRowPair(0, 2, MAX_CHARS_PER_LINE, "Setpoint:", set_point_str);
            printFourColumnRow(3, "Seal:", seal_temp_str, " Sump:", sump_temp_str); 
        }
        else {
            _isScreenUpdate = !_isScreenUpdate;
            printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status);
            printFourColumnRow(1, "P:", pressure +"psi", " Loop:", loops_str);
            printRowPair(0, 2, MAX_CHARS_PER_LINE, "Torque:", torques_str);
            printFourColumnRow(3, "Seal:", seal_temp_str, " Sump:", sump_temp_str);
            }
    }
}

void DisplayController::statusUpdate(const String& test_status) {
    printRowPair(0, 0, MAX_CHARS_PER_LINE, "Status:", test_status);
}

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

void DisplayController::testCompleted(const String& test_status_str) {
     if (!_hasTestCompletedBeenCalled) {
        turnOffHeaters();
        stopProgram(); // don't remove this!
        _hasTestCompletedBeenCalled = true;
        _screenTimer = 0;
     }
    Serial.println(digitalRead(_run_bus_pin)); // don't remove this!
    
    if (_completeTimer >= 1500) {
        if (_flasher) {
            lcd.setFastBacklight(RGB_GREEN);
        }
        else {
            lcd.setFastBacklight(RGB_WHITE);
        }
        
        updateLCD(test_status_str);
        _flasher = !_flasher;
        _completeTimer = 0;
    }
}

void DisplayController::errorScreen(const String& msg, const int& time) {
    messageScreen(msg);
    delay(1000);
    if (_isSDCardActive && _isSDCardInserted) {
        writeToLog(msg, "ERROR");
    }
    if (time==0) {
        turnOffHeaters();
        stopProgram();
        while (true) {
            if (_errorTimer >= 1000) {
                
                if (_flasher) {
                    lcd.setFastBacklight(RGB_RED);
                }
                else {
                    lcd.setFastBacklight(RGB_OFF);
                }
                update(_current_loop_count);
                _flasher = !_flasher;
                _errorTimer = 0;
            }
            else if (getResetSwitch()) {
                begin(_pinMappings);
                break;
            }
        }
    }
    else {
        for (int x=time; x>0; x--) {
        if (_errorTimer >= 1000) {
            if (_flasher) {
                lcd.setFastBacklight(RGB_RED);
            }
            else {
                lcd.setFastBacklight(RGB_WHITE);
            }
            _flasher = !_flasher;
            _errorTimer = 0;
            delay(10);
            messageScreen(msg, false, "continue in " + String(x) + "s");
        }
        else if (getResetSwitch()) {
                begin(_pinMappings);
                break;
            }
        delay(1000);
    }
    lcd.setFastBacklight(RGB_WHITE);
    }
}

void DisplayController::resetScreen() {
    lcd.clear();
    lcd.setFastBacklight(RGB_WHITE);
}



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

String DisplayController::tempToStrLCD(const double& temp,
                                    const bool& unit) {
    if (temp > 99) {
        if (unit) {
            return String(temp, 0) + "C";
        }
        else return String(temp, 0) + "F";
    }
    else {
        if (unit) {
            return String(temp, 0) + "\xDF" + "C";
        }
        else return String(temp, 0) + "\xDF" + "F";
    }
}

String DisplayController::tempToStrLog(const double& temp,
                                    const bool& unit) {
  if (unit) {
    return String(temp, 0) + "\xB0" + "C";
  }
  else return String(temp, 0) + "\xB0" + "F";
}

String DisplayController::getTimeStr() {
    char timeBuffer[9];
    sprintf(timeBuffer, "%02d:%02d:%02d", hour(), minute(), second());
    return String(timeBuffer);
}

String DisplayController::getDateStr() {
    char dateBuffer[11];
    sprintf(dateBuffer, "%02d/%02d/%04d", month(), day(), year());
    return String(dateBuffer);
}

time_t DisplayController::_getTeensyTime() {
  return Teensy3Clock.get();
}
