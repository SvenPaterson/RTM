#include "DisplayController.h"

// LCD screen parameters
#define RGB_WHITE 255, 255, 255
#define RGB_RED 255, 0, 0
#define MAX_CHARS_PER_LINE 20
#define ERROR_SCREEN_DURATION 10

// Pressure Sensor Parameters
#define PRESS_AVG_TIME 3000

DisplayController::DisplayController() : _heaterPIDControl(&_input, &_output, &_setpoint_temp, _Kp, _Ki, _Kd, DIRECT)
{
    _setpoint_temp = 70.0;
    _heaterPIDControl.SetMode(AUTOMATIC);
    _heaterPIDControl.SetOutputLimits(0, 150);
    _runSwitch = NULL;
    _resetSwitch = NULL;
}

void DisplayController::setHeaterPins(const byte& safety_pin, const byte& output_pin) {
    _safety_pin = safety_pin;
    _output_pin = output_pin;
    pinMode(_safety_pin, OUTPUT);
    pinMode(_output_pin, OUTPUT);
    digitalWrite(_safety_pin, LOW);
    digitalWrite(_output_pin, LOW);
}

void DisplayController::setSwitchPins(const byte& run_pin, const byte& reset_pin) {
    _run_pin = run_pin;
    _reset_pin = reset_pin;
    pinMode(_run_pin, INPUT_PULLDOWN);
    pinMode(_reset_pin, INPUT_PULLDOWN);
}

void DisplayController::setBusPins(const byte& run_bus_pin, const byte& reset_bus_pin,
                                   const byte& read_torque_bus_pin, const byte& loop_bus_pin) {
    _run_bus_pin = run_bus_pin;
    _reset_bus_pin = reset_bus_pin;
    _read_torque_bus_pin = read_torque_bus_pin;
    _loop_bus_pin = loop_bus_pin;

    pinMode(_run_bus_pin, OUTPUT);
    pinMode(_reset_bus_pin, OUTPUT);
    pinMode(_loop_bus_pin, INPUT_PULLDOWN);
    pinMode(_read_torque_bus_pin, INPUT_PULLDOWN);
}

void 
void DisplayController::setTempSetpoint(double setpoint) {
    _setpoint_temp = setpoint;
}

void DisplayController::begin() {
    Serial.begin(115200);
    Serial.println("\nInitializing display controller...");
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

    // Initialize pressure sensor
    _press_fault = _pressSensor.begin();
    if (_press_fault) {
        _msg = "Pressure Sensor Fault!";
        errorScreen(_msg);
    }

    // Initialize seal thermocouple sensor
    _seal_fault = !_sealTempSensor.begin(_SEAL_TC_ADDR);
    if (_seal_fault) _msg = "Seal TC Sensor Fault!";
    _sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sealTempSensor.setThermocoupleResolution(RES_14_BIT);
    Serial.println(" - Seal TC sensor initialized");
    delay(100);

    // Initialize sump thermocouple sensor
    _sump_fault = !_sumpTempSensor.begin(_SUMP_TC_ADDR);
    if (_sump_fault) _msg = "Sump TC Sensor Fault!";
    _sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sumpTempSensor.setThermocoupleResolution(RES_14_BIT);
    Serial.println(" - Sump TC sensor initialized");
    delay(100);

    Serial.println(_msg);

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

double DisplayController::computeHeaterOutput() {
    _input = getSumpTemp();
    _heaterPIDControl.Compute();
    analogWrite(_output_pin, _output);
    Serial.print("heater pin output: ");
    Serial.println(_output);
    return _output;
}

void DisplayController::turnOffHeaters() {
    analogWrite(_output_pin, 0);
    digitalWrite(_safety_pin, LOW);
}

void DisplayController::armHeaters() {
    digitalWrite(_safety_pin, HIGH);
}

void DisplayController::update() {
    _seal_temp = _sealTempSensor.getThermocoupleTemp(false);
    _sump_temp = _sumpTempSensor.getThermocoupleTemp(false);
    _pressure = _pressSensor.readPressure();
    _runSwitch->update();
    _resetSwitch->update();
}

void DisplayController::setPressureOffset() {
    String status, msg = "Determining zero offset for pressure sensor:";
    elapsedMillis avgPressTimer, sampleTimer;
    messageScreen(msg);
    int sample_count = 0;
    while (avgPressTimer <= PRESS_AVG_TIME) {
        if (sampleTimer > 1000) {
            update();
            _press_offset += _pressure;
            sample_count++;  
            sampleTimer = 0;
            status = "please wait... " + String(sample_count) + "s";
            messageScreen(msg, false, status);
        }
    }
    _press_offset /= sample_count;
    Serial.println(" - pressure sensor initialized");
    status = "offset by " + String(_press_offset) + " psi";
    messageScreen(msg, false, status);
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

bool DisplayController::getRunSwitch() {
    return _runSwitch->read();
}

bool DisplayController::getResetSwitch() {
    return _resetSwitch->read();
}

/****** LCD SCREEN FUNCTION DEFINITIONS ******/
void DisplayController::messageScreen(const String& msg, const bool& cls, const String& status) {
    /* Function takes both a main message and a status update and displays the main
   * message on the LCD screen, left justified with text wrapping. The status
   * message is displayed in the lower right corner and is right justified.
   * 
   * The function can be called to clear the screen (default) prior to displaying 
   * message. Setting this to false helps prevent flicker during multiple calls
   * of function while updating only the status_msg.
  */

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

void DisplayController::errorScreen(const String& msg) {
    /* Function displays an error message on the LCD
    * and flashes the backlight red. It contains an 
    * infinite while loop so the only way to clear
    * the error is the restart the controller 
    */
    bool flasher = true;
    elapsedMillis _errorTimer;
    messageScreen(msg);
    for (int x=ERROR_SCREEN_DURATION; x>0; x--) {
        //update();
        if (_errorTimer >= 1000) {
            if (flasher) {
                lcd.setBacklight(RGB_RED);
            }
            else lcd.setBacklight(RGB_WHITE);
            flasher = !flasher;
            _errorTimer = 0;
            messageScreen(msg, false, "continue in " + String(x) + "s");
            /* if (getResetSwitch()) {
                x = 0;
            } */
        }
        delay(1000);
        
    }
    lcd.setBacklight(RGB_WHITE);
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