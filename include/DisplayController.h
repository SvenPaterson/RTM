#ifndef DC_h
#define DC_h

#include <Arduino.h>
#include <map>
#include <SparkFun_MCP9600.h>
#include <SparkFun_MicroPressure.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include <TimeLib.h>
#include <SD.h>
#include <SPI.h>
#include <cmath>
#include <avr/wdt.h>
#include <map>

#define MCP9600_DEFAULT_ADDR_ONE 0x60
#define MCP9600_DEFAULT_ADDR_TWO 0x67

class DisplayController {
    public:
        DisplayController();

        /**  
         * Assign pins, initialize devices and handle any errors. 
         */
        void begin(const std::map<String, uint8_t>& pinMappings);

        /**  
         * Set the heater control setpoint temperature and display message
         * stating that heater control has been initialized.
         * @param setpoint temperature setpoint
         * @param deltaT_safety the allowed delta above setpoint 
         * @param units false for farenheit, true for celcius
         */
        void setTempSetpoint(const double& setpoint, const double& deltaT_safety=25,
                             const bool& units=false);
        
        /**  
         * Take a number of pressure measurements at 1sec intervals with
         * rig head open to atmosphere and averaged them. Use this value 
         * to set the pressure offset.
         * @param num_meas default: 10 measurements
         */
        void setPressureOffset(const uint8_t& num_meas=10);

        /**  
         * Take measurements from all attached i/o, except torque and
         * update variables. Will also read air supply and dump states
         * from motor controller and set corresponding valve outputs.
         */
        void update(const uint32_t& loop_count);

        /**  
         * Take a measurement from the motor high level feedback pin
         * and calculate torque value based on motor direction then
         * updates variables.
         */
        void readTorque();

        /**
         * @brief Send signal to motor controller to run test loop
         */
        void runProgram();

        /**
         * @brief Send signal to motor controller to stop test loop
         */
        void stopProgram();

        /**
         * @brief Send signal to motor controller to reset test loop
         */
        void resetProgram();

        /**  
         * @return returns value of _seal_temp
         */
        double getSealTemp();

        /**  
         * @return returns value of _sump_temp
         */
        double getSumpTemp();

        /**  
         * @brief Returns relative pressure.
         * @return pressure, in psi
         */
        double getPressure();

        /**  
         * @brief Reads the run switch
         * @return returns true for on, false for off
         */
        bool getRunSwitch();

        /**  
         * @brief Reads the reset switch
         * @return returns true for on, false for off
         */
        bool getResetSwitch();
        
        // LCD SCREEN
        SerLCD lcd;

        /**  
         * @brief Displays a 'msg' string on the screen, left justified 
         * with text wrapping. The 'status' string is displayed 
         * in the lower right corner and is right justified.
         * @param msg main message to be displayed
         * @param status status update to be displayed
         * @param cls clear screen is set to 'true' by default.
         *            Will reduce flickering if set to 'false' 
         *            while updating a 'status' only.
         */
        void messageScreen(const String& msg, const bool& cls=true,
                           const String& status="");

        /**  
         * @brief Displays a 'test completed' screen which includes the
         * total loops completed as well as test duration
         * @param loop_count this is the current loop count
         * @param total_loops this is the total number of loops from test spec
         * @param cls clear screen is set to 'true' by default.
         *            Will reduce flickering if set to 'false' 
         *            while updating a 'status' only.
         */
        void testDoneScreen(const uint8_t& loop_count);
        
        /**  
         * @brief Updates the main test loop LCD status screen with all
         * pertinent test information including current status,
         * temp setpoint, measured torque, seal and oil temp, and
         * loop count.
         * @param test_status current status of test to be displayed
         * @param loop_count current loop count
         */
        void updateLCD(const String& test_status);

        /**
         * @brief Interrupts with a red flashing error screen and
         * displays the provided error message. Error screen will be
         * ignored after the ignore time provided, or will halt remaining
         * execution of test program.
         * @param msg error message to be displayed
         * @param time amount of time in seconds error will be displayed, 
         * set to 0 to halt program 
         */
        void errorScreen(const String& msg, const int& time=0);

        /**  
         * Write an entry to the log file. Writes one line per execution
         * @param msg msg to be logged
         * @param type type of message, e.g. INFO, LOG, ERROR
         * @param first if first is true, then it will print msg without 
         *              any formatting
         * 
         */ 
        void writeToLog(const String& msg, const String& type="LOG",
                        const bool& first=false);

        /**  
         * Write an entry to the the data file after a defined interval.
         * @param interval interval in seconds that data should be logged
         */ 
        void setRecordInterval(const uint8_t& interval);

        /**  
         * Will write an entry to the test data file
         */ 
        void writeToDataFile();

        /**  
         * Will update the state file
         */ 
        void writeToStateFile();

        // Heater PID Loop //

        /**  
         * Computes the PID loop for heater control at the specified
         * interval and writes PWM output to the heater output pin.
         * PID loop will not update unless specified interval has passed.
         * @param interval interval between updates, in milliseconds
         */
        void computeHeaterOutput(const unsigned int& interval=100);

        /**  
         * Sets the heater safety pin to high, allowing the output pin
         * to control the heater.
         */
        void armHeaters();

        /**  
         * Sets both the heater safety pin and heater output pin to low
         * to preven the heater band from receiving power.
         */ 
        void turnOffHeaters();

        /**
         * @brief Runs the reset test code. loop_count will be set to
         * zero and the pressure offset routine will run again. 
         */
        void resetTest();

    private:
        // PIN MAPPINGS
        std::map<String, uint8_t> _pinMappings;

        // TIME
        static time_t _getTeensyTime();
        uint32_t _loop_count;

        // SENSORS
        bool _temp_units; // false for farenheit, true for celcius
        uint8_t _SEAL_TC_ADDR = MCP9600_DEFAULT_ADDR_ONE;
        uint8_t _SUMP_TC_ADDR = MCP9600_DEFAULT_ADDR_TWO;
        MCP9600 _sealTempSensor, _sumpTempSensor;
        double _seal_temp, _sump_temp;
        SparkFun_MicroPressure _pressSensor;
        double _abs_pressure, _rel_pressure;
        double _press_offset=0.0;
        bool _seal_fault = false;
        bool _sump_fault = false;
        bool _press_fault = false;
        char _torque_pin;
        double _cw_torque, _ccw_torque;

        // HEATER
        elapsedMillis _PIDTimer, _heatSafetyTimer;
        uint8_t _heat_safety_pin, _heat_output_pin;
        double _setpoint_temp, _input, _output;
        double _Kp = 60, _Ki = 40, _Kd = 25;
        double _deltaT_safety;
        PID _heaterPIDControl;
        bool _areHeatersArmed;

        // SWITCH
        uint8_t _run_sw_pin, _reset_sw_pin;
        Bounce *_runSwitch, *_resetSwitch;

        // BUS PINS
        uint8_t _run_bus_pin, _reset_bus_pin;
        uint8_t _loop_bus_pin; //_read_torque_bus_pin;
        uint8_t _supply_bus_pin, _dump_bus_pin;

        // AIR VALVES
        uint8_t _supply_valve_pin, _dump_valve_pin;
        bool _isPressureApplied;

        // SD CARD AND FILES
        File _restartFile, _dataFile, _logFile;
        elapsedMillis _dataLoggerTimer;
        uint8_t _record_interval;
        bool _isSDCardInserted, _isSDCardActive;
        bool _hasHeaderBeenWritten;
        uint8_t _SD_detect_pin;

        // LCD SCREEN
        elapsedMillis_h
        elapsedMillis _screenTimer, _errorTimer, _completeTimer;
        bool _flasher, _isScreenUpdate = true;

        /**  
         * Prints a row on the lcd screen using four strings, they will 
         * be printed on four columns equally spaced.
         * @param row row of the lcd on which to print
         * @param str1 first string
         * @param str2 second string
         * @param str3 third string
         * @param str4 fourth string
         */  
        void printFourColumnRow(const int& row,
                                const String& str1, const String& str2,
                                const String& str3, const String& str4);

        /**  
         * Takes two strings and prints 'str1' left justified and 'str2' 
         * right justified on the specified 'row' between a distance, 
         * specified by width and a start location specified by 'col' 
         * on the lcd screen.
         * @param row row of the lcd on which to print
         * @param col column of the lcd on which to start print
         * @param width space between start of 1st str and end of 2nd
         * @param str1 first string
         * @param str1 second string
         */
        void printRowPair(const int& col, const int& row, const int& width,
                          const String& str1, const String& str2);

        /**
         * Returns a String with correct suffix for temperature. 
         * ASCII for LCD screen.
         * Pass 'true' after temp to set units to celcius.
         * @param temp temperature to be converted to string
         * @param unit true for celcius, false for farenheit
         */
        String tempToStrLCD(const double& temp, const bool& unit);

        /**
         * Returns a String with correct suffix for temperature. 
         * ASCII for log file.
         * Pass 'true' after temp to set units to celcius.
         * @param temp temperature to be converted to string
         * @param unit true for celcius, false for farenheit
         */
        String tempToStrLog(const double& temp, const bool& unit);

        /**  
         * Calculates the correct amount of padding to space two strings apart
         * by a set amount of characters, from 1st char of 1st string to last 
         * char of 2nd string.
         * @param num_chars num of chars between start of 1st str and end of 2nd
         * @param str1 first string
         * @param str2 second string
         */ 
        String padBetweenChars(const int& num_chars, const String& str1, const String& str2);
        
        /**  
         * Prepends padding using space chars to right justify a string
         * on a line of the lcd screen.
         * @param str string to be right justified
         * @return padded string
         */ 
        String rightJustifiedString(const String& str);

        /**  
         * Returns the current time as a string in 24-hr format
         * @return str in hh:mm:ss
         */
        String getTimeStr();

        /**  
         * Retrun the current date as a string
         * @return str in mm:dd:yyyy
         */
        String getDateStr();


};




#endif // DC_h