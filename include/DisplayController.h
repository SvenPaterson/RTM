#ifndef DC_h
#define DC_h

#include <Arduino.h>
#include <SparkFun_MCP9600.h>
#include <SparkFun_MicroPressure.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PID_v1.h>
#include <Bounce2.h>
#include <TimeLib.h>

#define MCP9600_DEFAULT_ADDR_ONE 0x60
#define MCP9600_DEFAULT_ADDR_TWO 0x67

class DisplayController {
    public:
        DisplayController();

        /**  
         * Set heater control i/o pins, initialize pinMode and
         * write outputs as low for safety.
         * @param safety_pin pin 7, typical
         * @param output_pin pin 6, typical
         */
        void setHeaterPins(const byte& saftey_pin, const byte& output_pin);

        /**  
         * Set rocker switch run and reset control i/o pins and 
         * initialize pinMode.
         * @param run_pin pin 16, typical
         * @param reset_pin pin 17, typical
         */
        void setSwitchPins(const byte& run_pin, const byte& reset_pin);

        /**  
         * Set the heater control setpoint temperature and display message
         * stating that heater control has been initialized.
         * @param setpoint temperature setpoint
         * @param units false for farenheit, true for celcius
         */
        void setTempSetpoint(const double& setpoint, const bool& units=false);
        
        /**  
         * Set i/o bus pins for comms between motor and display controller
         *  i/o pins and initialize pinMode
         * @param run_bus_pin pin 10, typical
         * @param reset_bus_pin pin 20, typical
         * @param loop_bus_pin pin 1, typical
         */
        void setTestBusPins(const byte& run_bus_pin, 
                            const byte& reset_bus_pin,
                            const byte& loop_bus_pin);
        
        /**  
         * Set motor PWM torque measurement pin and initialize pinMode
         * @param torque_pin pin 2, typical
         * @param read_torque_bus_pin pin 11, typical
         */
        void setTorquePins(const byte& torque_pin, const byte& torque_bus_pin);

        /**  
         * Set i/o bus pins air supply and dump valves as well as bus pins for
         * supply and dump commands from motor controller
         * @param supply_bus_pin pin 14, typical
         * @param dump_bus_pin pin 15, typical
         * @param supply_valve_pin pin 8, typical
         * @param dump_valve_pin pin 9, typical
         */
        void setAirPins(const byte& supply_bus_pin, const byte& dump_bus_pin,
                        const byte& supply_valve_pin, const byte& dump_valve_pin);
        /**  
         * Begin serial, wire, and lcd objects. Begin temp and pressure
         * sensors and handle any errors. Begin rocker switch bounce
         * objects.
         */
        void begin();

        /**  
         * Take a number of pressure measurements at 1sec intervals with
         * rig head open to atmosphere and averaged them. Use this value 
         * to set the pressure offset.
         * @param num_meas default: 10 measurements
         */
        void setPressureOffset(const byte& num_meas=10);

        /**  
         * Take measurements from all attached i/o, except torque and
         * update variables.
         */
        void update();

        /**  
         * Take a measurement from the motor high level feedback pin
         * and calculate torque value based on motor direction then
         * updates variables.
         */
        void readTorque();

        /**  
         * @return returns value of _seal_temp
         */
        double getSealTemp();

        /**  
         * @return returns value of _sump_temp
         */
        double getSumpTemp();

        /**  
         * Determines if a pressure offset exists and returns
         * calculate pressure reading.
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
         * @brief Updates the main test loop LCD status screen with all
         * pertinent test information including current status,
         * temp setpoint, measured torque, seal and oil temp, and
         * loop count.
         * @param test_status current status of test to be displayed
         * @param loop_count current loop count
         */
        void updateLCD(const String& test_status, 
                       const unsigned int& loop_count);

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

    private:
        // SENSORS
        bool _temp_units; // false for farenheit, true for celcius
        char _SEAL_TC_ADDR = MCP9600_DEFAULT_ADDR_ONE;
        char _SUMP_TC_ADDR = MCP9600_DEFAULT_ADDR_TWO;
        MCP9600 _sealTempSensor, _sumpTempSensor;
        double _seal_temp, _sump_temp;

        SparkFun_MicroPressure _pressSensor;
        double _pressure, _press_offset=0.0;

        bool _seal_fault = false;
        bool _sump_fault = false;
        bool _press_fault = false;

        char _torque_pin;
        double _cw_torque, _ccw_torque;

        // HEATER
        elapsedMillis _PIDTimer;
        byte _safety_pin, _output_pin;
        double _setpoint_temp, _input, _output;
        double _Kp = 60, _Ki = 40, _Kd = 25;
        PID _heaterPIDControl;

        // SWITCH
        byte _run_pin, _reset_pin;
        Bounce *_runSwitch;
        Bounce *_resetSwitch;

        // BUS PINS
        byte _run_bus_pin, _reset_bus_pin;
        byte _loop_bus_pin, _read_torque_bus_pin;
        byte _supply_bus_pin, _dump_bus_pin;

        // AIR VALVES
        byte _supply_valve_pin, _dump_valve_pin;

        // LCD SCREEN
        elapsedMillis _screenTimer;
        bool _isScreenUpdate = true;

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
         * Returns a String with correct suffix for temperature by default.
         * Pass 'true' after temp to set units to celcius.
         * @param temp temperature to be converted to string
         * @param unit true for celcius, false for farenheit
         */
        String tempToStr(const double& temp, const bool& unit);

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
};


#endif // DC_h