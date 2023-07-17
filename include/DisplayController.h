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

        // Initialize connected devices
        void begin();
        void setHeaterPins(const byte& saftey_pin, const byte& output_pin);
        void setSwitchPins(const byte& run_pin, const byte& reset_pin);
        void setTempSetpoint(const double& setpoint, const bool& units=false);
        void setBusPins(const byte& run_bus_pin, const byte& reset_bus_pin,
                        const byte& read_torque_bus_pin, const byte& loop_bus_pin);
        void setTorquePin(const byte& torque_pin);

        // read sensors & inputs
        void update();
        void setPressureOffset();
        void readTorque();

        // return measured values
        double getSealTemp();
        double getSumpTemp();
        double getPressure();
        bool getRunSwitch();
        bool getResetSwitch();
        
        // LCD SCREEN
        SerLCD lcd;
        void messageScreen(const String& msg, const bool& cls=true, const String& status="");
        void updateLCD(const String& test_status, const unsigned int& loop_count);

        // Heater PID Loop
        void computeHeaterOutput();
        void armHeaters();
        void turnOffHeaters();

    private:
        String _msg;
        
        // SENSORS
        bool _temp_units = false; // true for Celcius TODO
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

        // LCD SCREEN
        elapsedMillis _screenTimer;
        bool _isScreenUpdate = true;
        void printFourColumnRow(const int& row,
                                const String& str1, const String& str2,
                                const String& str3, const String& str4);
        void printRowPair(const int& col, const int& row, const int& width,
                          const String& str1, const String& str2);
        void errorScreen(const String& msg);

        // String Constructors
        String tempToStr(const double& temp, const bool& unit);
        String padBetweenChars(const int& num_chars, const String& str1, const String& str2);
        String rightJustifiedString(const String& str);
};


#endif // DC_h