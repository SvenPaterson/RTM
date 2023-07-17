#ifndef DC_h
#define DC_h

#include <Arduino.h>
#include <SparkFun_MCP9600.h>
#include <SparkFun_MicroPressure.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PID_v1.h>
#include <Bounce2.h>

#define MCP9600_DEFAULT_ADDR_ONE 0x60
#define MCP9600_DEFAULT_ADDR_TWO 0x67

class DisplayController {
    public:
        DisplayController();

        // Initialize connected devices
        void begin();
        void setHeaterPins(const byte& saftey_pin, const byte& output_pin);
        void setSwitchPins(const byte& run_pin, const byte& reset_pin);
        void setTempSetpoint(double setpoint);
        void setBusPins(const byte& run_bus_pin, const byte& reset_bus_pin,
                        const byte& read_torque_bus_pin, const byte& loop_bus_pin);
        
        // read sensors & unputs
        void update();
        void setPressureOffset();

        // return measured values
        double getSealTemp();
        double getSumpTemp();
        double getPressure();
        bool getRunSwitch();
        bool getResetSwitch();
        
        // LCD SCREEN
        SerLCD lcd;
        void messageScreen(const String& msg, const bool& cls=true, const String& status="");

        // Heater PID Loop
        double computeHeaterOutput();
        void armHeaters();
        void turnOffHeaters();
        

    private:
        String _msg;
        
        // SENSORS
        char _SEAL_TC_ADDR = MCP9600_DEFAULT_ADDR_ONE;
        char _SUMP_TC_ADDR = MCP9600_DEFAULT_ADDR_TWO;
        MCP9600 _sealTempSensor, _sumpTempSensor;
        double _seal_temp, _sump_temp;

        SparkFun_MicroPressure _pressSensor;
        double _pressure, _press_offset=0.0;

        bool _seal_fault = false;
        bool _sump_fault = false;
        bool _press_fault = false;

        // HEATER
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

        void errorScreen(const String& msg);
        String rightJustifiedString(const String& str);
};


#endif // DC_h