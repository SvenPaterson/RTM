#ifndef DC_h
#define DC_h

#include <Arduino.h>
#include <SparkFun_MCP9600.h>
#include <SparkFun_MicroPressure.h>
#include <Wire.h>
#include <SerLCD.h>
#include <PID_v1.h>

#define MCP9600_DEFAULT_ADDR_ONE 0x60
#define MCP9600_DEFAULT_ADDR_TWO 0x67

class DisplayController {
    public:
        DisplayController();

        // Initialize connected devices
        void begin();

        void heaterPins(byte saftey_pin, byte output_pin);
        int heaterOutput();

        // read sensors
        void update();

        // return measured values
        double getSealTemp();
        double getSumpTemp();
        double getPressure();
        
        // LCD SCREEN
        SerLCD lcd;
        

    private:
        String _msg;
        
        // SENSORS
        char _SEAL_TC_ADDR = MCP9600_DEFAULT_ADDR_ONE;
        char _SUMP_TC_ADDR = MCP9600_DEFAULT_ADDR_TWO;
        MCP9600 _sealTempSensor, _sumpTempSensor;
        double _seal_temp, _sump_temp;
        bool _seal_fault = false;
        bool _sump_fault = false;
        bool _press_fault = false;

        // HEATER
        byte _safety_pin, _output_pin;
        double _setpoint, _input, _output;
        double _Kp = 60, _Ki = 40, _Kd = 25;
        PID _heaterPIDControl;

        //elapsedMillis PIDTimer;

        SparkFun_MicroPressure _pressSensor;
        double _pressure, _press_offset;
 
};


#endif // DC_h