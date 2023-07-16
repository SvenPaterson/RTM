#ifndef DC_h
#define DC_h

#include <Arduino.h>
#include <SparkFun_MCP9600.h>

#define MCP9600_DEFAULT_ADDR_ONE 0x60
#define MCP9600_DEFAULT_ADDR_TWO 0x67

class DisplayController {
    public:
        DisplayController();
        void updateTemps();
        double getSealTemp();
        double getSumpTemp();


    private:
        char _SEAL_TC_ADDR = MCP9600_DEFAULT_ADDR_ONE;
        char _SUMP_TC_ADDR = MCP9600_DEFAULT_ADDR_TWO;
        MCP9600 _sealTempSensor, _sumpTempSensor;
        double _seal_temp, _sump_temp;
};


#endif // DC_h