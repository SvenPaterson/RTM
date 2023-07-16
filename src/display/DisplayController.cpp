#include "DisplayController.h"

#include <SparkFun_MCP9600.h>

DisplayController::DisplayController() {
    _sealTempSensor.begin(_SEAL_TC_ADDR);
    _sealTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sealTempSensor.setThermocoupleResolution(RES_14_BIT);
    _sumpTempSensor.begin(_SUMP_TC_ADDR);
    _sumpTempSensor.setAmbientResolution(RES_ZERO_POINT_25);
    _sumpTempSensor.setThermocoupleResolution(RES_14_BIT);
}

void DisplayController::updateTemps() {
    _seal_temp = _sealTempSensor.getThermocoupleTemp(false);
    _sump_temp = _sumpTempSensor.getThermocoupleTemp(false);
}

double DisplayController::getSealTemp() {
    return _seal_temp;
}

double DisplayController::getSumpTemp() {
    return _sump_temp;
}