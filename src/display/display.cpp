#include <Arduino.h>
#include <SerLCD.h>
#include <Wire.h>

// define the board I/O pin numbers
#define SD_DETECT_PIN 0
#define LOOP_BUS_PIN 1
#define MOTOR_HLFB_PIN 2
#define HEAT_OUTPUT_PIN 6
#define HEAT_SAFETY_PIN 7
#define AIR_SUPPLY_PIN 8
#define AIR_DUMP_PIN 9
#define PRGM_RUN_BUS_PIN 10
#define TORQ_FLAG_BUS_PIN 11
#define AIR_SUPPLY_BUS_PIN 14
#define AIR_DUMP_BUS_PIN 15
#define RUN_SW_PIN 16
#define RESET_SW_PIN 17
#define SDA0_PIN 18
#define SDL0_PIN 19
#define RESET_BUS_PIN 20

// INITIALIZE TIMERS
elapsedMillis SysTimer;
elapsedMillis LCDTimer;

// LCD SCREEN
SerLCD lcd;
bool isScreenUpdate = true;

// PRESSURE SENSORS

// FUNCTION DECLARATIONS
void updateLCD();

// INTIAL SETUP 
void setup() {
  Wire.begin(); // start the I2C bus
  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();
}

// MAIN LOOP
void loop() {
  updateLCD();
}

// FUNCTION DEFINITIONS //
void updateLCD() { // This script Updates the LCD withinformation.
  if (LCDTimer > 2000) {
    LCDTimer = 0;
    double psi = 25;//(mpr.readPressure() - psiCalibration);
    delay(5);
    double Sealtemp = 74;//SealtempSensor.getThermocoupleTemp(false);
    delay(5);
    double Sumptemp = 72;//SumptempSensor.getThermocoupleTemp(false);// false returns temp in farenheight.

    double CW_torque = 1.26;
    double CCW_torque = -0.98;
    int LoopCount = 123;
    String Status = "RUNNING";
    int Setpoint = 160;

  if (isScreenUpdate == true) {
      isScreenUpdate = !isScreenUpdate;
      lcd.setCursor(0, 0);
      lcd.print("Status");
      lcd.setCursor(11, 0);
      lcd.print("SetPoint");
      lcd.setCursor(0, 1);
      lcd.print(Status);
      lcd.setCursor(8, 1);
      lcd.print( "    " + String(Setpoint, 0) + String((char)223) + "F   ");
      lcd.setCursor(19, 1);
      lcd.print("  ");
      lcd.setCursor(0, 2);
      lcd.print("PSI: " + String(psi, 1) + " ");
      lcd.setCursor(11, 2);
      lcd.print("Loop:    " );
      lcd.setCursor(16, 2);
      lcd.print(String(LoopCount));
      lcd.setCursor(0, 3);
      lcd.print("Temp: " + String(Sealtemp, 0) + String((char)223) + "F  ");
      lcd.setCursor(11, 3);
      if (Sumptemp >= 100) {
        lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
      } else     lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
    } 
    else {
      isScreenUpdate = !isScreenUpdate;
      lcd.setCursor(0, 0);
      lcd.print("Status");
      lcd.setCursor(11, 0);
      //lcd.print("SetPoint");
      lcd.print("Torque  ");
      lcd.setCursor(0, 1);
      lcd.print(Status);
      lcd.setCursor(8, 1);
      //lcd.print( String(Setpoint, 0) + String((char)223) + "F   ");
      lcd.print(String(CW_torque) + " / " + String(CCW_torque));
      lcd.setCursor(0, 2);
      lcd.print("PSI: " + String(psi, 1) + " ");
      lcd.setCursor(11, 2);
      lcd.print("Loop:    " );
      lcd.setCursor(16, 2);
      lcd.print(String(LoopCount));
      lcd.setCursor(0, 3);
      lcd.print("Temp: " + String(Sealtemp, 0) + String((char)223) + "F ");
      lcd.setCursor(11, 3);
      if (Sumptemp >= 100) {
        lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));
      } else     lcd.print("Sump: " + String(Sumptemp, 0) + String((char)223));

    }
  }
}