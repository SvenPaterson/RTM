/*
  this has all the code to run the SEW spec and update the LCD creen and save data
  every cycle. this also has the PID code to run the heaters but is not enabled.


*/

#include <Wire.h>
#include <FlexyStepper.h>
#include <avr/pgmspace.h>
#include <Bounce2.h>



//Interupt pins and variables

volatile unsigned long start_micros = 0.0;
volatile unsigned long end_micros = 0;
volatile unsigned long duration = 0;

volatile bool Motor_Error = LOW;
volatile bool Resetbool = false;

// Define a stepper and the pins it will use
FlexyStepper stepper;
uint16_t PPR = 200;//(800 / 3);  // pulses per rev for motor

const uint8_t LoopPin = 1;  // This Pin is used to keeps track of the loops compleated by the motor controller.
const uint8_t PWM_TorquePin = 2;   //This pin reads the PWM signal from the motor for torque. ** Not Used at this time**
const uint8_t MOTOR_STEP_PIN = 3;
const uint8_t MOTOR_DIRECTION_PIN = 4;
const uint8_t EnablePin = 5;        // This is the pin that enables the motor.
const uint8_t AirValvePin = 14;        // the number of the AirValvePin pin
const uint8_t AirDumpValvePin = 15 ;        // the number of the AirValvePin pin
const uint8_t ProgramRunPin = 10; //Pin That tell the to run the motor program.
const uint8_t TorqueFLagPin = 11; //Pin That tell the to run the motor program.
const uint8_t LEDPIN = 13;
const uint8_t ResetPin = 20;  // This Pin is used

/****** TEST STEP PARAMS ******/
uint32_t sum_time = 0;

uint16_t step1_speed = 100;//2475; // rpm
double step1_accel = step1_speed / 8; // rpm per sec
uint32_t step1_time = 18000;

uint16_t step2_speed = 200;//8500;
double step2_accel = (step2_speed - step1_speed) / 15;
uint32_t step2_time = 15000;

uint16_t step3_speed = 0;
double step3_accel = step2_speed / 5;
uint32_t step3_time = 5000;

bool ProgramRun = false; //Pin That tell the to run the motor program.
bool Runbool = false;

elapsedMillis loop_time;

void display_srcfile_details();
void rising();
void falling();
int pgm_lastIndexOf(uint8_t c, const char * p);

void setup() ////////////////////////////////////////////////////////////Setup ////////////////////////////////////////////////////////////////
{
  pinMode(PWM_TorquePin, INPUT_PULLUP); // Motor torque pin.
  pinMode(ProgramRunPin, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(ResetPin, INPUT_PULLDOWN); // This pin get the signel from the disply to run the programe.
  pinMode(LEDPIN, OUTPUT);  // Make it an OUTput.
  pinMode(AirValvePin, OUTPUT);  // Make it an OUTput.
  pinMode(AirDumpValvePin, OUTPUT);// Make it an OUTput.
  pinMode(EnablePin, OUTPUT);   // Make it an OUTput.
  pinMode(MOTOR_STEP_PIN, OUTPUT);   // Make it an OUTput.
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);   // Make it an OUTput.
  pinMode(LoopPin, OUTPUT);   // Make it an OUTput.
  pinMode(TorqueFLagPin, OUTPUT);   // Make it an OUTput.

  digitalWrite (EnablePin, LOW);
  digitalWrite (AirValvePin, LOW);
  digitalWrite (AirDumpValvePin, LOW);
  digitalWrite (LEDPIN, HIGH);
  digitalWrite (LoopPin, LOW);
  digitalWrite (TorqueFLagPin, LOW);
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

  Serial.begin(115200); // Initalize UART, I2C bus, and connect to the micropressure sensor

  Wire.begin(); // start the I2C bus
  Wire.setClock(100000); //Optional - set I2C SCL to High Speed Mode of 100kHz

  delay(2000);
  display_srcfile_details();

  attachInterrupt(digitalPinToInterrupt(ResetPin), rising, RISING); // Enable the interupt.
}

void loop() /////////////////////////////////////////////////Loop///////////////////////////////////////////////////////////////////////////
{
  Runbool = digitalRead(ProgramRunPin);

  if (Runbool ==  LOW) {//If the switch is in mid position, DO nothing.
    digitalWrite (LEDPIN, LOW);
    digitalWrite (EnablePin, LOW);
    delay(100);
    if (Resetbool ==  HIGH) {
      Resetbool = LOW;
      display_srcfile_details();
    }
  }

  else  {
    loop_time = 0;
    digitalWrite (LEDPIN, HIGH);
    digitalWrite (EnablePin, HIGH);
    stepper.setTargetPositionInSteps(2147483647); // set motor to turn to max pos
    
    // test step 1
    stepper.setSpeedInStepsPerSecond((step1_speed / 60) * PPR);
    stepper.setAccelerationInStepsPerSecondPerSecond((step1_accel / 60) * PPR);
    digitalWrite (LEDPIN, !digitalRead(LEDPIN));
    sum_time += step1_time;
    while (loop_time < sum_time) {
      stepper.processMovement();
      if (Resetbool) {
        break;
      }
    }

    // test step 2
    stepper.setSpeedInStepsPerSecond((step2_speed / 60)*PPR);
    stepper.setAccelerationInStepsPerSecondPerSecond((step2_accel / 60) * PPR);
    digitalWrite (LEDPIN, !digitalRead(LEDPIN));
    sum_time += step2_time;
    while (loop_time < sum_time) {
      stepper.processMovement();
      if (Resetbool) {
        break;
      }
    }

    // test step 3
    stepper.setSpeedInStepsPerSecond((step3_speed / 60)*PPR);
    stepper.setAccelerationInStepsPerSecondPerSecond((step3_accel / 60) * PPR);
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    sum_time += step3_time;
    while (loop_time < sum_time) {
      stepper.processMovement();
      if (Resetbool) {
        break;
      }
    }

    digitalWrite (LoopPin, HIGH);
    delay(5);
    digitalWrite (LoopPin, LOW);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Functions/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void rising() {
  attachInterrupt(digitalPinToInterrupt(ResetPin), falling, FALLING);
  start_micros = micros();
}

void falling() {
  end_micros = micros();
  duration = end_micros - start_micros;
  attachInterrupt(digitalPinToInterrupt(ResetPin), rising, RISING); // Enable the interupt.

  if (duration >= 9900 && duration < 10100) {
    Resetbool = HIGH;
  } else Resetbool = LOW;
}

int pgm_lastIndexOf(uint8_t c, const char * p)// displays at startup the Sketch running in the Arduino
{
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++) {
    b = pgm_read_byte(p++);
    if (b == c)
      last_index = i;
    else if (b == 0) break;
  }
  return last_index;
}

void display_srcfile_details(void) { // displays at startup the Sketch running in the Arduino

  const char *the_path = PSTR(__FILE__);           // save RAM, use flash to hold __FILE__ instead

  int slash_loc = pgm_lastIndexOf('/', the_path); // index of last '/'
  if (slash_loc < 0) slash_loc = pgm_lastIndexOf('\\', the_path); // or last '\' (windows, ugh)

  int dot_loc = pgm_lastIndexOf('.', the_path);  // index of last '.'
  if (dot_loc < 0) dot_loc = pgm_lastIndexOf(0, the_path); // if no dot, return end of string

  Serial.print("\nFirmware Version: ");

  for (int i = slash_loc + 1; i < dot_loc; i++) {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0) {
      Serial.print((char) b);
    }
    else break;

  }

  Serial.print(", Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.print("\n");

}

