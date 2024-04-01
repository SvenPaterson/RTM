  #include <Wire.h>
  #include <FlexyStepper.h>
  #include <avr/pgmspace.h>
  #include <Bounce2.h>

  //Interupt variables
  volatile unsigned long start_micros = 0.0;
  volatile unsigned long end_micros = 0;
  volatile unsigned long duration = 0;
  volatile bool askingToRun = false;

  const uint8_t LOOP_BUS_PIN = 1;  
  const uint8_t MOTOR_STEP_PIN = 3;
  const uint8_t MOTOR_DIRECTION_PIN = 4;
  const uint8_t MOTOR_ENABLE_PIN = 5;
  const uint8_t AIR_SUPPLY_BUS_PIN = 14;
  const uint8_t AIR_DUMP_BUS_PIN = 15 ;
  const uint8_t PRGM_RUN_BUS_PIN = 10;
  const uint8_t TORQUE_FLAG_BUS_PIN = 11;
  const uint8_t LED_PIN = 13;
  const uint8_t PRGM_RESET_BUS_PIN = 20;
  const uint8_t HEAT_BUS_PIN = 12;

  enum SystemState {
  IDLE,
  RUNNING,
  PAUSED,
  RESET_REQUESTED,
  RESUME
  };
  // Initialize the system state to IDLE at startup
  SystemState currentState = IDLE;
  bool isFullyStopped = false;
  bool isStepInitialized = false;
  uint16_t distance_revs = 0;
  uint16_t currentStepIndex = 0; // keeps track during PAUSE & RESUME states
  elapsedMillis LED_timer, dwell_timer;
  struct Step {
    bool motorDwell;  // true if motor stopped, false for motor spinning
    bool turnOnHeat;  // true for heat, false for none
    bool spin_dir;    // true for CW from front, false for CCW
    uint16_t target_speed;   // rpm
    double accel;     // rpm per second
    uint32_t time;    // milliseconds
  };

  /****** TEST STEP PARAMS ******/

  // enter the test stand type, high speed or standard
  // and the steps per rev in ClearPath MSP software
  bool isHighSpeedGearBox = true;
  // for high speed gear box MUST MUST MUST be a multiple of 3
  uint16_t SPR = 600; 

  // may need to tweak acceleration on steps to ensure achieving set point
  // within the step time or if performing a deceleration step to make sure
  // motor comes to a complete stop before dwell step.
  // UNITS:   speed, rpm | acceleration, rpm/s | time, ms
  Step steps[] = {
  // CODE WILL NOT COMPILE UNTIL YOU CUSTOMIZE AND UNCOMMENT OUT THIS CODE SECTION.
  // motorDwell   heating   direction   speed   acceleration    time
    {false,       true,     true,       3000,   1000,        13000},
    {false,       true,     true,       0,      1000,        3000},
    {true,        true,     true,       0,      0,           5000},
    {false,       true,     false,      3000,   1000,        10000},
    {false,       true,     false,      0,      1000,        3000},
    {true,        true,     false,       0,      0,           5000},
    {false,       false,    true,       1500,   500,        10000},
    {false,       false,    true,       0,      500,        3000},
    {true,        false,    true,      0,      0,           5000},
    {false,       false,    false,      1500,   500,        10000},
    {false,       false,    false,      0,      500,        3000},
    {true,        false,    false,       0,      0,           5000},
  };
  uint8_t size_steps = sizeof(steps) / sizeof(steps[0]);

  FlexyStepper stepper;

  /******* FUNC DECLARATIONS *******/
  void display_srcfile_details();
  void reset_rising();
  void reset_falling();
  void run_rising();
  void run_falling();
  int pgm_lastIndexOf(uint8_t c, const char * p);
  double calcTotalRevs(double initial_velocity_rpm, double acceleration_rpm_s, 
                       double speed_rpm, double duration_ms);
  void printCurrentStepInfo();
  void printCurrentState();


  void setup() {
    pinMode(PRGM_RUN_BUS_PIN, INPUT_PULLDOWN);
    pinMode(PRGM_RESET_BUS_PIN, INPUT_PULLDOWN); 
    pinMode(LED_PIN, OUTPUT);
    pinMode(AIR_SUPPLY_BUS_PIN, OUTPUT);  
    pinMode(AIR_DUMP_BUS_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_STEP_PIN, OUTPUT);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(LOOP_BUS_PIN, OUTPUT);
    pinMode(TORQUE_FLAG_BUS_PIN, OUTPUT);
    pinMode(HEAT_BUS_PIN, OUTPUT);

    digitalWrite (MOTOR_ENABLE_PIN, LOW);
    digitalWrite (AIR_SUPPLY_BUS_PIN, LOW);
    digitalWrite (AIR_DUMP_BUS_PIN, LOW);
    digitalWrite (LED_PIN, HIGH);
    digitalWrite (LOOP_BUS_PIN, LOW);
    digitalWrite (TORQUE_FLAG_BUS_PIN, LOW);

    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    stepper.setStepsPerRevolution(isHighSpeedGearBox ? SPR / 3 : SPR);

    Serial.begin(115200); // Initalize UART, I2C bus, and connect to the micropressure sensor

    Wire.begin(); // start the I2C bus
    Wire.setClock(100000); //Optional - set I2C SCL to High Speed Mode of 100kHz

    delay(2000);
    display_srcfile_details();

    attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_rising, RISING);
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_rising, RISING);
    LED_timer = 0;
    printCurrentState();
  }


  void loop() {
    switch (currentState) {
      case IDLE:
        // flash LED slowly to signal IDLE state
        if (LED_timer > 1000) { 
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          LED_timer = 0;
        }
        
        // power down motor and heaters
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        digitalWrite(HEAT_BUS_PIN, LOW);

        // check for run request
        if (askingToRun) {
          Serial.println("Transitioning to RUNNING state");
          currentState = RUNNING;
          printCurrentState();
          break;
        }

      case RESET_REQUESTED:
        display_srcfile_details();
        currentStepIndex = 0;
        currentState = IDLE;
        printCurrentState();
        break;

      case PAUSED:
        // flash LED quickly to signal PAUSED state
        if (LED_timer > 100) {
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          LED_timer = 0;
        }

        if (!stepper.motionComplete()) {
          /* NOTE:
          The time spent spinning down to a stop (and resuming again)
          will count towards the target distance. FlexyStepper library
          will not allow us to ignore and account for this.
          */
          stepper.setAccelerationInRevolutionsPerSecondPerSecond(15);
          stepper.setTargetPositionToStop();
          stepper.processMovement();

        }
        else if (!isFullyStopped) {
          digitalWrite(MOTOR_ENABLE_PIN, LOW);
          isFullyStopped = true;
        }

        if (askingToRun) {
          currentState = RESUME;
          printCurrentState();
          break;
        }
        
      case RESUME:
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
        if (!steps[currentStepIndex].motorDwell) {
          digitalWrite(MOTOR_ENABLE_PIN, HIGH);
          stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[currentStepIndex].accel / 60);
          stepper.setSpeedInRevolutionsPerSecond(steps[currentStepIndex].target_speed / 60);
          stepper.setTargetPositionInRevolutions(steps[currentStepIndex].spin_dir ? -distance_revs : distance_revs);
        }
        else {
          digitalWrite(MOTOR_ENABLE_PIN, LOW);
        }
        Serial.println("Resuming the following step:");
        printCurrentStepInfo();
        currentState = RUNNING;
        printCurrentState();
        break;

      case RUNNING:
        if (!isStepInitialized) {
          // if current step is 0, previous step is set to final step
          uint8_t prevIndex = (currentStepIndex == 0) ? size_steps - 1 : currentStepIndex - 1;
          distance_revs = calcTotalRevs(steps[prevIndex].target_speed, steps[currentStepIndex].accel, 
                                        steps[currentStepIndex].target_speed, steps[currentStepIndex].time);
          // Initialize the heater and motor
          digitalWrite(LED_PIN, HIGH);
          digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
          if (!steps[currentStepIndex].motorDwell) {
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.setAccelerationInRevolutionsPerSecondPerSecond(steps[currentStepIndex].accel / 60);
            stepper.setSpeedInRevolutionsPerSecond(steps[currentStepIndex].target_speed / 60);
            stepper.setTargetPositionInRevolutions(steps[currentStepIndex].spin_dir ? -distance_revs : distance_revs);
          }
          else {
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
          }
          dwell_timer = 0;
          isStepInitialized = true;
          
          printCurrentStepInfo();
        }

        if (!steps[currentStepIndex].motorDwell) {
          stepper.processMovement();
        }

        if (dwell_timer >= steps[currentStepIndex].time) {
          if (stepper.motionComplete()) {
            Serial.println("Motor motion completed");
            stepper.setCurrentPositionInRevolutions(0); // reset abs. motor pos. to 0
            isStepInitialized = false;
            currentStepIndex = (currentStepIndex + 1) % size_steps;
          }
        }

        if (currentStepIndex == 0 && stepper.motionComplete()) {
          digitalWrite (LOOP_BUS_PIN, HIGH);
          delay(1);
          digitalWrite (LOOP_BUS_PIN, LOW);
        }
    }
  }



  ///////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////Functions//////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  void reset_rising() {
    Serial.println("Entering reset_rising ISR");
    attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_falling, FALLING);
    start_micros = micros();
  }

  void reset_falling() {
    Serial.println("Entering reset_falling ISR");
    end_micros = micros();
    duration = end_micros - start_micros;
    attachInterrupt(digitalPinToInterrupt(PRGM_RESET_BUS_PIN), reset_rising, RISING);

    if (duration >= 9900 && duration < 10100) {
      Serial.println("Transitioning to RESET_REQUESTED state");
      currentState = RESET_REQUESTED;
      printCurrentState();
    }
  }

  void run_rising() {
    Serial.println("Entering run_rising ISR");
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_falling, FALLING);
    askingToRun = true;
  }

  void run_falling() {
    Serial.println("Entering run_falling ISR");
    attachInterrupt(digitalPinToInterrupt(PRGM_RUN_BUS_PIN), run_rising, RISING);
    askingToRun = false;
    currentState = PAUSED;
    printCurrentState();
  }

  int pgm_lastIndexOf(uint8_t c, const char * p)// displays at startup the Sketch spinningning in the Arduino
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

  void display_srcfile_details(void) { // displays at startup the Sketch spinningning in the Arduino

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

  // Function signature update to accept initial velocity as an argument
  double calcTotalRevs(double initial_velocity_rpm, double acceleration_rpm_s, 
                                  double speed_rpm, double duration_ms) {
      // Calculate the effective acceleration time considering the initial velocity
      double time_to_speed_s;
      if (acceleration_rpm_s != 0) {
          time_to_speed_s = (speed_rpm - initial_velocity_rpm) / acceleration_rpm_s;
      } else {
          time_to_speed_s = 0;
      }
      
      // Ensure time_to_speed_s is not negative (which can happen if speed_rpm < initial_velocity_rpm)
      time_to_speed_s = max(0.0, time_to_speed_s);

      // Distance (rotations) during acceleration phase, accounting for initial velocity
      double distance_accel = 0.5 * (acceleration_rpm_s / 60) * (time_to_speed_s * time_to_speed_s) 
                              + (initial_velocity_rpm / 60) * time_to_speed_s;

      // Determine if there's a constant speed phase
      double duration_s = duration_ms / 1000.0;
      double time_constant_s = duration_s > time_to_speed_s ? duration_s - time_to_speed_s : 0.0;

      // Distance (rotations) at constant speed
      double distance_const = (speed_rpm / 60) * time_constant_s;

      // Total rotations
      return distance_accel + distance_const;
  }

  void printCurrentStepInfo() {
    Serial.print("Step ");
    Serial.print(currentStepIndex + 1);
    Serial.print(": Distance = ");
    Serial.print(steps[currentStepIndex].spin_dir ? -1 * distance_revs : distance_revs);
    Serial.print("revs, Time = ");
    Serial.print(steps[currentStepIndex].time);
    Serial.print(" ms, Speed = ");
    Serial.print(steps[currentStepIndex].target_speed);
    Serial.print(" rpm, Acceleration = ");
    Serial.print(steps[currentStepIndex].accel);
    Serial.println(" rpm/s");
  }

  void printCurrentState() {
    switch (currentState) {
      case IDLE:
        Serial.println("Current State: IDLE");
        break;
      case RUNNING:
        Serial.println("Current State: RUNNING");
        break;
      case PAUSED:
        Serial.println("Current State: PAUSED");
        break;
      case RESET_REQUESTED:
        Serial.println("Current State: RESET REQUESTED");
        break;
      case RESUME:
        Serial.println("Current State: RESUME");
        break;
      default:
        Serial.println("Unknown State");
        break;
    }
  }
