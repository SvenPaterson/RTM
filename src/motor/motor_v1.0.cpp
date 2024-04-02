  #include "motor_config.h"
  #include <AccelStepper.h>
  #include <avr/pgmspace.h>

  /******* INTERRUPT VARIABLES *******/
  volatile unsigned long start_micros = 0.0;
  volatile unsigned long end_micros = 0;
  volatile unsigned long duration = 0;
  volatile bool askingToRun = false;

  /******* I/O PINS *******/
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

  /******* SYSTEM STATE CONTROL *******/
  enum SystemState {
  IDLE,
  RUNNING,
  PAUSED,
  RESET_REQUESTED,
  RESUME
  };
  SystemState currentState = IDLE;
  bool isFullyStopped = false;
  bool isStepInitialized = false;
  bool isPauseInitiated = false;
  uint16_t currentStepIndex = 0;
  elapsedMillis LED_timer, step_timer;
  unsigned long pause_start_time = 0;

  /******* STEPPER MOTOR INIT *******/
  AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  uint32_t motor_target_steps = 0;
  uint16_t steps_per_rev = SPR;
  uint8_t size_steps = sizeof(steps) / sizeof(steps[0]);

  /******* FUNC DECLARATIONS *******/
  void display_srcfile_details();
  void reset_rising();
  void reset_falling();
  void run_rising();
  void run_falling();
  int pgm_lastIndexOf(uint8_t c, const char * p);
  void printCurrentStepInfo();
  void printCurrentState();
  uint32_t calcTotalSteps(double initial_velocity_rpm, double acceleration_rpm_s, 
                       double speed_rpm, double duration_ms);


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

    Serial.begin(115200);
    float max_speed_rpm = isHighSpeedGearBox ? 8500.0 : 2800.0;
    stepper.setMaxSpeed(max_speed_rpm / 60 * steps_per_rev);

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
        pause_start_time = step_timer;
        // Flash LED quickly to signal PAUSED state
        if (LED_timer > 100) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            LED_timer = 0;
        }

        // // Instruct the motor to stop as quickly as possible only once
        if (!isPauseInitiated) {
            Serial.println("Motor called to stop");
            stepper.stop();
            motor_target_steps = motor_target_steps - stepper.currentPosition();
            isPauseInitiated = true;
        }

        // Check if the motor has completely stopped and keep running if not
        if (!stepper.run()) {
            digitalWrite(MOTOR_ENABLE_PIN, LOW); // Optionally disable motor power
            isFullyStopped = true;
        }

        // Check if it's time to resume
        if (askingToRun && isFullyStopped) {
            Serial.println("Test called to resume");
            currentState = RESUME;
            isPauseInitiated = false; // Reset pause flag
            printCurrentState();
        }
        break;

        
      case RESUME:
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
        
        if (steps[currentStepIndex].target_speed != 0) {
          // Enable motor driver
          digitalWrite(MOTOR_ENABLE_PIN, HIGH);
          
          // Configure acceleration and speed according to the current step
          stepper.setAcceleration(steps[currentStepIndex].accel / 60 * steps_per_rev);
          stepper.setMaxSpeed(steps[currentStepIndex].target_speed / 60 * steps_per_rev);
          stepper.moveTo(steps[currentStepIndex].is_CCW ? -motor_target_steps : motor_target_steps);
          
        } else {
          // If it's a dwell step, ensure the motor is disabled
          digitalWrite(MOTOR_ENABLE_PIN, LOW);
        }

        Serial.println("Resuming the following step:");
        printCurrentStepInfo();
        
        currentState = RUNNING; // Change the state back to RUNNING to continue with the test sequence
        printCurrentState();
        
        step_timer = pause_start_time;
        break;


      case RUNNING:
        // only perform these actions once
        if (!isStepInitialized) {
          // Initialize common settings for all steps
          digitalWrite(LED_PIN, HIGH);
          digitalWrite(HEAT_BUS_PIN, steps[currentStepIndex].turnOnHeat);
          stepper.setAcceleration(steps[currentStepIndex].accel / 60 * steps_per_rev);
          if (steps[currentStepIndex].target_speed != 0) { // Deceleration to stop
            // Enable motor driver and set speed and acceleration
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.setMaxSpeed(steps[currentStepIndex].target_speed / 60 * steps_per_rev);
            motor_target_steps = calcTotalSteps(stepper.speed(), steps[currentStepIndex].accel, 
                                         steps[currentStepIndex].target_speed, steps[currentStepIndex].time);
            stepper.moveTo(steps[currentStepIndex].is_CCW ? -motor_target_steps : motor_target_steps);
          } else { // Decel and stop step logic
            stepper.stop();
          }
          step_timer = 0; // Reset step timer
          isStepInitialized = true; // Mark step as initialized
          printCurrentStepInfo();
        }

        // run motor if not dwelling
        if (stepper.isRunning()) {
          stepper.run();
        } else {
          digitalWrite(MOTOR_ENABLE_PIN, LOW);
        }

        // Check if the step time has elapsed
        if (step_timer >= steps[currentStepIndex].time && !stepper.isRunning()) {
          Serial.println("Step duration completed");
          isStepInitialized = false;
          currentStepIndex = (currentStepIndex + 1) % size_steps;
          stepper.setCurrentPosition(0);
        }

        // Inform display controller of loop completion
        if (currentStepIndex == 0 && !isStepInitialized) {
          digitalWrite(LOOP_BUS_PIN, HIGH);
          delay(1);
          digitalWrite(LOOP_BUS_PIN, LOW);
        }
        break;
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
    Serial.println(isFullyStopped);
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

  void printCurrentStepInfo() {
    Serial.print("Step ");
    Serial.print(currentStepIndex + 1);
    Serial.print(": Distance = ");
    Serial.print(steps[currentStepIndex].is_CCW ? -1 * motor_target_steps : motor_target_steps);
    Serial.print(" steps, Time = ");
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


// Function signature update to accept initial velocity as an argument
  uint32_t calcTotalSteps(double initial_velocity_rpm, double acceleration_rpm_s, 
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
      return (distance_accel + distance_const) * steps_per_rev;
  }