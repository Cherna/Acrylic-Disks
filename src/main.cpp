#include <Arduino.h>
#include <AccelStepper.h>

// Define stepper motor connections and steps per revolution
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR3_STEP_PIN 4
#define MOTOR3_DIR_PIN 7
#define MOTOR3_DIR_PIN 9
#define MOTOR4_STEP_PIN 10
#define MOTOR4_DIR_PIN 11
#define MOTOR5_STEP_PIN 12
#define MOTOR5_DIR_PIN 13
#define MOTOR6_STEP_PIN 14
#define MOTOR6_DIR_PIN 15
#define MOTORS_ENABLE 8
#define MICSTEP 2 // Microstepping set on the driver

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);
AccelStepper stepper5(AccelStepper::DRIVER, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN);
AccelStepper stepper6(AccelStepper::DRIVER, MOTOR6_STEP_PIN, MOTOR6_DIR_PIN);

AccelStepper circle1[3] = {stepper1, stepper2, stepper3};
AccelStepper circle2[3] = {stepper1, stepper2, stepper3};
AccelStepper allSteppers[6] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};

template <typename T, size_t N>
size_t getArrayLength(const T (&)[N]) {
    return N;
}

void setup() {
  // Set up the serial communication
  Serial.begin(115200);

  // Enable motors
  pinMode(MOTORS_ENABLE, OUTPUT);
  digitalWrite(MOTORS_ENABLE, LOW);


  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : allSteppers) {
    motor.setMaxSpeed(4000 * MICSTEP);
    motor.setAcceleration(4000 * MICSTEP);
  }
}

// TODO: Function to run homing sequence
void homeAllMotors(AccelStepper (&circle)[3] = circle1) {
  // Move all motors in a certain direction until the home switches are tripped
}

int angleToSteps(int angle) {
  static double hypothenuseInMm = 1700;
  static double circumferenceInMm = 264;
  static double distancePerStepInMm = 0.04;
  double angleInRadians = angle * M_PI / 180.0;
  double oppositeSideInMm = hypothenuseInMm * sin(angleInRadians);

  return oppositeSideInMm / distancePerStepInMm; 
}

// TODO: Function to hold specific angle
void moveCircleToAngle(
  AccelStepper (&circle)[3] = circle1,
  int angle = 10, // Angle in degrees
  int direction = 0
) {
  circle[direction].move(angleToSteps(angle) * MICSTEP);
}

// TODO: Function to move a circle to a "neutral" horizontal position
void normalizeMotorsHeight(AccelStepper (&circle)[3] = circle1) {
  // Get the current position of the motor and set the new position to
  // a round number closest to the average between this circles motors position
  int avgPos = (
    circle[0].currentPosition() +
    circle[1].currentPosition() +
    circle[2].currentPosition()
    )/3;
  
  for (auto &motor : circle) {
    motor.moveTo(avgPos);
  }
}

// TODO: Function to move entire circle up
void allMotorsUp(AccelStepper (&circle)[3] = circle1) {
  for (auto &motor : circle) {
    motor.move(-1000 * MICSTEP);
  }
}

// TODO: Function to move entire circle down
void allMotorsDown(AccelStepper (&circle)[3] = circle1) {
  for (auto &motor : circle) {
    motor.move(1000 * MICSTEP);
  }
}

void stopAllMotors() {
  for (auto &motor : allSteppers) {
    motor.stop();
  }
}

bool anyMotorMoving() {
  for (auto &motor : allSteppers) {
    if (motor.isRunning()) {
      return true;
    }
  }
  return false;
}

void runSine(
  bool reset = false,
  bool debug = false,
  AccelStepper (&circle)[3] = circle1,
  int magnitude = 200, // Vertical magnitude of the wave in steps
  float overlap = 0.5 // Fraction of the wave to overlap with the previous move
) {
  static int currentMove = 0;
  static int nextMove = 1;
  if (reset) {
    currentMove = 0;
    nextMove = 1;
  }
  static int direction = 1;

  for(auto &motor : circle) {
    motor.setMaxSpeed(100 * MICSTEP);
  }

  // Start by moving the first motor if no motor is moving
  if (!anyMotorMoving()) {
    circle[currentMove].move(magnitude * MICSTEP * direction);
  }

  if (debug) {
    Serial.println(abs(circle[currentMove].distanceToGo()));
    Serial.println(magnitude * MICSTEP * overlap);

    Serial.print("Current move: ");
    Serial.println(currentMove);
    Serial.print("Next move: ");
    Serial.println(nextMove);
  };

  if (
    circle[currentMove].isRunning() &&
    abs(circle[currentMove].distanceToGo()) <= magnitude * MICSTEP * overlap
  ) {
    if (debug) {
      Serial.println("Starting next move");
    };
    circle[nextMove].move(magnitude * MICSTEP * direction);

    // Reverse current move direction
    direction *= (currentMove == getArrayLength(circle) - 1) ? -1 : 1;
    currentMove = (currentMove == getArrayLength(circle) - 1) ? 0 : currentMove + 1;
    nextMove = (nextMove == getArrayLength(circle) - 1) ? 0 : nextMove + 1;
    // // Advance currentMove or reset to 0 if we've reached the end
    // if (currentMove == (MOTOR_COUNT - 1)) {
    //   currentMove = 0;
    // } else if (nextMove == (MOTOR_COUNT - 1)) {
    //   nextMove = 0;
    // } else {
    //   currentMove++;
    //   nextMove++;
    // }
  }
}

void runSequentially(AccelStepper (&circle)[3] = circle1) {
  int counter = 0;
  counter = counter == getArrayLength(circle) - 1 ? 0 : counter;
  unsigned long previousMillis = previousMillis || 0;
  int direction = direction || 1;
  if (anyMotorMoving()) {
    // Stop all motors
    stopAllMotors();
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 500) {
      Serial.print("Moving: ");
      Serial.println(counter);
      allSteppers[counter].move(200 * MICSTEP * direction);

      // Increase motor counter
      ++counter;

      // Reverse the direction after all motors have run
      if (counter >= getArrayLength(circle)) {
        Serial.println("Resetting counter and reversing");
        counter = 0;
        direction *= -1;
      }

      // Reset the timer for the next interval
      previousMillis = currentMillis;
    }
  }
}

void loop() {
  // runSequentially();
  runSine();

  for (auto &motor : allSteppers) {
    motor.run();
  }
}