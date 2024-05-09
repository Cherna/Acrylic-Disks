#include <Arduino.h>
#include <AccelStepper.h>
#include <array>

// Define stepper motor connections and steps per revolution
#define MOTOR1_STEP_PIN 12
#define MOTOR1_DIR_PIN 13
#define MOTOR2_STEP_PIN 14
#define MOTOR2_DIR_PIN 15
#define MOTOR3_STEP_PIN 16
#define MOTOR3_DIR_PIN 17
#define MOTOR4_STEP_PIN 18
#define MOTOR4_DIR_PIN 19
#define MOTOR5_STEP_PIN 20
#define MOTOR5_DIR_PIN 21
#define MOTOR6_STEP_PIN 22
#define MOTOR6_DIR_PIN 23

#define MOTOR1_EN 4
#define MOTOR2_EN 25
#define MOTOR3_EN 26
#define MOTOR4_EN 27
#define MOTOR5_EN 32
#define MOTOR6_EN 33

#define UP_BTN 34 // Needs external PULLDOWN resistor
#define DOWN_BTN 35 // Needs external PULLDOWN resistor

#define DEFAULT_SPEED 1000
#define DEFAULT_ACCELERATION 4000

#define MICSTEP 16 // Microstepping set on the driver

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);
AccelStepper stepper5(AccelStepper::DRIVER, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN);
AccelStepper stepper6(AccelStepper::DRIVER, MOTOR6_STEP_PIN, MOTOR6_DIR_PIN);

std::array<AccelStepper*, 3> circle1 = {&stepper1, &stepper2, &stepper3};
std::array<AccelStepper*, 3> circle2 = {&stepper4, &stepper5, &stepper6};
std::array<AccelStepper*, 6> allSteppers = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};

template <typename T, size_t N>
size_t getArrayLength(const std::array<T, N> &) {
    return N;
}

void setup() {
  // Set up the serial communication
  Serial.begin(250000);

  // Set up buttons
  pinMode(UP_BTN, INPUT);
  pinMode(DOWN_BTN, INPUT);

  // Set up enable pins
  pinMode(MOTOR1_EN, OUTPUT);
  digitalWrite(MOTOR1_EN, HIGH);
  pinMode(MOTOR2_EN, OUTPUT);
  digitalWrite(MOTOR2_EN, HIGH);
  pinMode(MOTOR3_EN, OUTPUT);
  digitalWrite(MOTOR3_EN, HIGH);
  pinMode(MOTOR4_EN, OUTPUT);
  digitalWrite(MOTOR4_EN, HIGH);
  pinMode(MOTOR5_EN, OUTPUT);
  digitalWrite(MOTOR5_EN, HIGH);
  pinMode(MOTOR6_EN, OUTPUT);
  digitalWrite(MOTOR6_EN, HIGH);

  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : allSteppers) {
    motor->setMaxSpeed(DEFAULT_SPEED * MICSTEP);
    motor->setAcceleration(DEFAULT_ACCELERATION * MICSTEP);
  }
}

// TODO: Function to run homing sequence
void homeAllMotors(std::array<AccelStepper*, 3> circle = circle1) {
  // Move all motors in a certain direction until the home switches are tripped
}

void toggleCircle(std::array<AccelStepper*, 3> circle, bool enable) {
  if (circle == circle1) {
    digitalWrite(MOTOR1_EN, !enable);
    digitalWrite(MOTOR2_EN, !enable);
    digitalWrite(MOTOR3_EN, !enable);
  } else {
    digitalWrite(MOTOR4_EN, !enable);
    digitalWrite(MOTOR5_EN, !enable);
    digitalWrite(MOTOR6_EN, !enable);
  }
}

bool isCircleEnabled(std::array<AccelStepper*, 3> circle) {
  if (circle == circle1) {
    return !digitalRead(MOTOR1_EN);
  } else {
    return !digitalRead(MOTOR4_EN);
  }
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
  std::array<AccelStepper*, 3> circle = circle1,
  int angle = 10, // Angle in degrees
  int direction = 0
) {
  circle[direction]->moveTo(angleToSteps(angle) * MICSTEP);
}

// TODO: Function to move a circle to a "neutral" horizontal position
void normalizeMotorsHeight(std::array<AccelStepper*, 3> circle = circle1) {
  // Get the current position of the motor and set the new position to
  // a round number closest to the average between this circles motors position
  int avgPos = (
    circle[0]->currentPosition() +
    circle[1]->currentPosition() +
    circle[2]->currentPosition()
    )/3;
  
  for (auto &motor : circle) {
    motor->moveTo(avgPos);
  }
}

void setSpeedAll(int speed = DEFAULT_SPEED) {
  for (auto &motor : allSteppers) {
    motor->setMaxSpeed(speed * MICSTEP);
  }
}

void setAccelAll(int accel = DEFAULT_ACCELERATION) {
  for (auto &motor : allSteppers) {
    motor->setAcceleration(accel * MICSTEP);
  }
}

// Function to move entire circle up
void allMotorsUp(std::array<AccelStepper*, 3> circle = circle1) {
  for (auto motor : circle) {
    motor->move(-200 * MICSTEP);
  }
}

// Function to move entire circle down
void allMotorsDown(std::array<AccelStepper*, 3> circle = circle1) {
  for (auto motor : circle) {
    motor->move(200 * MICSTEP);
  }
}

void stopAllMotors() {
  for (auto &motor : allSteppers) {
    motor->stop();
  }
}

void circleToZero(std::array<AccelStepper*, 3> circle) {
  for (auto &motor : circle) {
    motor->moveTo(0);
  }
}

bool isCircleMoving(std::array<AccelStepper*, 3> circle) {
  for (auto &motor : circle) {
    if (motor->isRunning()) {
      return true;
    }
  }
  return false;
}

struct sineArgs {
  bool reset = false;
  std::array<AccelStepper*, 3> circle = circle1;
  int magnitude = 500; // Vertical magnitude of the wave in steps
  double overlap = 0.33; // Fraction of the next move to overlap with the previous move
  int speed = 200;
};

static sineArgs baseArgs;
void static runSine(sineArgs args = baseArgs) {
  static int currentMove = 0;
  static int nextMove = 1;
  static int direction = 1;
  static bool start = true;

  static bool reset = false;
  static int magnitude = args.magnitude;
  static double overlap = args.overlap;
  static int speed = args.speed;
  if (args.reset) {
    currentMove = 0;
    nextMove = 1;
    direction = 1;
    start = true;
    reset = false;
  }

  if (
    args.circle[currentMove]->isRunning() &&
    abs(args.circle[currentMove]->distanceToGo()) <= magnitude * MICSTEP * overlap ||
    start
  ) {
    // Serial.println("Starting next move");
    start = false;
    args.circle[nextMove]->move(magnitude * MICSTEP * direction);

    direction *= (currentMove == getArrayLength(args.circle) - 1) ? -1 : 1;
    currentMove = (currentMove == getArrayLength(args.circle) - 1) ? 0 : currentMove + 1;
    nextMove = (nextMove == getArrayLength(args.circle) - 1) ? 0 : nextMove + 1;
  }
}

void runSequentially(std::array<AccelStepper*, 3> circle = circle1) {
  int counter = 0;
  counter = counter == getArrayLength(circle) - 1 ? 0 : counter;
  unsigned long previousMillis = previousMillis || 0;
  int direction = direction || 1;
  if (!isCircleMoving(circle)) {
    // Stop all motors
    stopAllMotors();
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 500) {
      Serial.print("Moving: ");
      Serial.println(counter);
      allSteppers[counter]->move(200 * MICSTEP * direction);

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

void static runSlowSine(bool reset = false) {
  sineArgs args {};
  args.magnitude = 1000;
  args.overlap = 0.5;
  if (reset) {
    setAccelAll(100);
    setSpeedAll(200);
    args.reset = true;
    runSine(args);
  } else {
    args.reset = false;
    runSine(args);
  }
}

void static runFastSine(bool reset = false) {
  sineArgs args {};
  args.magnitude = 800;
  args.overlap = 0.5;
  if (reset) {
    setAccelAll(800);
    setSpeedAll(2000);
    args.reset = true;
    runSine(args);
  } else {
    args.reset = false;
    runSine(args);
  }
}

void static runKeepAngle(bool reset = false) {
  if (reset) {
    setAccelAll(DEFAULT_ACCELERATION);
    setSpeedAll(DEFAULT_SPEED);
    moveCircleToAngle(circle1, 2, 0);
  }
}

void runSequence(bool next = false) {
  static int current = 0;
  static std::array<void (*)(bool reset), 3> sequence = {
    runSlowSine,
    runFastSine,
    runKeepAngle
  };
  if (next) {
    current = current >= (getArrayLength(sequence) - 1) ? 0 : current + 1;
    Serial.println(current);
  }
  sequence[current](next);
}

void resetCircle(std::array<AccelStepper*, 3> circle) {
  setAccelAll(DEFAULT_ACCELERATION);
  setSpeedAll(DEFAULT_SPEED);
  circleToZero(circle);
}

static unsigned long previousLogMillis = 0;
static unsigned long prevCountMillis = 0;
static bool stopped = true;
static bool awaitToFinishReset = false;
void loop() {
  // if (millis() - previousLogMillis >= 1000) {
  //   previousLogMillis = millis();
  //   Serial.print("UP: ");
  //   Serial.println(digitalRead(UP_BTN));
  //   Serial.print("DOWN: ");
  //   Serial.println(digitalRead(DOWN_BTN));
  // }
  
  // if (millis() - previousLogMillis >= 100) {
  //   previousLogMillis = millis();
  //   Serial.print("1: ");
  //   Serial.println(stepper1.currentPosition());
  //   Serial.print("2: ");
  //   Serial.println(stepper2.currentPosition());
  //   Serial.print("3: ");
  //   Serial.println(stepper3.currentPosition());
  // }

  // if (digitalRead(UP_BTN)) {
  //   allMotorsUp();
  // } else if (digitalRead(DOWN_BTN)) {
  //   allMotorsDown();
  // }

  // Stop
  if (digitalRead(UP_BTN) && !stopped) {
    Serial.println("STOP");
    stopped = true;
    resetCircle(circle1);
  }

  // Start
  if (digitalRead(DOWN_BTN) && stopped) {
    Serial.println("START");
    stopped = false;
    runSequence(true);
  }

  // Continue
  if (!stopped) {
    if (millis() - previousLogMillis >= 2) {
      previousLogMillis = millis();
      Serial.println(stopped);
    }
    runSequence();
    // if (awaitToFinishReset) {
    //   Serial.println("Awaiting finish reset");
    //   if (!isCircleMoving(circle1)) {
    //     awaitToFinishReset = false;
    //   }
    // } else {
      // if (millis() - prevCountMillis >= 10000) {
      //   Serial.println("Switching Sequence");
      //   prevCountMillis = millis();
      //   runSequence(true);
      //   // resetCircle(circle1);
      //   // awaitToFinishReset = true;
      // } else {
      //   Serial.println("Continuing");
      //   runSequence();
      // }
    // }
  }

  // Disable when not moving
  if (!isCircleMoving(circle1) && isCircleEnabled(circle1)) {
    Serial.println("Disabling");
    toggleCircle(circle1, false);
  }

  // Enable when moving
  if (isCircleMoving(circle1) && !isCircleEnabled(circle1)) {
    Serial.println("Enabling");
    toggleCircle(circle1, true);
  }

  if (isCircleEnabled(circle1)) {
    for (auto &motor : circle1) {
      motor->run();
    }
  }

  if (isCircleEnabled(circle2)) {
    for (auto &motor : circle2) {
      motor->run();
    }
  }
}