#include <Arduino.h>
#include <AccelStepper.h>
#include <array>

#include <utility.h>
#include <sequences.h>

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

const int DEFAULT_SPEED = 1000;
const int DEFAULT_ACCELERATION = 4000;

const int MICSTEP = 16; // Microstepping set on the driver

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

void runSine(
  std::array<AccelStepper*, 3> circle,
  bool reset,
  int magnitude, // Vertical magnitude of the wave in steps
  double overlap // Fraction of the next move to overlap with the previous move)
) {
  static unsigned long logMillis = 0;
  static int currentMove = 0;
  static int nextMove = 1;
  static int direction = 1;
  static bool start = true;

  // static int magnitude = magnitude;
  // static double overlap = overlap;
  if (reset) {
    currentMove = 0;
    nextMove = 1;
    direction = 1;
    start = true;
    reset = false;
  }

  if (millis() - logMillis > 500) {
    logMillis = millis();
    Serial.println(overlap);
    Serial.println(reset);
  }

  if (
    circle[currentMove]->isRunning() &&
    abs(circle[currentMove]->distanceToGo()) <= magnitude * MICSTEP * overlap ||
    start
  ) {
    // Serial.println("Starting next move");
    start = false;
    circle[nextMove]->move(magnitude * MICSTEP * direction);

    direction *= (currentMove == getArrayLength(circle) - 1) ? -1 : 1;
    currentMove = (currentMove == getArrayLength(circle) - 1) ? 0 : currentMove + 1;
    nextMove = (nextMove == getArrayLength(circle) - 1) ? 0 : nextMove + 1;
  }
}

struct sequenceType {
  void (*seqFn)(std::array<AccelStepper*, 3> circle, bool reset);
  int time;
};

sequenceType slowSineSeq = {runSlowSine, 10000};
sequenceType fastSineSeq = {runFastSine, 10000};
sequenceType keepAngle1Seq = {runKeepAngle1, 5000};
sequenceType keepAngle2Seq = {runKeepAngle2, 5000};
sequenceType fastSeq = {runFastSequentially, 5000};

bool runSequence(
  std::array<AccelStepper*, 3> circle,
  bool startNext = false,
  bool restart = false
) {
  static unsigned long prevLogMillis = 0;
  static unsigned long prevFnMillis = 0;
  static int current = 0;
  static bool advancing = false;
  if (restart) {
    current = 0;
  }
  static std::array<sequenceType, 5> sequence = {
    fastSeq,
    slowSineSeq,
    fastSineSeq,
    keepAngle1Seq,
    keepAngle2Seq
  };
  if (millis() - prevLogMillis >= 500) {
    prevLogMillis = millis();
    Serial.println("Run Sequence");
  }
  if (millis() - prevFnMillis >= sequence[current].time) {
    prevFnMillis = millis();
    current = current >= (getArrayLength(sequence) - 1) ? 0 : current + 1;
    advancing = true;
    Serial.print("current fn: ");
    Serial.println(current);
  }
  sequence[current].seqFn(circle, advancing || startNext);
  if (advancing) {
    advancing = false;
    return true;
  } else {
    return false;
  }
}

void resetCircle(std::array<AccelStepper*, 3> circle) {
  setAccelAll(circle, DEFAULT_ACCELERATION);
  setSpeedAll(circle, DEFAULT_SPEED);
  circleToZero(circle);
}

#define debugLoop 0

void loop() {
  static unsigned long prevLogMillis = 0;
  static unsigned long prevPosMillis = 0;
  static bool stopped = true;
  static bool awaitToFinishReset = false;
  if (debugLoop) {
    if (millis() - prevLogMillis >= 1000) {
      prevLogMillis = millis();
      Serial.print("UP: ");
      Serial.println(digitalRead(UP_BTN));
      Serial.print("DOWN: ");
      Serial.println(digitalRead(DOWN_BTN));
    }
  }
  
  // if (millis() - prevPosMillis >= 200) {
  //   prevPosMillis = millis();
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
    runSequence(circle1, true, true);
  }

  // Continue
  if (!stopped) {
    if (awaitToFinishReset) {
      if (debugLoop) {
        if (millis() - prevLogMillis >= 500) {
          prevLogMillis = millis();
          Serial.println("Awaiting to finish reset");
        }
      }
      if (!isCircleMoving(circle1)) {
        awaitToFinishReset = false;

        // TODO: Fix why it won't run without this call
        runSequence(circle1, true);
      }
    } else {
        if (debugLoop) {
          if (millis() - prevLogMillis >= 500) {
            prevLogMillis = millis();
            Serial.print("Continuing: ");
          }
        }
        if (runSequence(circle1)) {
          resetCircle(circle1);
          awaitToFinishReset = true;
        }
    }
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