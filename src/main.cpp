#include <Arduino.h>
#include <AccelStepper.h>
#include <array>

#include <utility.h>
#include <sequences.h>
#include <PCF8574.h>

// Define stepper motor connections and steps per revolution
#define MOTOR1_STEP_PIN 12
#define MOTOR1_DIR_PIN 13
#define MOTOR2_STEP_PIN 14
#define MOTOR2_DIR_PIN 15
#define MOTOR3_STEP_PIN 16
#define MOTOR3_DIR_PIN 17
#define MOTOR4_STEP_PIN 18
#define MOTOR4_DIR_PIN 19
#define MOTOR5_STEP_PIN 23
#define MOTOR5_DIR_PIN 2
#define MOTOR6_STEP_PIN 0
#define MOTOR6_DIR_PIN 5

// Default I2C pins on ESP32s
#define I2C_SDA_LINE 21
#define I2C_SCL_LINE 22

// Endstop pins
// They are very important, so using native pins on the mcu is ideal
#define ENDSTOP_1 4
#define ENDSTOP_2 25
#define ENDSTOP_3 26
#define ENDSTOP_4 27
#define ENDSTOP_5 32
#define ENDSTOP_6 33

// These pins are not ideal,
// they should ideally be one of the PCF8574 boards since they are not that imporant
// They would not be used after the piece is installed, unless maybe for maintenance
#define MOTOR0_CTRL 27
#define MOTOR1_CTRL 32
#define MOTOR2_CTRL 33
#define MOTOR_INVERT 23

// Enable pins are in PCF8574 expander pinset
/*
  One important caveat is that the PCF8574 pins can't source enough power,
  but they can sink it.
  So they need to be wired in reverse of the other inputs of the stepper drivers
  Example: The P0 pin of the pcf8574 would be connected to EN- of the driver
  and EN+ would be wired to 3.3v in the mcu. If wired as the step and dir pins
  (signal pin to STEP+ and DIR+) it would not work.
*/
#define MOTOR1_EN P0
#define MOTOR2_EN P1
#define MOTOR3_EN P2
#define MOTOR4_EN P3
#define MOTOR5_EN P4
#define MOTOR6_EN P5

// Missing pins
// Circle1LEDs
// Circle2LEDs

#define ENABLE_CONTROLS 0

PCF8574 pcf8574_1(0x20);

#define START_BTN 34 // Needs external PULLDOWN resistor on ESP32s
#define STOP_BTN 35 // Needs external PULLDOWN resistor on ESP32s

const int DEFAULT_SPEED = 600;
const int DEFAULT_ACCELERATION = 1000;

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
  pinMode(START_BTN, INPUT); // Can't be set as PULLDOWN, needs external pulldown resistor
  pinMode(STOP_BTN, INPUT); // Can't be set as PULLDOWN, needs external pulldown resistor

  // Set up enable pins
  pinMode(ENDSTOP_1, INPUT_PULLDOWN);
  pinMode(ENDSTOP_2, INPUT_PULLDOWN);
  pinMode(ENDSTOP_3, INPUT_PULLDOWN);
  pinMode(ENDSTOP_4, INPUT_PULLDOWN);
  pinMode(ENDSTOP_5, INPUT_PULLDOWN);
  pinMode(ENDSTOP_6, INPUT_PULLDOWN);

  // Set up Motor Outputs
  pcf8574_1.pinMode(MOTOR1_EN, OUTPUT);
  pcf8574_1.pinMode(MOTOR2_EN, OUTPUT);
  pcf8574_1.pinMode(MOTOR3_EN, OUTPUT);
  pcf8574_1.pinMode(MOTOR4_EN, OUTPUT);
  pcf8574_1.pinMode(MOTOR5_EN, OUTPUT);
  pcf8574_1.pinMode(MOTOR6_EN, OUTPUT);
  
  pcf8574_1.begin();
  // Disable all motors by default
  pcf8574_1.digitalWrite(MOTOR1_EN, LOW);
  pcf8574_1.digitalWrite(MOTOR2_EN, LOW);
  pcf8574_1.digitalWrite(MOTOR3_EN, LOW);
  pcf8574_1.digitalWrite(MOTOR4_EN, LOW);
  pcf8574_1.digitalWrite(MOTOR5_EN, LOW);
  pcf8574_1.digitalWrite(MOTOR6_EN, LOW);

  if (ENABLE_CONTROLS) {
    pinMode(MOTOR0_CTRL, INPUT_PULLDOWN);
    pinMode(MOTOR1_CTRL, INPUT_PULLDOWN);
    pinMode(MOTOR2_CTRL, INPUT_PULLDOWN);
    pinMode(MOTOR_INVERT, INPUT_PULLDOWN);
  }

  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : allSteppers) {
    motor->setMaxSpeed(DEFAULT_SPEED * MICSTEP);
    motor->setAcceleration(DEFAULT_ACCELERATION * MICSTEP);
  }
}

static bool stepper1Homed = false;
static bool stepper2Homed = false;
static bool stepper3Homed = false;
static bool stepper4Homed = false;
static bool stepper5Homed = false;
static bool stepper6Homed = false;

// Returns true if circle is homed
bool homeCircle(std::array<AccelStepper*, 3> circle) {
  setSpeedAll(circle, 1000);
  setAccelAll(circle, 1000);
  if (circle == circle1) {
    if (digitalRead(ENDSTOP_1)) {
      stepper1Homed = true;
      stepper1.stop();
      stepper1.setCurrentPosition(0);
    }
    if (digitalRead(ENDSTOP_2)) {
      stepper2Homed = true;
      stepper2.stop();
      stepper2.setCurrentPosition(0);
    }
    if (digitalRead(ENDSTOP_3)) {
      stepper3Homed = true;
      stepper3.stop();
      stepper3.setCurrentPosition(0);
    }
    if (!stepper1Homed) {
      stepper1.move(50 * MICSTEP);
    };
    if (!stepper2Homed) {
      stepper2.move(50 * MICSTEP);
    }
    if (!stepper3Homed) {
      stepper3.move(50 * MICSTEP);
    }
  } else {
    if (digitalRead(ENDSTOP_4)) {
      stepper4Homed = true;
      stepper4.stop();
      stepper4.setCurrentPosition(0);
    }
    if (digitalRead(ENDSTOP_5)) {
      stepper5Homed = true;
      stepper5.stop();
      stepper5.setCurrentPosition(0);
    }
    if (digitalRead(ENDSTOP_6)) {
      stepper6Homed = true;
      stepper6.stop();
      stepper6.setCurrentPosition(0);
    }
    if (!stepper4Homed) {
      stepper4.move(50 * MICSTEP);
    };
    if (!stepper5Homed) {
      stepper5.move(50 * MICSTEP);
    }
    if (!stepper6Homed) {
      stepper6.move(50 * MICSTEP);
    }
  }

  return circle == circle1
    ? stepper1Homed && stepper2Homed && stepper3Homed
    : stepper4Homed && stepper5Homed && stepper6Homed;
}

void toggleCircle(std::array<AccelStepper*, 3> circle, bool enable) {
  if (circle == circle1) {
    pcf8574_1.digitalWrite(MOTOR1_EN, enable);
    pcf8574_1.digitalWrite(MOTOR2_EN, enable);
    pcf8574_1.digitalWrite(MOTOR3_EN, enable);
  } else {
    pcf8574_1.digitalWrite(MOTOR4_EN, enable);
    pcf8574_1.digitalWrite(MOTOR5_EN, enable);
    pcf8574_1.digitalWrite(MOTOR6_EN, enable);
  }
}

// Returns true if circle is enabled
// TODO: Maybe steppers should be enabled separately and not by entire circle
bool isCircleEnabled(std::array<AccelStepper*, 3> circle) {
  if (circle == circle1) {
    return pcf8574_1.digitalRead(MOTOR1_EN);
  } else {
    return pcf8574_1.digitalRead(MOTOR4_EN);
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
  int time; // Sequence duration in milliseconds
};

sequenceType slowSineSeq = {runSlowSine, 30000};
sequenceType fastSineSeq = {runFastSine, 30000};
sequenceType keepAngle1Seq = {runKeepAngle1, 15000};
sequenceType keepAngle2Seq = {runKeepAngle2, 15000};
sequenceType fastSeq = {runFastSequentially, 15000};

// Runs the current sequence selected in order from a list of [sequenceType]s
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

void runControls() {
  if (digitalRead(MOTOR0_CTRL)) {
    if (digitalRead(MOTOR_INVERT)) {
      stepper1.move(50 * MICSTEP);
    } else {
      stepper1.move(-50 * MICSTEP);
    }
  }
  if (digitalRead(MOTOR1_CTRL)) {
    if (digitalRead(MOTOR_INVERT)) {
      stepper2.move(50 * MICSTEP);
    } else {
      stepper2.move(-50 * MICSTEP);
    }
  }
  if (digitalRead(MOTOR2_CTRL)) {
    if (digitalRead(MOTOR_INVERT)) {
      stepper3.move(50 * MICSTEP);
    } else {
      stepper3.move(-50 * MICSTEP);
    }
  }
}

#define debugLoop 0

void loop() {
  // TODO: Maybe some of these flags should be moved to outside of the loop()
  // instead of being defined as static
  static unsigned long prevLogMillis = 0;
  static unsigned long prevPosMillis = 0;
  static bool stopped = true;
  static bool awaitToFinishReset = false;
  static bool circle1Homed = false;
  static bool circle2Homed = false;
  static bool circle1IsHoming = false;
  static bool circle2IsHoming = false;

  if (debugLoop) {
    if (millis() - prevLogMillis >= 200) {
      prevLogMillis = millis();
      Serial.print("UP: ");
      Serial.println(digitalRead(START_BTN));
      Serial.print("DOWN: ");
      Serial.println(digitalRead(STOP_BTN));

      Serial.print("1: ");
      Serial.print(digitalRead(ENDSTOP_1));
      Serial.print(" - ");
      Serial.println(pcf8574_1.digitalRead(MOTOR1_EN));
      Serial.print("2: ");
      Serial.print(digitalRead(ENDSTOP_2));
      Serial.print(" - ");
      Serial.println(pcf8574_1.digitalRead(MOTOR2_EN));
      Serial.print("3: ");
      Serial.print(digitalRead(ENDSTOP_3));
      Serial.print(" - ");
      Serial.println(pcf8574_1.digitalRead(MOTOR3_EN));

      // Serial.print("homed: ");
      // Serial.println(circle1Homed);
      // Serial.print("homing?: ");
      // Serial.println(circle1IsHoming);
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

  // if (digitalRead(START_BTN)) {
  //   allMotorsUp(circle1);
  // } else if (digitalRead(STOP_BTN)) {
  //   allMotorsDown(circle1);
  // }

  if (ENABLE_CONTROLS) {
    runControls();
  }

  // Stop
  if (digitalRead(STOP_BTN)) {
    if (debugLoop) {
      Serial.println("STOP"); 
    }
    stopped = true;
    resetCircle(circle1);
  }

  // Start
  if (digitalRead(START_BTN)) {
    if (debugLoop) {
      Serial.println("START");
    }
    if (circle1Homed) {
      setSpeedAll(circle1, DEFAULT_SPEED);
      stopped = false;
      runSequence(circle1, true, true);
    } else {
      circle1IsHoming = true;
      homeCircle(circle1);
    }
  }

  if (circle1IsHoming) {
    if (homeCircle(circle1)) {
      circle1IsHoming = false;
      circle1Homed = true;
      setSpeedAll(circle1, DEFAULT_SPEED);
      setAccelAll(circle1, DEFAULT_ACCELERATION);
    }
  }
  if (circle2IsHoming) {
    if (homeCircle(circle2)) {
      circle2IsHoming = false;
      circle2Homed = true;
      setSpeedAll(circle2, DEFAULT_SPEED);
      setAccelAll(circle2, DEFAULT_ACCELERATION);
    }
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