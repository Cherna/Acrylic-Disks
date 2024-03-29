#include <Arduino.h>
#include <AccelStepper.h>

// Define stepper motor connections and steps per revolution
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR3_STEP_PIN 4
#define MOTOR3_DIR_PIN 7
#define MOTORS_ENABLE 8
#define MICSTEP 2 // Microstepping set on the driver

#define MOTOR_COUNT 3

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

AccelStepper steppers[MOTOR_COUNT] = {stepper1, stepper2, stepper3};

void setup() {
  // Set up the serial communication
  Serial.begin(115200);

  // Enable motors
  pinMode(MOTORS_ENABLE, OUTPUT);
  digitalWrite(MOTORS_ENABLE, LOW);


  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : steppers) {
    motor.setMaxSpeed(4000 * MICSTEP);
    motor.setAcceleration(4000 * MICSTEP);
  }
}

// TODO: this should move the motors to their "base" position
void stopAllMotors() {
  for (auto &motor : steppers) {
    motor.stop();
  }
}

bool anyMotorMoving() {
  for (auto &motor : steppers) {
    if (motor.isRunning()) {
      return true;
    }
  }
  return false;
}


// void runInternals(AccelStepper &motor, int i, int DIST) {
//   if (!motor.isRunning()) {
//     Serial.print(i);
//     Serial.println(" - Motor is not running, proceeding. Current pos: ");
//     Serial.println(motor.currentPosition());

//     if (motor.currentPosition() >= DIST || motor.currentPosition() == 0) {
//       Serial.println(" - Running to negative " + String(DIST));
//       motor.move(-DIST);
//     } else if (motor.currentPosition() <= -DIST) {
//       Serial.println("Running to positive " + String(DIST));
//       motor.move(DIST);
//     }
//   }
// }

// void runSine(
//   int magnitude = 300 // Vertical magnitude of the wave
// ) {
//   for (auto &motor : steppers) {
//     motor.setMaxSpeed(50 * MICSTEP);
//   }
//   unsigned long currentMillis = millis();
//   unsigned long previousMillis = previousMillis || 0;
//   const int DIST = magnitude * MICSTEP;
//   for (int i = 0; i < MOTOR_COUNT; ++i) {
//     AccelStepper &motor = steppers[i];
//     if (i == 0) {
//       runInternals(motor, i, DIST);
//     } else {
//       if (i == 1) {
//         if (currentMillis - previousMillis >= 1500) {
//           runInternals(motor, i, DIST);
//         }
//       }
//       if (i == 2) {
//         if (currentMillis - previousMillis >= 3000) {
//           runInternals(motor, i, DIST);
//           previousMillis = currentMillis;
//         }
//       }
//     }
//   }
// }


int currentMove = 0;
int nextMove = 1;
void runSine(
  int magnitude = 200, // Vertical magnitude of the wave
  float overlap = 0.5 // Fraction of the wave to overlap with the next wave
) {
  int direction = direction || 1;

  for(auto &motor : steppers) {
    motor.setMaxSpeed(100 * MICSTEP);
  }

  if (!anyMotorMoving()) {
    steppers[currentMove].move(magnitude * MICSTEP * direction);
  }

  // Serial.println(abs(steppers[currentMove].distanceToGo()));
  // Serial.println(magnitude * MICSTEP * overlap);

  Serial.print("Current move: ");
  Serial.println(currentMove);
  Serial.print("Next move: ");
  Serial.println(nextMove);

  if (
    steppers[currentMove].isRunning() &&
    abs(steppers[currentMove].distanceToGo()) <= magnitude * MICSTEP * overlap
  ) {
    Serial.println("Starting next move");
    steppers[nextMove].move(magnitude * MICSTEP * direction);
    if (currentMove == MOTOR_COUNT - 1) {
      direction *= -1;
    }
    // Advance currentMove or reset to 0 if we've reached the end
    if (currentMove == (MOTOR_COUNT - 1)) {
      currentMove = 0;
    } else if (nextMove == (MOTOR_COUNT - 1)) {
      nextMove = 0;
    } else {
      currentMove++;
      nextMove++;
    }
  }
}

// void runSine(
//   int magnitude = 200 // Vertical magnitude of the wave
// ) {
//   unsigned long currentMillis = millis();
//   int direction = direction || 1;
  
//   if (!stepper1.isRunning()) {
//     if (abs(stepper2.distanceToGo()) <= magnitude * MICSTEP * 0.3) {

//     }
//     stepper1.move(magnitude * MICSTEP * direction);
//   }
//   if (currentMillis - previousMillis >= 1000) {
//     stepper2.move(magnitude * MICSTEP * direction);
//     previousMillis = currentMillis;
//   }
//   if (currentMillis - previousMillis >= 1000) {
//     stepper3.move(magnitude * MICSTEP * direction);
//     previousMillis = currentMillis;
//   }
// }

void runSequentially() {
  int counter = 0;
  counter = counter == MOTOR_COUNT - 1 ? 0 : counter;
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
      steppers[counter].move(200 * MICSTEP * direction);

      // Increase motor counter
      ++counter;

      // Reverse the direction after all motors have run
      if (counter >= MOTOR_COUNT) {
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

  for (auto &motor : steppers) {
    motor.run();
  }
}