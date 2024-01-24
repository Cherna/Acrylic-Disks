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

unsigned long previousMillis = 0;
const long interval = 500;

void setup() {
  // Set up the serial communication
  Serial.begin(115200);

  // Enable motors
  pinMode(MOTORS_ENABLE, OUTPUT);
  digitalWrite(MOTORS_ENABLE, LOW);


  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : steppers) {
    motor.setMaxSpeed(800 * MICSTEP);
    motor.setAcceleration(2000 * MICSTEP);
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
    if (motor.distanceToGo() != 0) {
      return true;
    }
  }
  return false;
}

// void runSine(
//   int magnitude = 200, // Vertical magnitude of the wave
//   float overlap = 0.4 // Fraction of the wave to overlap with the next wave
// ) {
//   int direction = direction || 1;
//   int currentMove = currentMove || 0;
//   int nextMove = currentMove == MOTOR_COUNT - 1 ? 0 : currentMove + 1;

//   if (!anyMotorMoving()) {
//     steppers[0].move(magnitude * MICSTEP * direction);
//   } else {
//     if (abs(steppers[currentMove].distanceToGo()) <= magnitude * MICSTEP * overlap) {
//       steppers[nextMove].move(magnitude * MICSTEP * direction);
//       // Advance currentMove or reset to 0 if we've reached the end
//       currentMove = currentMove == MOTOR_COUNT - 1 ? 0 : currentMove + 1;
//     }
//   }
// }

void runSine(
  int magnitude = 200 // Vertical magnitude of the wave
) {
  unsigned long currentMillis = millis();
  int direction = direction || 1;
  
  if (!stepper1.isRunning()) {
    if (abs(stepper2.distanceToGo()) <= magnitude * MICSTEP * 0.3) {

    }
    stepper1.move(magnitude * MICSTEP * direction);
  }
  if (currentMillis - previousMillis >= 1000) {
    stepper2.move(magnitude * MICSTEP * direction);
    previousMillis = currentMillis;
  }
  if (currentMillis - previousMillis >= 1000) {
    stepper3.move(magnitude * MICSTEP * direction);
    previousMillis = currentMillis;
  }
}

void runSequentially() {
  int counter = counter == MOTOR_COUNT - 1 ? 0 : counter;
  int direction = direction || 1;
  if (anyMotorMoving()) {
    // Stop all motors
    stopAllMotors();
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
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