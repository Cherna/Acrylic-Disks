#include <Arduino.h>
#include <AccelStepper.h>
#include <Ticker.h>

// Define stepper motor connections and steps per revolution
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR3_STEP_PIN 4
#define MOTOR3_DIR_PIN 7

#define MOTORS_ENABLE 8

AccelStepper stepper1(AccelStepper::FULL4WIRE, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper stepper2(AccelStepper::FULL4WIRE, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper stepper3(AccelStepper::FULL4WIRE, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

AccelStepper steppers[] = {stepper1, stepper2, stepper3};

Ticker timer;

int direction = 1; // 1 for forward, -1 for backward
unsigned long previousMillis = 0;
const long interval = 1000;

void setup()
{
  // Set up the serial communication
  Serial.begin(115200);

  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : steppers)
  {
    motor.setMaxSpeed(1000.0);
    motor.setAcceleration(500.0);
  }
}

void loop()
{
  for (auto &motor : steppers)
  {
    motor.run();
  }

  for (auto &motor : steppers)
  {
    motor.move(200 * direction);
  }

  // Check if the motors have reached the target position
  bool anyMotorRunning;
  while (anyMotorRunning)
  {
    anyMotorRunning = false;
    for (auto &motor : steppers)
    {
      anyMotorRunning = anyMotorRunning || motor.run();
    }
  }

  if (!anyMotorRunning)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      // Reverse the direction
      direction *= -1;

      // Reset the timer for the next interval
      previousMillis = currentMillis;
    }
  }
}