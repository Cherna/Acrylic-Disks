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

int direction = 1; // 1 for forward, -1 for backward
unsigned long previousMillis = 0;
const long interval = 500;

void setup()
{
  // Set up the serial communication
  Serial.begin(115200);

  pinMode(MOTORS_ENABLE, OUTPUT);
  digitalWrite(MOTORS_ENABLE, LOW);


  // Set the maximum speed and acceleration for each stepper motor
  for (auto &motor : steppers)
  {
    motor.setMaxSpeed(800 * MICSTEP);
    motor.setAcceleration(2000 * MICSTEP);
  }
}

bool anyMotorRunning;
int counter = 0;

void loop()
{

  anyMotorRunning = false;
  for (auto &motor : steppers)
  {
    anyMotorRunning = anyMotorRunning || motor.run();
  }

  if (!anyMotorRunning)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      Serial.print("Moving: ");
      Serial.println(counter);
      steppers[counter].move(200 * MICSTEP * direction);

      // Increase motor counter
      ++counter;

      // Reverse the direction after all motors have run
      if (counter >= MOTOR_COUNT)
      {
        Serial.println("Resetting counter and reversing");
        counter = 0;
        direction *= -1;
      }

      // Reset the timer for the next interval
      previousMillis = currentMillis;
    }
  }

  for (auto &motor : steppers)
  {
    motor.run();
  }
}