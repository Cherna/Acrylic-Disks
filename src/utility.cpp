#include "utility.h"

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

void stopAllMotors(std::array<AccelStepper*, 3> circle) {
  for (auto &motor : circle) {
    motor->stop();
  }
}

void allMotorsDown(std::array<AccelStepper*, 3> circle) {
  for (auto motor : circle) {
    motor->move(200 * MICSTEP);
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

// Function to hold specific angle
void moveCircleToAngle(
  std::array<AccelStepper*, 3> circle,
  double angle, // Angle in degrees
  int direction,
  double otherAngle,
  int otherDirection
) {
  if (angle) {
    circle[direction]->moveTo(angleToSteps(angle) * MICSTEP);
  }
  if (otherAngle) {
    circle[otherDirection]->moveTo(angleToSteps(otherAngle) * MICSTEP);
  }
}

// TODO: Function to move a circle to a "neutral" horizontal position
void normalizeMotorsHeight(std::array<AccelStepper*, 3> circle) {
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

void setSpeedAll(
  std::array<AccelStepper*, 3> circle,
  int speed
) {
  for (auto &motor : circle) {
    motor->setMaxSpeed(speed * MICSTEP);
  }
}

void setAccelAll(
  std::array<AccelStepper*, 3> circle,
  int accel
) {
  for (auto &motor : circle) {
    motor->setAcceleration(accel * MICSTEP);
  }
}

// Function to move entire circle up
void allMotorsUp(std::array<AccelStepper*, 3> circle) {
  for (auto motor : circle) {
    motor->move(-200 * MICSTEP);
  }
}