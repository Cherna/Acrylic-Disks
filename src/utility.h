#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <AccelStepper.h>
#include <array>

extern const int MICSTEP;
extern const int DEFAULT_SPEED;
extern const int DEFAULT_ACCELERATION;

void circleToZero(std::array<AccelStepper*, 3> circle);
bool isCircleMoving(std::array<AccelStepper*, 3> circle);
void allMotorsDown(std::array<AccelStepper*, 3> circle);
void allMotorsUp(std::array<AccelStepper*, 3> circle);
int angleToSteps(int angle);
void moveCircleToAngle(
  std::array<AccelStepper*, 3> circle,
  double angle = 0, // Angle in degrees
  int direction = 0,
  double otherAngle = 0,
  int otherDirection = 0
);
void normalizeMotorsHeight(std::array<AccelStepper*, 3> circle);
void setSpeedAll(
  std::array<AccelStepper*, 3> circle,
  int speed = DEFAULT_SPEED
);

void setAccelAll(
  std::array<AccelStepper*, 3> circle,
  int accel = DEFAULT_ACCELERATION
);

#endif