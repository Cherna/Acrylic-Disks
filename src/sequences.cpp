#include "sequences.h"
#include "utility.h"

void runSlowSine(std::array<AccelStepper*, 3> circle, bool reset) {
  if (reset) {
    setAccelAll(circle, 40);
    setSpeedAll(circle, 120);
    runSine(circle, reset, 1000, 0.66);
  } else {
    runSine(circle, reset, 1000, 0.66);
  }
}

void runFastSine(std::array<AccelStepper*, 3> circle, bool reset) {
  if (reset) {
    setAccelAll(circle, 100);
    setSpeedAll(circle, 200);
    runSine(circle, reset, 800, 0.5);
  } else {
    runSine(circle, reset, 800, 0.5);
  }
}

void runFastSequentially(std::array<AccelStepper*, 3> circle, bool reset) {
  if (reset) {
    setAccelAll(circle, 600);
    setSpeedAll(circle, 300);
    runSine(circle, reset, 100, 0.01);
  } else {
    runSine(circle, reset, 100, 0.01);
  }
}

void runKeepAngle1(std::array<AccelStepper*, 3> circle, bool reset) {
  if (reset) {
    setAccelAll(circle, DEFAULT_ACCELERATION);
    setSpeedAll(circle, DEFAULT_SPEED);
    moveCircleToAngle(circle, 2, 0);
  }
}

void runKeepAngle2(std::array<AccelStepper*, 3> circle, bool reset) {
  if (reset) {
    setAccelAll(circle, DEFAULT_ACCELERATION);
    setSpeedAll(circle, DEFAULT_SPEED);
    moveCircleToAngle(circle, 1.5, 2, 2, 1);
  }
}