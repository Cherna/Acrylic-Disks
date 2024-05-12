#ifndef SEQUENCES_H
#define SEQUENCES_H

#include <AccelStepper.h>
#include <array>

extern void runSine(
  std::array<AccelStepper*, 3> circle,
  bool reset = false,
  int magnitude = 500, // Vertical magnitude of the wave in steps
  double overlap = 0.33 // Fraction of the next move to overlap with the previous move
);

void runSlowSine(std::array<AccelStepper*, 3> circle, bool reset = false);

void runFastSine(std::array<AccelStepper*, 3> circle, bool reset = false);

void runFastSequentially(std::array<AccelStepper*, 3> circle, bool reset = false);

void runKeepAngle1(std::array<AccelStepper*, 3> circle, bool reset = false);

void runKeepAngle2(std::array<AccelStepper*, 3> circle, bool reset = false);

#endif