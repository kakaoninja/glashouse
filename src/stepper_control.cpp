#include <Arduino.h>
#include "stepper_control.h"
#include "stop_button.h"

StepperControl::StepperControl(int stepperAPlus, int stepperAMinus,
                               int stepperBPlus, int stepperBMinus) {
  // Store pin numbers for release function
  motorPins[0] = stepperAPlus;
  motorPins[1] = stepperAMinus;
  motorPins[2] = stepperBPlus;
  motorPins[3] = stepperBMinus;

  // Initialize Stepper library with pin order: A+, A-, B+, B-
  stepper = new Stepper(stepsPerRevolution, stepperAPlus, stepperAMinus,
                        stepperBPlus, stepperBMinus);

  // Set default speed (RPM)
  stepper->setSpeed(10);  // Adjust as needed for your motor

  // Initialize position tracking
  currentPosition = 0;
  maxSteps = stepsPerRevolution * 10;  // Adjust based on your window mechanism
}

void StepperControl::oneRotation(bool forward) {
  int steps = forward ? stepsPerRevolution : -stepsPerRevolution;
  moveSteps(stepsPerRevolution, forward);
}

void StepperControl::moveSteps(int steps, bool forward) {
  int actualSteps = forward ? steps : -steps;
  int stepsMoved = 0;

  // Move in smaller increments to check stop buttons frequently
  int stepIncrement = 10;  // Check every 10 steps

  while (stepsMoved < steps) {
    // Check stop buttons
    if (Stop::isStopped_forward() && forward) {
      release();
      return;
    }

    if (Stop::isStopped_backward() && !forward) {
      release();
      return;
    }

    // Move a small increment
    int stepsToMove = min(stepIncrement, steps - stepsMoved);
    int actualMove = forward ? stepsToMove : -stepsToMove;
    stepper->step(actualMove);
    stepsMoved += stepsToMove;

    // Update position tracking
    currentPosition += actualMove;
  }

  release();  // Cut power when idle (no holding force needed)
}

void StepperControl::openWindow(int percentage) {
  // Calculate steps needed for the specified percentage
  // This function is ready for future implementation
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

  int targetSteps = (maxSteps * percentage) / 100;
  int stepsToMove = targetSteps - currentPosition;

  if (stepsToMove > 0) {
    moveSteps(stepsToMove, true);
  } else if (stepsToMove < 0) {
    moveSteps(-stepsToMove, false);
  }
}

void StepperControl::closeWindow(int percentage) {
  // Close by the specified percentage
  // 100% close means fully closed, 50% means close halfway from current position
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

  int stepsToMove = (currentPosition * percentage) / 100;

  if (stepsToMove > 0) {
    moveSteps(stepsToMove, false);
  }
}

void StepperControl::setSpeed(int rpm) {
  stepper->setSpeed(rpm);
}

// Cut power to all motor coils (eliminates vibrations)
void StepperControl::release() {
  for(int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
}