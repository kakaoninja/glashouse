// StepperControl.h
#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <Stepper.h>

class StepperControl {
  public:
    StepperControl(int stepperAPlus = 53, int stepperAMinus = 51,
                   int stepperBPlus = 49, int stepperBMinus = 47);
    void oneRotation(bool forward = true);
    void moveSteps(int steps, bool forward = true);
    void openWindow(int percentage);
    void closeWindow(int percentage);
    void setSpeed(int rpm);
    void release();

  private:
    Stepper* stepper;
    const int stepsPerRevolution = 2048; // Typical for 28BYJ-48 with 1/64 reduction
    int motorPins[4];
    int currentPosition;  // Track current position for future use
    int maxSteps;         // Maximum steps for full open/close
};

#endif