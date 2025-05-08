// StepperControl.h
#ifndef STEPPERCONTROL_H
#define STEPPERCONTROL_H

#include <Arduino.h>

class StepperControl {
  public:
    StepperControl(int pin1 = 53, int pin2 = 47, int pin3 = 51, int pin4 = 49);
    void oneRotation(bool forward = true);
    void release();
    
  private:
    int motorPins[4];
    const int stepsPerRevolution = 2048; // Typical for 28BYJ-48 with 1/64 reduction
    const int stepDelay = 2; // ms between steps (controls speed)
    void stepMotor(int step, bool forward);
};

#endif