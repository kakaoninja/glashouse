#include <Arduino.h>
#include "stepper_control.h"
#include "stop_button.h"


StepperControl::StepperControl(int pin1, int pin2, int pin3, int pin4) {
  motorPins[0] = pin1;
  motorPins[1] = pin2;
  motorPins[2] = pin3;
  motorPins[3] = pin4;
  
  for(int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
}

void StepperControl::oneRotation(bool forward) {
  for(int i = 0; i < stepsPerRevolution; i++) {
    // check if stop buttons are pressed
    // maybe need to write !Stop (not stop) might be wrong

    if (Stop::isStopped_forward()){
      if(forward){
        release();
        return;
      }
    }

    if (Stop::isStopped_backward()){
      if(!forward){
        release();
        return;
      }
    }
    unsigned long currentMicros = micros();
    //end check stop buttons
    if (micros() - previousStepTime >= stepDelay * 1000) {
      stepMotor(i % 8, forward);
      delayMicroseconds(stepDelay * 1000 - (currentMicros - previousStepTime));
      previousStepTime = micros(); // Reset timer
    }
  }
  release(); // cut power when idle (no holding force needed)
}

void StepperControl::stepMotor(int step, bool forward) {
  // 8-step sequence for smooth operation
  const byte stepSequence[8] = {
    B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001
  };

  step = forward ? step : 7 - step; // Reverse step order if !forward
  
  for(int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], bitRead(stepSequence[step], i));
  }
}

// Cut power to all motor coils (eliminates vibrations)
void StepperControl::release() {
  for(int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }
}