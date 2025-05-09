#include "stop_button.h"
#include <Arduino.h>

volatile bool Stop::stopped_forward = false;
volatile bool Stop::stopped_backward = false;

void Stop::initialize() {
    // Initialize your switch pins here (use INPUT_PULLUP if switches are connected to ground)
    pinMode(2, INPUT_PULLUP);  // First stop switch on pin 2
    pinMode(3, INPUT_PULLUP);  // Second stop switch on pin 3
    
    // Attach interrupts (FALLING for when switch is pressed, goes from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(2), stopISR1, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), stopISR2, FALLING);
}

bool Stop::isStopped_forward() {
    return stopped_forward;
}

bool Stop::isStopped_backward() {
    return stopped_backward;
}

void Stop::resetStop() {
    stopped_forward = false;
    stopped_backward = false;
}

void Stop::stopISR1() {
    stopped_forward = true;
}

void Stop::stopISR2() {
    stopped_backward = true;
}