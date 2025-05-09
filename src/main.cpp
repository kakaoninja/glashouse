// libraries:
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// own scripts:
#include "stepper_control.h"
#include "display_control.h"

#define CYCLE_PIN 52
#define INCREASE_PIN 50
#define DECREASE_PIN 48

#define TEMPERATURE_INCREMENT_VALUE 1
#define OPEN_INCREMENT_VALUE 5
#define FROST_INCREMENT_VALUE 1
#define WIND_INCREMENT_VALUE 10

StepperControl* stepper = new StepperControl();

//initialize stepper

void setup() {
  Serial.begin(9600);
  
  pinMode(CYCLE_PIN, INPUT_PULLUP);
  pinMode(INCREASE_PIN, INPUT_PULLUP);
  pinMode(DECREASE_PIN, INPUT_PULLUP);
  
  initializeDisplay();
  }

void loop() {
  handleButtons(CYCLE_PIN, INCREASE_PIN, DECREASE_PIN,
                TEMPERATURE_INCREMENT_VALUE, OPEN_INCREMENT_VALUE,
                FROST_INCREMENT_VALUE, WIND_INCREMENT_VALUE);
  
  //stepper->oneRotation();
}
