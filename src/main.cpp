// libraries:
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// own scripts:
#include "stepper_control.h"
#include "display_control.h"
#include "wind_sensor.h"
#include "stop_button.h"
#include "temperature_sensor.h"

const int WIND_SENSOR_PIN = 2;
WindSensor windSensor(WIND_SENSOR_PIN);

TemperatureSensor tempSensor(A0);

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
  Stop::initialize();
  windSensor.begin();


  
  pinMode(CYCLE_PIN, INPUT_PULLUP);
  pinMode(INCREASE_PIN, INPUT_PULLUP);
  pinMode(DECREASE_PIN, INPUT_PULLUP);
  
  initializeDisplay();
  tempTarget = 20;
  frostTarget = 5;
  windTarget = 50;
  currentLine = 0;
  }

void loop() {


  tempSensor.update();
  //Serial.print(tempSensor.temperatureC);
  tempCurrent = tempSensor.temperatureC;

  //windsensor
  //TODO: implement logic to turn motor until window is closed
  float currentSpeed = windSensor.getSpeed();

  if (windSensor.isSpeedCritical()) {
    windSensor.emergencyStop();
  }

  static unsigned long lastReset = 0;
  if (millis() - lastReset >= 1000) {
      windSensor.resetCount();
      lastReset = millis();
      
      // Optional: Print speed for debugging
      // Serial.print("Wind speed: ");
      // Serial.print(currentSpeed);
      // Serial.println(" km/h");
  }
  //end windsensor


  handleButtons(CYCLE_PIN, INCREASE_PIN, DECREASE_PIN,
                TEMPERATURE_INCREMENT_VALUE, OPEN_INCREMENT_VALUE,
                FROST_INCREMENT_VALUE, WIND_INCREMENT_VALUE);
  
  if (tempCurrent>tempTarget){
    stepper->oneRotation(true);
  }
  //
}
