#include "temperature_sensor.h"
#include <Arduino.h>

TemperatureSensor::TemperatureSensor(int pin) {
  _sensorPin = pin;
  pinMode(_sensorPin, INPUT);
  temperatureC = 0.0; // Initialize temperature
}

void TemperatureSensor::update() {
  // Read analog value (0-1023 for 0-5V)
  int sensorValue = analogRead(_sensorPin);
  
  // Convert to voltage (5V reference)
  float voltage = sensorValue * (5.0 / 1023.0);
  
  // Convert voltage to temperature (LM35: 10mV per °C)
  temperatureC = voltage * 100.0;
}