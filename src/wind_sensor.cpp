#include "wind_sensor.h"

WindSensor* WindSensor::_instance = nullptr;

WindSensor::WindSensor(int interruptPin) : _interruptPin(interruptPin), _rotationCount(0) {
    _instance = this;
}

void WindSensor::begin() {
    pinMode(_interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_interruptPin), []() {
        if (_instance) _instance->_rotationCount++;
    }, FALLING);
}

float WindSensor::getSpeed() {
    return calculateSpeed(_rotationCount);
}

void WindSensor::resetCount() {
    noInterrupts();
    _rotationCount = 0;
    interrupts();
}

bool WindSensor::isSpeedCritical() {
    return getSpeed() > 30.0; // 30 km/h threshold
}

void WindSensor::emergencyStop() {
    Serial.println("! WIND ZU SCHNELL - ABSCHALTUNG !");
    // Add emergency stop actions here
}

float WindSensor::calculateSpeed(unsigned long rotations) {
    const float radius = 0.1; // Radius in meters
    const float circumference = 2 * PI * radius;
    float speed = (rotations * circumference) / 1.0; // m/s (simplified)
    return speed * 3.6; // Convert to km/h
}