#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <Arduino.h>

class WindSensor {
public:
    WindSensor(int interruptPin);
    void begin();
    float getSpeed();
    void resetCount();
    bool isSpeedCritical();
    void emergencyStop();

private:
    int _interruptPin;
    volatile unsigned long _rotationCount;
    static void countRotation();
    static WindSensor* _instance; // For ISR access
    float calculateSpeed(unsigned long rotations);
};

#endif