#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

class TemperatureSensor {
  public:
    // Public readable temperature variable (in Celsius)
    float temperatureC;
    
    // Constructor
    TemperatureSensor(int pin);
    
    // Method to update temperature reading
    void update();
    
  private:
    int _sensorPin;
};

#endif