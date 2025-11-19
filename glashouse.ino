// ============================================
// GLASHOUSE - Combined Arduino Sketch
// ============================================
// This file combines all modules into a single .ino file
// for easy upload using the Arduino IDE

// ============================================
// LIBRARY INCLUDES
// ============================================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================
// STEPPER CONTROL CLASS
// ============================================
class StepperControl {
  public:
    StepperControl(int pin1 = 53, int pin2 = 47, int pin3 = 51, int pin4 = 49);
    void oneRotation(bool forward = true);
    void release();

  private:
    int motorPins[4];
    const int stepsPerRevolution = 2048; // Typical for 28BYJ-48 with 1/64 reduction
    unsigned long previousStepTime = 0;
    const int stepDelay = 2; // ms between steps (controls speed)
    void stepMotor(int step, bool forward);
};

StepperControl::StepperControl(int pin1, int pin2, int pin3, int pin4) {
  motorPins[0] = pin1;
  motorPins[1] = pin2;
  motorPins[2] = pin3;
  motorPins[3] = pin4;

  for(int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
}

void StepperControl::stepMotor(int step, bool forward);

// ============================================
// STOP BUTTON CLASS
// ============================================
class Stop {
public:
    static void initialize();
    static bool isStopped_forward();
    static bool isStopped_backward();
    static void resetStop();

private:
    static volatile bool stopped_forward;
    static volatile bool stopped_backward;
    static void stopISR1();
    static void stopISR2();
};

volatile bool Stop::stopped_forward = false;
volatile bool Stop::stopped_backward = false;

void Stop::initialize() {
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

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

// ============================================
// STEPPER CONTROL IMPLEMENTATION
// ============================================
void StepperControl::oneRotation(bool forward) {
  for(int i = 0; i < stepsPerRevolution; i++) {
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
    if (micros() - previousStepTime >= stepDelay * 1000) {
      stepMotor(i % 8, forward);
      delayMicroseconds(stepDelay * 1000 - (currentMicros - previousStepTime));
      previousStepTime = micros();
    }
  }
  release();
}

void StepperControl::stepMotor(int step, bool forward) {
  const byte stepSequence[8] = {
    B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001
  };

  step = forward ? step : 7 - step;

  for(int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], bitRead(stepSequence[step], i));
  }
}

void StepperControl::release() {
  for(int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }
}

// ============================================
// TEMPERATURE SENSOR CLASS
// ============================================
class TemperatureSensor {
  public:
    float temperatureC;

    TemperatureSensor(int pin);
    void update();

  private:
    int _sensorPin;
};

TemperatureSensor::TemperatureSensor(int pin) {
  _sensorPin = pin;
  pinMode(_sensorPin, INPUT);
  temperatureC = 0.0;
}

void TemperatureSensor::update() {
  int sensorValue = analogRead(_sensorPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  temperatureC = voltage * 100.0; // LM35: 10mV per °C
}

// ============================================
// WIND SENSOR CLASS
// ============================================
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
    static WindSensor* _instance;
    float calculateSpeed(unsigned long rotations);
};

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
}

float WindSensor::calculateSpeed(unsigned long rotations) {
    const float radius = 0.1; // Radius in meters
    const float circumference = 2 * PI * radius;
    float speed = (rotations * circumference) / 1.0; // m/s
    return speed * 3.6; // Convert to km/h
}

// ============================================
// DISPLAY CONTROL - LCD & VARIABLES
// ============================================
#define LCD_COLS 20
#define LCD_ROWS 4

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

int tempCurrent = 25;
int tempTarget = 0;
int openPercent = 0;
int openTarget = 0;
int frostCurrent = 0;
int frostTarget = 0;
int windCurrent = 0;
int windTarget = 0;

int currentLine = 0; // 0: Temp, 1: Open, 2: Frost, 3: Wind

bool cyclePressed = false;
bool incPressed = false;
bool decPressed = false;

// ============================================
// DISPLAY CONTROL - FUNCTIONS
// ============================================
void scanI2CDevices() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void printWithHighlight(int value, int col, int row, bool highlight) {
  lcd.setCursor(col, row);

  if (highlight) {
    lcd.setCursor(14, row);
    lcd.write(byte(1)); // Custom [

    lcd.setCursor(15, row);
    if (value < 10) {
      lcd.print("0");
    }
    lcd.print(value);

    lcd.setCursor(17, row);
    lcd.write(byte(2)); // Custom ]
  } else {
    lcd.setCursor(15, row);
    if (value < 10) lcd.print("0");
    lcd.print(value);
  }
}

void updateDisplay() {
  lcd.clear();

  // Temperature line
  lcd.setCursor(0, 0);
  lcd.print("Temp  ");
  if (tempCurrent < 10) lcd.print(" ");
  lcd.print(tempCurrent);
  lcd.write(223);
  lcd.print("C  -> ");
  printWithHighlight(tempTarget, 15, 0, currentLine == 0);
  lcd.write(223);
  lcd.print("C");

  // Open line
  lcd.setCursor(0, 1);
  lcd.print("Offen ");
  if (openPercent < 10) lcd.print("0");
  lcd.print(openPercent);
  lcd.print("%   -> ");
  printWithHighlight(openTarget, 15, 1, currentLine == 1);
  lcd.print("%");

  // Frost line
  lcd.setCursor(0, 2);
  lcd.print("Frost ");
  if (frostCurrent < 10) lcd.print("0");
  lcd.print(frostCurrent);
  lcd.write(223);
  lcd.print("C  -> ");
  printWithHighlight(frostTarget, 15, 2, currentLine == 2);
  lcd.write(223);
  lcd.print("C");

  // Wind line
  lcd.setCursor(0, 3);
  lcd.print("Wind  ");
  if (windCurrent < 10) lcd.print("0");
  lcd.print(windCurrent);
  lcd.print("kmH -> ");
  printWithHighlight(windTarget, 15, 3, currentLine == 3);
  lcd.print("kmH");
}

void initializeDisplay() {
  Wire.begin();
  scanI2CDevices();

  lcd.init();
  lcd.backlight();

  byte leftBracket[8] = {
    B01111,
    B01100,
    B01100,
    B01100,
    B01100,
    B01100,
    B01111,
    B00000
  };
  byte rightBracket[8] = {
    B11110,
    B00110,
    B00110,
    B00110,
    B00110,
    B00110,
    B11110,
    B00000
  };

  lcd.createChar(1, leftBracket);
  lcd.createChar(2, rightBracket);
  updateDisplay();
}

void handleButtons(int cyclePin, int increasePin, int decreasePin,
                   int increment_1, int increment_2,
                   int increment_3, int increment_4) {
  static unsigned long lastCycleTime = 0;
  static unsigned long lastIncreaseTime = 0;
  static unsigned long lastDecreaseTime = 0;
  static const unsigned long debounceDelay = 20;

  if (digitalRead(cyclePin) == HIGH) {
    if (!cyclePressed && millis() - lastCycleTime > debounceDelay) {
      cyclePressed = true;
      lastCycleTime = millis();
      currentLine = (currentLine + 1) % 4;
      updateDisplay();
    }
  } else {
    cyclePressed = false;
  }

  if (digitalRead(increasePin) == HIGH) {
    if (!incPressed && millis() - lastIncreaseTime > debounceDelay) {
      incPressed = true;
      lastIncreaseTime = millis();
      switch(currentLine) {
        case 0: tempTarget += increment_1; break;
        case 1: openTarget += increment_2; break;
        case 2: frostTarget += increment_3; break;
        case 3: windTarget += increment_4; break;
      }
      updateDisplay();
    }
  } else {
    incPressed = false;
  }

  if (digitalRead(decreasePin) == HIGH) {
    if (!decPressed && millis() - lastDecreaseTime > debounceDelay) {
      decPressed = true;
      lastDecreaseTime = millis();
      switch(currentLine) {
        case 0: tempTarget = max(0, tempTarget - increment_1); break;
        case 1: openTarget = max(0, openTarget - increment_2); break;
        case 2: frostTarget = max(0, frostTarget - increment_3); break;
        case 3: windTarget = max(0, windTarget - increment_4); break;
      }
      updateDisplay();
    }
  } else {
    decPressed = false;
  }
}

// ============================================
// GLOBAL VARIABLES & SENSOR INSTANCES
// ============================================
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

// ============================================
// SETUP FUNCTION
// ============================================
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

// ============================================
// LOOP FUNCTION
// ============================================
void loop() {
  tempSensor.update();
  tempCurrent = tempSensor.temperatureC;

  // Wind sensor logic
  float currentSpeed = windSensor.getSpeed();

  if (windSensor.isSpeedCritical()) {
    windSensor.emergencyStop();
  }

  static unsigned long lastReset = 0;
  if (millis() - lastReset >= 1000) {
      windSensor.resetCount();
      lastReset = millis();
  }

  handleButtons(CYCLE_PIN, INCREASE_PIN, DECREASE_PIN,
                TEMPERATURE_INCREMENT_VALUE, OPEN_INCREMENT_VALUE,
                FROST_INCREMENT_VALUE, WIND_INCREMENT_VALUE);

  if (tempCurrent > tempTarget){
    stepper->oneRotation(true);
  }
}
