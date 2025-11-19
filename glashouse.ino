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
#include <Stepper.h>

// ============================================
// STEPPER CONTROL CLASS
// ============================================
class StepperControl {
  public:
    StepperControl(int stepperAPlus = 53, int stepperAMinus = 51,
                   int stepperBPlus = 49, int stepperBMinus = 47);
    void oneRotation(bool forward = true);
    void moveSteps(int steps, bool forward = true);
    void openWindow(int percentage);
    void closeWindow(int percentage);
    void setSpeed(int rpm);
    void release();

  private:
    Stepper* stepper;
    const int stepsPerRevolution = 2048; // Typical for 28BYJ-48 with 1/64 reduction
    int motorPins[4];
    int currentPosition;  // Track current position for future use
    int maxSteps;         // Maximum steps for full open/close
};

StepperControl::StepperControl(int stepperAPlus, int stepperAMinus,
                               int stepperBPlus, int stepperBMinus) {
  // Store pin numbers for release function
  motorPins[0] = stepperAPlus;
  motorPins[1] = stepperAMinus;
  motorPins[2] = stepperBPlus;
  motorPins[3] = stepperBMinus;

  // Initialize Stepper library with pin order: A+, A-, B+, B-
  stepper = new Stepper(stepsPerRevolution, stepperAPlus, stepperAMinus,
                        stepperBPlus, stepperBMinus);

  // Set default speed (RPM)
  stepper->setSpeed(10);  // Adjust as needed for your motor

  // Initialize position tracking
  currentPosition = 0;
  maxSteps = stepsPerRevolution * 10;  // Adjust based on your window mechanism
}

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
    bool isMotorBlocked();
    unsigned long getCooldownRemaining();

private:
    int _interruptPin;
    volatile unsigned long _rotationCount;
    volatile unsigned long _lastWindDetectionTime;
    static const unsigned long COOLDOWN_PERIOD = 600000; // 10 minutes in milliseconds
    static void windDetectedISR();
    static WindSensor* _instance;
    float calculateSpeed(unsigned long rotations);
};

WindSensor* WindSensor::_instance = nullptr;

// ============================================
// GLOBAL VARIABLES & CONSTANTS
// ============================================

// ============================================
// TEMPERATURE CONTROL CONSTANTS
// ============================================
// Temperature sensor pins
const int TEMP_SENSOR_INSIDE_PIN = A0;   // Analog pin for inside temperature sensor
const int TEMP_SENSOR_OUTSIDE_PIN = A1;  // Analog pin for outside temperature sensor

// Heating relay pins (active LOW - connected to ground when activated)
const int HEATING_RELAY_1_PIN = 24;      // First heating relay control pin
const int HEATING_RELAY_2_PIN = 26;      // Second heating relay control pin

// Temperature thresholds
const float FREEZING_POINT = 0.0;        // Freezing point in Celsius

// ============================================
// TEMPERATURE SENSOR INSTANCES
// ============================================
TemperatureSensor tempSensorInside(TEMP_SENSOR_INSIDE_PIN);   // Inside glashouse sensor
TemperatureSensor tempSensorOutside(TEMP_SENSOR_OUTSIDE_PIN); // Outside glashouse sensor

// ============================================
// WIND SENSOR VARIABLES
// ============================================
const int WIND_SENSOR_PIN = 21;  // Changed from pin 2 to avoid conflict with Stop button
WindSensor windSensor(WIND_SENSOR_PIN);

// ============================================
// LCD CONFIGURATION & DISPLAY VARIABLES
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
// BUTTON PIN DEFINITIONS
// ============================================
#define CYCLE_PIN 52
#define INCREASE_PIN 50
#define DECREASE_PIN 48

// ============================================
// INCREMENT VALUE DEFINITIONS
// ============================================
#define TEMPERATURE_INCREMENT_VALUE 1
#define OPEN_INCREMENT_VALUE 5
#define FROST_INCREMENT_VALUE 1
#define WIND_INCREMENT_VALUE 10

// ============================================
// STEPPER CONTROL INSTANCE
// ============================================
StepperControl* stepper = new StepperControl();

// ============================================
// STEPPER CONTROL IMPLEMENTATION
// ============================================
void StepperControl::oneRotation(bool forward) {
  int steps = forward ? stepsPerRevolution : -stepsPerRevolution;
  moveSteps(stepsPerRevolution, forward);
}

void StepperControl::moveSteps(int steps, bool forward) {
  // Check if motor is blocked by wind cooldown
  extern WindSensor windSensor;
  if (windSensor.isMotorBlocked()) {
    unsigned long remainingMs = windSensor.getCooldownRemaining();
    unsigned long remainingMin = remainingMs / 60000;
    unsigned long remainingSec = (remainingMs % 60000) / 1000;
    Serial.print("! MOTOR BLOCKED - Wind cooldown active. Remaining: ");
    Serial.print(remainingMin);
    Serial.print("m ");
    Serial.print(remainingSec);
    Serial.println("s");
    release();
    return;
  }

  int actualSteps = forward ? steps : -steps;
  int stepsMoved = 0;

  // Move in smaller increments to check stop buttons frequently
  int stepIncrement = 10;  // Check every 10 steps

  while (stepsMoved < steps) {
    // Check wind sensor during movement
    if (windSensor.isMotorBlocked()) {
      Serial.println("! MOTOR STOPPED - Wind detected during movement");
      release();
      return;
    }

    // Check stop buttons
    if (Stop::isStopped_forward() && forward) {
      release();
      return;
    }

    if (Stop::isStopped_backward() && !forward) {
      release();
      return;
    }

    // Move a small increment
    int stepsToMove = min(stepIncrement, steps - stepsMoved);
    int actualMove = forward ? stepsToMove : -stepsToMove;
    stepper->step(actualMove);
    stepsMoved += stepsToMove;

    // Update position tracking
    currentPosition += actualMove;
  }

  release();  // Cut power when idle (no holding force needed)
}

void StepperControl::openWindow(int percentage) {
  // Calculate steps needed for the specified percentage
  // This function is ready for future implementation
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

  int targetSteps = (maxSteps * percentage) / 100;
  int stepsToMove = targetSteps - currentPosition;

  if (stepsToMove > 0) {
    moveSteps(stepsToMove, true);
  } else if (stepsToMove < 0) {
    moveSteps(-stepsToMove, false);
  }
}

void StepperControl::closeWindow(int percentage) {
  // Close by the specified percentage
  // 100% close means fully closed, 50% means close halfway from current position
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;

  int stepsToMove = (currentPosition * percentage) / 100;

  if (stepsToMove > 0) {
    moveSteps(stepsToMove, false);
  }
}

void StepperControl::setSpeed(int rpm) {
  stepper->setSpeed(rpm);
}

void StepperControl::release() {
  for(int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
}

// ============================================
// TEMPERATURE SENSOR IMPLEMENTATION
// ============================================
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
// WIND SENSOR IMPLEMENTATION
// ============================================
WindSensor::WindSensor(int interruptPin) : _interruptPin(interruptPin), _rotationCount(0), _lastWindDetectionTime(0) {
    _instance = this;
}

void WindSensor::begin() {
    pinMode(_interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_interruptPin), windDetectedISR, FALLING);
}

void WindSensor::windDetectedISR() {
    if (_instance) {
        _instance->_rotationCount++;
        _instance->_lastWindDetectionTime = millis(); // Refresh cooldown timer on each detection
    }
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

bool WindSensor::isMotorBlocked() {
    // Motor is blocked if we're within the cooldown period
    if (_lastWindDetectionTime == 0) {
        return false; // No wind detected yet
    }
    unsigned long timeSinceLastWind = millis() - _lastWindDetectionTime;
    return timeSinceLastWind < COOLDOWN_PERIOD;
}

unsigned long WindSensor::getCooldownRemaining() {
    if (_lastWindDetectionTime == 0) {
        return 0; // No cooldown active
    }
    unsigned long timeSinceLastWind = millis() - _lastWindDetectionTime;
    if (timeSinceLastWind >= COOLDOWN_PERIOD) {
        return 0; // Cooldown expired
    }
    return COOLDOWN_PERIOD - timeSinceLastWind;
}

float WindSensor::calculateSpeed(unsigned long rotations) {
    const float radius = 0.1; // Radius in meters
    const float circumference = 2 * PI * radius;
    float speed = (rotations * circumference) / 1.0; // m/s
    return speed * 3.6; // Convert to km/h
}

// ============================================
// DISPLAY CONTROL FUNCTIONS
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
// TEMPERATURE CONTROL FUNCTIONS
// ============================================

/**
 * Check if inside temperature is below the target value set on display
 * @return true if inside temperature is below target, false otherwise
 */
bool isInsideTempBelowTarget() {
  return tempSensorInside.temperatureC < tempTarget;
}

/**
 * Check if outside temperature is below freezing point (0°C)
 * @return true if outside temperature is below 0°C, false otherwise
 */
bool isOutsideTempBelowFreezing() {
  return tempSensorOutside.temperatureC < FREEZING_POINT;
}

/**
 * Control heating relays based on temperature conditions
 * Activates both heating relays when:
 * - Outside temperature is below 0°C AND
 * - Inside temperature is below the target value set on display
 *
 * Relays are active LOW (connected to ground when heating is needed)
 */
void controlHeatingRelays() {
  // Check if both conditions are met for heating
  if (isOutsideTempBelowFreezing() && isInsideTempBelowTarget()) {
    // Activate both relays (connect to ground)
    digitalWrite(HEATING_RELAY_1_PIN, LOW);
    digitalWrite(HEATING_RELAY_2_PIN, LOW);
  } else {
    // Deactivate both relays (disconnect from ground)
    digitalWrite(HEATING_RELAY_1_PIN, HIGH);
    digitalWrite(HEATING_RELAY_2_PIN, HIGH);
  }
}

// ============================================
// SETUP FUNCTION
// ============================================
void setup() {
  Serial.begin(9600);
  Stop::initialize();
  windSensor.begin();

  // Initialize button pins
  pinMode(CYCLE_PIN, INPUT_PULLUP);
  pinMode(INCREASE_PIN, INPUT_PULLUP);
  pinMode(DECREASE_PIN, INPUT_PULLUP);

  // Initialize heating relay pins as outputs (start with relays off - HIGH)
  pinMode(HEATING_RELAY_1_PIN, OUTPUT);
  pinMode(HEATING_RELAY_2_PIN, OUTPUT);
  digitalWrite(HEATING_RELAY_1_PIN, HIGH);  // Relays inactive at startup
  digitalWrite(HEATING_RELAY_2_PIN, HIGH);  // Relays inactive at startup

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
  // Update temperature sensors
  tempSensorInside.update();
  tempSensorOutside.update();
  tempCurrent = tempSensorInside.temperatureC;

  // Control heating relays based on temperature conditions
  controlHeatingRelays();

  // Wind sensor logic - reset rotation count every second to calculate current speed
  static unsigned long lastReset = 0;
  if (millis() - lastReset >= 1000) {
      windSensor.resetCount();
      lastReset = millis();
  }

  // Display wind status if motor is blocked
  static unsigned long lastWindWarning = 0;
  if (windSensor.isMotorBlocked() && millis() - lastWindWarning >= 5000) {
    unsigned long remainingMs = windSensor.getCooldownRemaining();
    unsigned long remainingMin = remainingMs / 60000;
    unsigned long remainingSec = (remainingMs % 60000) / 1000;
    Serial.print("Wind cooldown active. Time remaining: ");
    Serial.print(remainingMin);
    Serial.print("m ");
    Serial.print(remainingSec);
    Serial.println("s");
    lastWindWarning = millis();
  }

  handleButtons(CYCLE_PIN, INCREASE_PIN, DECREASE_PIN,
                TEMPERATURE_INCREMENT_VALUE, OPEN_INCREMENT_VALUE,
                FROST_INCREMENT_VALUE, WIND_INCREMENT_VALUE);

  // Only operate motor if temperature control requires it
  // Motor blocking is handled inside moveSteps()
  if (tempCurrent > tempTarget){
    stepper->oneRotation(true);
  }
}
