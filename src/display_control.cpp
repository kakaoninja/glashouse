#include <Arduino.h>
#include "display_control.h"

// LCD object
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS); // Adjust address if needed

// Variables for display lines
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

void initializeDisplay() {
  // Initialize I2C communication
  Wire.begin();
  
  // Try to scan for I2C devices to check connection
  scanI2CDevices();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight(); // Turn on backlight

  // Create custom character for inverted display (block character)
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
  
  lcd.createChar(1, leftBracket);   // Custom char 1 = [
  lcd.createChar(2, rightBracket);  // Custom char 2 = ]
  updateDisplay();
}

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
    // Print left border at column 14
    lcd.setCursor(14, row);
    lcd.write(byte(1)); // Custom [
    
    // Print value at column 15-16
    lcd.setCursor(15, row);
    if (value < 10) {
      lcd.print("0"); // Leading zero for single digit
    }
    lcd.print(value);
    
    // Print right border at column 17
    lcd.setCursor(17, row);
    lcd.write(byte(2)); // Custom ]
  } else {
    // Normal printing (no borders)
    lcd.setCursor(15, row); // Center the non-highlighted value
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




// void updateDisplay() {
//   // Clear display first
//   lcd.clear();
  
//   // Update temperature line (row 0)
//   lcd.setCursor(0, 0);
//   lcd.print("Temp  ");
//   if (tempCurrent < 10) lcd.print(" ");
//   lcd.print(tempCurrent);
//   lcd.write(223);
//   lcd.print("C  ->");
//   printWithHighlight(tempTarget, 15, 0, currentLine == 0);
//   lcd.write(223);
//   lcd.print("C");
  
//   // Update open line (row 1)
//   lcd.setCursor(0, 1);
//   lcd.print("Offen ");
//   if (openPercent < 10) lcd.print("0");
//   lcd.print(openPercent);
//   lcd.print("%   ->");
//   printWithHighlight(openTarget, 15, 1, currentLine == 1);
//   lcd.print("%  ");
  
//   // Update frost line (row 2)
//   lcd.setCursor(0, 2);
//   lcd.print("Frost ");
//   if (frostCurrent < 10) lcd.print("0");
//   lcd.print(frostCurrent);
//   lcd.write(223);
//   lcd.print("C  ->");
//   printWithHighlight(frostTarget, 15, 2, currentLine == 2);
//   lcd.write(223);
//   lcd.print("C");
  
//   // Update wind line (row 3)
//   lcd.setCursor(0, 3);
//   lcd.print("Wind  ");
//   if (windCurrent < 10) lcd.print("0");
//   lcd.print(windCurrent);
//   lcd.print("kmH ->");
//   printWithHighlight(windTarget, 15, 3, currentLine == 3);
//   lcd.print("kmH");
// }

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

int getCurrentLine() {
  return currentLine;
}

void setCurrentLine(int line) {
  currentLine = constrain(line, 0, 3);
}