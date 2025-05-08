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


void initializeDisplay() {
  // Initialize I2C communication
  Wire.begin();
  
  // Try to scan for I2C devices to check connection
  scanI2CDevices();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight(); // Turn on backlight

  // Create custom character for inverted display (block character)
  byte invertedChar[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};
  lcd.createChar(0, invertedChar);

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
    if (value < 10) {
      lcd.write(byte(0)); // Inverted block for first digit
      lcd.setCursor(col + 1, row);
      lcd.print(value);
    } else {
      lcd.print(value); // Will print normally (we'll need two custom chars for full inversion)
    }
  } else {
    if (value < 10) lcd.print("0");
    lcd.print(value);
  }
}
void updateDisplay() {
  // Clear display first
  lcd.clear();
  
  // Update temperature line (row 0)
  lcd.setCursor(0, 0);
  lcd.print("Temp ");
  if (tempCurrent < 10) lcd.print(" ");
  lcd.print(tempCurrent);
  lcd.write(223);
  lcd.print("C  -> ");
  printWithHighlight(tempTarget, 13, 0, currentLine == 0);
  lcd.write(223);
  lcd.print("C");
  
  // Update open line (row 1)
  lcd.setCursor(0, 1);
  lcd.print("Offen ");
  if (openPercent < 10) lcd.print("0");
  lcd.print(openPercent);
  lcd.print("%   -> ");
  printWithHighlight(openTarget, 15, 1, currentLine == 1);
  lcd.print("%  ");
  
  // Update frost line (row 2)
  lcd.setCursor(0, 2);
  lcd.print("Frost ");
  if (frostCurrent < 10) lcd.print("0");
  lcd.print(frostCurrent);
  lcd.write(223);
  lcd.print("C  -> ");
  printWithHighlight(frostTarget, 14, 2, currentLine == 2);
  lcd.write(223);
  lcd.print("C");
  
  // Update wind line (row 3)
  lcd.setCursor(0, 3);
  lcd.print("Wind  ");
  if (windCurrent < 10) lcd.print("0");
  lcd.print(windCurrent);
  lcd.print("kmH -> ");
  printWithHighlight(windTarget, 15, 3, currentLine == 3);
  lcd.print("kmH");
}

void handleButtons(int cyclePin, int increasePin, int decreasePin, int increment) {
  static unsigned long lastDebounceTime = 0;
  static int debounceDelay = 50;
  
  if (digitalRead(cyclePin) == HIGH && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    currentLine = (currentLine + 1) % 4;
    updateDisplay();
  }
  
  if (digitalRead(increasePin) == HIGH && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    switch(currentLine) {
      case 0: tempTarget += increment; break;
      case 1: openTarget += increment; break;
      case 2: frostTarget += increment; break;
      case 3: windTarget += increment; break;
    }
    updateDisplay();
  }
  
  if (digitalRead(decreasePin) == HIGH && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    switch(currentLine) {
      case 0: tempTarget = max(0, tempTarget - increment); break;
      case 1: openTarget = max(0, openTarget - increment); break;
      case 2: frostTarget = max(0, frostTarget - increment); break;
      case 3: windTarget = max(0, windTarget - increment); break;
    }
    updateDisplay();
  }
}

int getCurrentLine() {
  return currentLine;
}

void setCurrentLine(int line) {
  currentLine = constrain(line, 0, 3);
}