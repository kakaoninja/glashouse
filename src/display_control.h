#ifndef DISPLAY_CONTROL_H
#define DISPLAY_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD dimensions
#define LCD_COLS 20
#define LCD_ROWS 4


// Display line variables
extern int tempCurrent;
extern int tempTarget;
extern int openPercent;
extern int openTarget;
extern int frostCurrent;
extern int frostTarget;
extern int windCurrent;
extern int windTarget;

extern LiquidCrystal_I2C lcd;
extern int currentLine;

void initializeDisplay();
void scanI2CDevices();
void updateDisplay();
void handleButtons(int cyclePin, int increasePin, int decreasePin,
                   int increment_1, int increment_2,
                   int increment_3, int increment_4);
int getCurrentLine();
void setCurrentLine(int line);
void printWithHighlight(int value, int col, int row, bool highlight);

#endif