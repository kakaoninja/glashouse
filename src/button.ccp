// Button debounce with single press and hold repeat functionality
const int buttonPin = 2;  // Change to your button pin
int buttonState;           // Current reading from button
int lastButtonState = HIGH; // Previous reading from button
unsigned long lastDebounceTime = 0;  // Last time the button was toggled
unsigned long debounceDelay = 50;    // Debounce time in milliseconds
unsigned long lastRepeatTime = 0;    // Last time a repeat was triggered
unsigned long repeatDelay = 200;     // Initial delay before repeats start (ms)
unsigned long repeatInterval = 100;  // Interval between repeats (ms)
bool buttonPressed = false;          // Track if button is pressed
bool buttonHandled = false;          // Track if press was handled



//void setup() {
//  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor
//  Serial.begin(9600);
//}

// void loop() {
//   // Read and debounce button
//   readAndDebounceButton();
  
//   // Handle button actions
//   handleButtonActions();
  
//   // Save the current reading for next loop
//   lastButtonState = digitalRead(buttonPin);
// }

// Function to read button state and apply debounce logic
void readAndDebounceButton() {
  int reading = digitalRead(buttonPin);
  
  // Check for state change (debounce)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // Check if debounce time has passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    updateButtonState(reading);
  }
}

// Function to update button state after debounce
void updateButtonState(int reading) {
  // If the button state has changed
  if (reading != buttonState) {
    buttonState = reading;
    
    // Button was pressed (LOW because we're using INPUT_PULLUP)
    if (buttonState == LOW) {
      buttonPressed = true;
      buttonHandled = false;
      lastRepeatTime = millis(); // Reset repeat timer
    } else {
      buttonPressed = false;
    }
  }
}

// Function to handle button press and hold actions
void handleButtonActions() {
  if (buttonPressed) {
    // Single press detection (not yet handled)
    if (!buttonHandled) {
      incrementValue();
      buttonHandled = true;
    }
    // Hold detection (after initial delay)
    else if (millis() - lastRepeatTime > repeatDelay) {
      // Repeat at specified interval
      if (millis() - lastRepeatTime > repeatInterval) {
        incrementValue();
        lastRepeatTime = millis();
      }
    }
  }
}

void incrementValue() {
  // Replace this with your actual increment logic
  static int value = 0;
  value++;
  Serial.print("Value incremented to: ");
  Serial.println(value);
}