// const int interruptPin = 2;  // Interrupt-Pin (z. B. D2)
// volatile unsigned long rotationCount = 0;  // Zählt Windrad-Umdrehungen

// void setup() {
//   Serial.begin(9600);
//   pinMode(interruptPin, INPUT_PULLUP);  // Interner Pull-up-Widerstand
//   attachInterrupt(digitalPinToInterrupt(interruptPin), countRotation, FALLING);  // Interrupt bei fallender Flanke
// }

// void loop() {
//   // Berechne Windgeschwindigkeit (z. B. Umdrehungen pro Sekunde)
//   float windSpeed = calculateSpeed(rotationCount);
//   Serial.print("Windgeschwindigkeit: ");
//   Serial.print(windSpeed);
//   Serial.println(" km/h");

//   // Falls Windgeschwindigkeit zu hoch -> Abschalten
//   if (windSpeed > 30.0) {  // Beispiel: 30 km/h Grenze
//     emergencyStop();
//   }

//   delay(1000);  // Nur zur Anzeige, nicht blockierend für Interrupts
// }

// // Interrupt Service Routine (ISR) – wird bei jedem Impuls aufgerufen
// void countRotation() {
//   rotationCount++;
// }

// // Berechnet Windgeschwindigkeit basierend auf Umdrehungen
// float calculateSpeed(unsigned long rotations) {
//   const float radius = 0.1;  // Radius des Windrads in Metern
//   const float circumference = 2 * PI * radius;  // Umfang in Metern
//   float speed = (rotations * circumference) / 1.0;  // m/s (hier vereinfacht)
//   return speed * 3.6;  // Umrechnung in km/h
// }

// // Not-Aus-Funktion
// void emergencyStop() {
//   Serial.println("! WIND ZU SCHNELL - ABSCHALTUNG !");
//   // Hier z. B. ein Relais/Mosfet abschalten
//   // digitalWrite(relayPin, LOW);
// }