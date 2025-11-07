#include <Wire.h>
#include <TinyScreen.h>
// #include <ServoDriver.h>  // uncomment later when servo is connected

TinyScreen display = TinyScreen(TinyScreenPlus);

// ----- STATUS VARIABLES -----
bool isRaining = false;
bool windowOpen = true;
bool autoMode = true;
int moistureVal = 0;  // simulated for now

unsigned long lastUpdate = 0;

void setup() {
  Wire.begin();
  display.begin();
  display.setFlip(true);
  display.setBrightness(12);
  display.clearScreen();
  display.setFont(thinPixel7_10ptFontInfo);
  randomSeed(analogRead(0));  // fake sensor randomness
}

void loop() {
  simulateSensor();  // temporary moisture simulation
  autoControl();     // automatic window logic
  drawScreen();      // update display
  delay(500);
}

// ---------- Simulated Sensor (replace later) ----------
void simulateSensor() {
  if (millis() - lastUpdate > 1000) {
    moistureVal = random(30, 100);    // fake moisture %
    isRaining = (moistureVal > 70);   // rain if > 70%
    lastUpdate = millis();
  }
}

// ---------- Auto Logic ----------
void autoControl() {
  if (autoMode) windowOpen = !isRaining;
}

// ---------- Display ----------
void drawScreen() {
  display.clearScreen();
  display.setCursor(10, 5);  display.print("SMART WINDOW");
  display.setCursor(10, 20); display.print("Moisture: "); display.print(moistureVal);
  display.setCursor(10, 35); display.print("Rain: "); display.print(isRaining ? "YES" : "NO");
  display.setCursor(10, 50); display.print("Window: "); display.print(windowOpen ? "OPEN" : "CLOSED");
}
