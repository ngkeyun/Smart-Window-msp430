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

// emergency stop variables
bool emergencyStop = false;
unsigned long flashTimer = 0;
bool flashState = false; // for flashing display

// flashing light for when window closing
bool isClosing = false; // for when window closing or not
unsigned long flashTimer2 = 0;
bool flashState2 = false;
unsigned long closingStart = 0; // start time of closing
unsigned long closingDuration = 4000; // ADJUST THIS ACCORDING TO THE TIME IT TAKES TO CLOSE

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
  //  check for emergency stop button
  uint8_t buttons = display.getButtons();
  if (buttons & TSButtonUpperLeft) {
    emergencyStop = !emergencyStop;  // toggle state

    if (emergencyStop) {
      // servoDriver.disableAll(); // to instantly cut servo signal
    }
    delay(300);
  }

  // stop everything when emergency stop is activaged
  if (emergencyStop) {
    drawEmergencyStop();
    return;  // exit loop early (stops sensor + auto logic)
  }

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
  static bool prevState = windowOpen; // trac previous state
  if (autoMode) windowOpen = !isRaining;

  // detect transition from open to close
  if (prevState == true && isRaining) {
    isClosing = true;
    closingStart = millis();
    flashTimer2 = millis(); // start flashing the timer
  }

  if (isClosing && millis() - closingStart >= closingDuration) {
    isClosing = false;
    windowOpen = false; // = fully closed
  }

  if (!isRaining) {
    windowOpen = true;
    isClosing = false;
  }  
  prevState = windowOpen;
}

// ---------- Display ----------
void drawScreen() {
  display.clearScreen();
  display.setCursor(10, 5);  display.print("SMART WINDOW");
  display.setCursor(10, 20); display.print("Moisture: "); display.print(moistureVal);
  display.setCursor(10, 35); display.print("Rain: "); display.print(isRaining ? "YES" : "NO");
  display.setCursor(10, 50); display.print("Window: "); display.print(windowOpen ? "OPEN" : "CLOSED");

  if (isClosing) {
    if (millis() - flashTimer2 > 400) {
      flashState2 = !flashState2;
      flashTimer2 = millis();
    }
    if (flashState2) {
      display.setCursor(15, 10);
      display.print("WINDOW CLOSING");
    }
  }
}

// "EMERGENCY STOP" screen display
void drawEmergencyStop() {
  if (millis() - flashTimer > 400) {  // toggle flash every 400ms
    flashState = !flashState;
    flashTimer = millis();
  }

  display.clearScreen();
  if (flashState) {
    display.setCursor(5, 20);
    display.print("EMERGENCY");
    display.setCursor(20, 35);
    display.print("STOP!!");
  }
}
