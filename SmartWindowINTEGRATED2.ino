/*
 * SMART WINDOW RainAway - ENHANCED
 
 * Hardware:
- TinyScreen+
- ASD2303 Servo Controller (I2C) – servo on channel 1
- Emergency Stop Button Wireling (Port 0)
- Soil Moisture Sensor Wireling (Port 1, ATtiny25)
- VL53L0X Time-of-Flight Distance Sensor Wireling (Port 3)

 * Features:
 - AUTO MODE: Opens/closes window based on moisture ("rain")
 - POSITION MEMORY: Servo stops at exact position on emergency/obstacle
 - SAFETY: Distance sensor blocks closing and stops servo at that position
 - AUTO MODE TOGGLE: TinyScreen Upper-Right button
 - EMERGENCY STOP: Wireling button on Port 0 (stops servo at current position)
 */

#include <Wire.h>
#include <Wireling.h>
#include <TinyScreen.h>
#include <ServoDriver.h>
#include "VL53L0X.h"
#include <ATtiny25.h>

//WIRELING PORT ASSIGNMENT
const int PORT_BUTTON   = 0;   // Emergency stop button Wireling
const int PORT_MOISTURE = 1;   // Soil moisture Wireling (ATtiny25)
const int PORT_DISTANCE = 3;   // VL53L0X ToF distance Wireling

// Tiny Screen Plus
TinyScreen display = TinyScreen(TinyScreenPlus);

// Moisture sensor (ATtiny25)
ATtiny25 moistureSensor;

// ToF distance sensor
VL53L0X distanceSensor;

// Servo driver (ASD2303)
ServoDriver servo(NO_R_REMOVED);
const int servoChannel = 1;
const int closePos = 1000;    // microseconds for closed
const int openPos  = 2000;    // microseconds for open

// SERVO POSITION TRACKING
int currentServoPos = openPos; // track current servo position

//  State Variables
enum SystemState {
  NORMAL,           // Normal operation
  EMERGENCY_ACTIVE  // Emergency stop active (servo motor off) - HIGHEST PRIORITY
};

SystemState systemState = NORMAL;

bool manualMode    = false;   // true = AUTO MODE OFF (user may push window)

bool isRaining = false;
bool windowOpen = true;       // true=open, false=closed
bool isClosing = false;       // true while auto-closing
bool blocked   = false;       // obstacle detected by ToF sensor

int  moistureRaw     = 0;
int  moisturePercent = 0;

unsigned long lastMoistureRead = 0;

// emergency flashing
unsigned long flashTimer  = 0;
bool          flashState  = false;

// closing flashing
unsigned long closeFlashTimer = 0;
bool          closeFlashState = false;
unsigned long closingStart    = 0;
unsigned long closingDuration = 15000; // ms – 15 seconds for servo to close

// Button debounce
unsigned long lastButtonPress = 0;
const int BUTTON_DEBOUNCE = 300;

// Setup
void setup() {
  Wire.begin();
  Wireling.begin();

  //Display 
  display.begin();
  display.setFlip(true);
  display.setBrightness(12);
  display.clearScreen();
  display.setFont(thinPixel7_10ptFontInfo);

  //Emergency Stop Button (Wireling Port 0) 
  Wireling.selectPort(PORT_BUTTON);
  pinMode(0, INPUT_PULLUP);   // button Wireling uses this pin

  //Moisture Sensor (Wireling Port 1, ATtiny25) 
  Wireling.selectPort(PORT_MOISTURE);
  // ATtiny25 sensor doesn't need special begin()

  //Distance Sensor (Wireling Port 3, VL53L0X) 
  Wireling.selectPort(PORT_DISTANCE);
  distanceSensor.init();
  distanceSensor.setTimeout(500);
  distanceSensor.setMeasurementTimingBudget(200000);
  distanceSensor.startContinuous();

  //Servo Driver (ASD2303) 
  servo.useResetPin();
  if (servo.begin(20000)) {
  }
  servo.setFailsafe(1000); // stop if no command for 1s
}

// Loop
void loop() {
  // 1. EMERGENCY STOP BUTTON (Wireling Port 0) 
  Wireling.selectPort(PORT_BUTTON);
  int buttonValue = analogRead(A0);  // Wireling button analog threshold

  if (buttonValue < 100 && (millis() - lastButtonPress > BUTTON_DEBOUNCE)) {
    handleButtonPress();
    lastButtonPress = millis();
  }

  // If emergency active: show flashing STOP and freeze everything
  if (systemState == EMERGENCY_ACTIVE) {
    drawEmergencyStop();
    // Motor is off - no servo command sent
    return;  // skip sensors, auto logic, etc.
  }

  // 2. READ SENSORS 
  readMoisture();   // uses PORT_MOISTURE
  readDistance();   // uses PORT_DISTANCE

  // 3. AUTO MODE TOGGLE (TinyScreen buttons) 
  uint8_t buttons = display.getButtons();

  // Upper-Right button toggles AUTO MODE ON/OFF
  if (buttons & TSButtonUpperRight) {
    manualMode = !manualMode;   // true = Auto OFF
    delay(200);                 // debounce
  }

  // When manualMode == true, AUTO MODE is OFF.
  // Servo is stopped, and user may manually push the window.
  if (manualMode) {
    // Motor is off - no servo command sent
    drawManualDisplay();                    // "AUTO MODE OFF"
    return;                                 // skip auto logic and servo movement
  }

  /********** 4. AUTO LOGIC (ONLY when Auto Mode is ON) **********/
  autoLogic();

  /********** 5. SERVO CONTROL **********/
  applyServoMovement();

  /********** 6. NORMAL DISPLAY **********/
  drawNormalDisplay();
}

/**************************************************************
 * HANDLE BUTTON PRESS
 **************************************************************/
void handleButtonPress() {
  if (systemState == EMERGENCY_ACTIVE) {
    // Reset from emergency back to normal
    systemState = NORMAL;
    isClosing = false;
  } else {
    // Trigger emergency stop - HIGHEST PRIORITY STATE
    systemState = EMERGENCY_ACTIVE;
    isClosing = false;
    // Motor turns off at current position
  }
}

/**************************************************************
 * READ MOISTURE SENSOR (ATtiny25 Wireling)
 **************************************************************/
void readMoisture() {
  if (millis() - lastMoistureRead > 500) {
    Wireling.selectPort(PORT_MOISTURE);

    moistureRaw = moistureSensor.readMoisture();   // library function
    // Map raw reading to a rough 0–100% range (adjust if needed)
    moisturePercent = map(moistureRaw, 0, 1023, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);

    // Threshold for "raining" – tune if needed
    isRaining = (moisturePercent > 5);

    lastMoistureRead = millis();
  }
}

/**************************************************************
 * READ DISTANCE SENSOR (VL53L0X Wireling)
 **************************************************************/
void readDistance() {
  Wireling.selectPort(PORT_DISTANCE);
  int mm = distanceSensor.readRangeContinuousMillimeters();

  // Obstacle if something < 100mm (10cm) and valid reading
  blocked = (mm > 0 && mm < 100);
}

/**************************************************************
 * AUTO LOGIC – Option B (safety only while closing)
 **************************************************************/
void autoLogic() {
  static bool previousTarget = true; // last value of windowOpen

  // Auto target: if raining, we want it closed; else open
  windowOpen = !isRaining;

  // Detect transition OPEN -> CLOSED (start closing)
  if (previousTarget == true && windowOpen == false) {
    isClosing      = true;
    closingStart   = millis();
    closeFlashTimer = millis();
    systemState = NORMAL; // ensure we're in normal state
  }

  // If rain stops while closing, immediately stop closing and open
  if (isClosing && !isRaining) {
    isClosing = false;
  }

  // When enough time has passed AND no obstacle,
  // assume the window is fully closed and stop "closing" state.
  if (isClosing && !blocked && (millis() - closingStart >= closingDuration)) {
    isClosing = false;
  }

  previousTarget = windowOpen;
}

/**************************************************************
 * SERVO CONTROL
 **************************************************************/
void applyServoMovement() {
  if (systemState == EMERGENCY_ACTIVE) {
    // Emergency: motor is off - no command sent (HIGHEST PRIORITY)
  }
  else if (isClosing && blocked) {
    // Obstacle detected while closing: motor is off - no command sent
  }
  else if (isClosing) {
    // Closing: drive servo to close position
    servo.setServo(servoChannel, closePos);
    currentServoPos = closePos;
  }
  else if (windowOpen) {
    // Opening: drive servo to open position
    servo.setServo(servoChannel, openPos);
    currentServoPos = openPos;
  }
  else {
    // Closed target, no active movement - motor off
  }
}

/**************************************************************
 * DISPLAY – NORMAL RUNNING
 **************************************************************/
void drawNormalDisplay() {
  display.clearScreen();

  display.setCursor(5, 5);
  display.print("SMART WINDOW");

  display.setCursor(5, 20);
  display.print("Moist: ");
  display.print(moisturePercent);
  display.print("%");

  display.setCursor(5, 32);
  display.print("Rain: ");
  display.print(isRaining ? "YES" : "NO");

  display.setCursor(5, 44);
  display.print("Window: ");
  if (isClosing)       display.print("CLOSING");
  else if (windowOpen) display.print("OPEN");
  else                 display.print("CLOSED");

  display.setCursor(5, 56);
  display.print("Mode: ");
  display.print("AUTO");

  // Show if blocked
  if (blocked) {
    display.setCursor(70, 56);
    display.print("BLK");
  }

  // Flashing "WINDOW CLOSING" message
  if (isClosing) {
    if (millis() - closeFlashTimer > 300) {
      closeFlashState = !closeFlashState;
      closeFlashTimer = millis();
    }
    if (closeFlashState) {
      display.setCursor(10, 10);
      display.print("WINDOW CLOSING");
    }
  }
}

/**************************************************************
 * DISPLAY – AUTO MODE OFF (manual push allowed)
 **************************************************************/
void drawManualDisplay() {
  display.clearScreen();

  display.setCursor(10, 20);
  display.print("AUTO MODE OFF");

  display.setCursor(10, 40);
  display.print("UP-R: Toggle Mode");
}

/**************************************************************
 * DISPLAY – EMERGENCY STOP
 **************************************************************/
void drawEmergencyStop() {
  if (millis() - flashTimer > 300) {
    flashState = !flashState;
    flashTimer = millis();
  }

  // Fill screen with flashing white/black
  for (int y = 0; y < 64; y++) {
    display.drawLine(0, y, 95, y, flashState ? 1 : 0);
  }

  display.setCursor(28, 26);
  display.print("STOP!!!");
  display.setCursor(29, 27);  // slight offset for "bold" effect
  display.print("STOP!!!");
  display.setCursor(15, 50);
  display.print("Press to RESET");
}
