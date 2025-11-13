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
 - RESET: Press button again after emergency to resume normal operation
 - SAFETY (Option B): Distance sensor blocks closing
 - AUTO MODE TOGGLE: TinyScreen Upper-Right button
 - EMERGENCY STOP: Wireling button on Port 0 (toggle with reset)
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
const int stopPos  = 1500;    // neutral/stop pulse

// SERVO POSITION TRACKING
int currentServoPos = stopPos; // track current servo position
int savedServoPos   = stopPos; // saved position when stopped

//  State Variables
enum SystemState {
  NORMAL,           // Normal operation
  EMERGENCY_ACTIVE, // Emergency stop active (servo held)
  OBSTACLE_BLOCKED  // Obstacle detected while closing (servo held)
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
unsigned long closingDuration = 4000; // ms – adjust to match your servo closing time

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
    // Do not send servo command - motor stays off at current position
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
    servo.setServo(servoChannel, stopPos);  // stop pulse (motor not driving)
    drawManualDisplay();                    // "AUTO MODE OFF"
    return;                                 // skip auto logic and servo movement
  }

  /********** 4. AUTO LOGIC (ONLY when Auto Mode is ON) **********/
  autoLogic();

  /********** 5. SERVO CONTROL **********/
  applyServoMovement();

  /********** 6. NORMAL DISPLAY **********/
  drawNormalDisplay();

  delay(100);
}

/**************************************************************
 * HANDLE BUTTON PRESS
 **************************************************************/
void handleButtonPress() {
  if (systemState == EMERGENCY_ACTIVE) {
    // Reset from emergency back to normal
    systemState = NORMAL;
    isClosing = false;
  } else if (systemState == OBSTACLE_BLOCKED) {
    // Reset from obstacle back to normal
    systemState = NORMAL;
    isClosing = false;
  } else {
    // Trigger emergency stop
    systemState = EMERGENCY_ACTIVE;
    savedServoPos = currentServoPos; // save current position
    servo.setServo(servoChannel, savedServoPos);
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
  bool wasBlocked = blocked;
  blocked = (mm > 0 && mm < 100);

  // If obstacle detected while closing, enter OBSTACLE_BLOCKED state
  if (isClosing && blocked && !wasBlocked) {
    systemState = OBSTACLE_BLOCKED;
    savedServoPos = currentServoPos; // save current position
  }

  // If obstacle cleared and in OBSTACLE_BLOCKED state, resume normal
  if (!blocked && systemState == OBSTACLE_BLOCKED) {
    systemState = NORMAL;
  }
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
    // Emergency: hold at saved position, no command sent (motor off)
    // Do not send any servo command to turn off the motor
  }
  else if (systemState == OBSTACLE_BLOCKED) {
    // Obstacle detected: hold at saved position, no command sent (motor off)
    // Do not send any servo command to turn off the motor
  }
  else if (isClosing) {
    // Closing: drive servo to close position
    servo.setServo(servoChannel, closePos);
    currentServoPos = closePos;
  }
  else if (windowOpen) {
    // Opening: no safety needed, just open
    servo.setServo(servoChannel, openPos);
    currentServoPos = openPos;
  }
  else {
    // Closed target, no active movement; keep stopped
    servo.setServo(servoChannel, stopPos);
    currentServoPos = stopPos;
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

  // Show state if not normal
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
}