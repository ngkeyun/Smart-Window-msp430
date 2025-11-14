/************************************************************************
 * Complete Servo/Sensor Project
 *
 * This sketch implements the full state machine logic.
 *
 * *** NEW LOGIC v10 ***
 * - Scraps the "deadzone" and "idle" states.
 * - Uses a single "STATE_AUTO_ACTIVE" that constantly seeks its
 * target position (wet or dry).
 * - This fixes the "stuck in homing" loop.
 * - Uses the new user-provided calibration data.
 *
 ************************************************************************/

// --- Core Libraries ---
#include <Wire.h>
#include <SPI.h>
#include <Wireling.h>

// --- Screen Library ---
#include <TinyScreen.h>   // The screen library we know works

// --- Component Libraries ---
#include <ATtiny25.h>     // For Moisture Sensor
#include <VL53L0X.h>      // For Time of Flight Sensor
#include <ServoDriver.h>  // For Servo Controller (PCA9685 chip)


// =======================================================================
// ===                   !!! USER CONFIGURATION !!!                    ===
// =======================================================================

// --- 1. PORT NUMBERS (from your file) ---
#define PORT_BUTTON   0
#define PORT_MOISTURE 1
#define PORT_DISTANCE 3

// --- 2. SET SENSOR THRESHOLDS (from your file) ---
#define MOISTURE_WET_PERCENT 5   // "isRaining = (moisturePercent > 5)"
#define MOISTURE_DRY_PERCENT 4   // Back to 4. Hysteresis is less important now.

// --- 3. SET SERVO CONSTANTS (from user calibration) ---
// *** NEW CALIBRATION DATA ***
#define closePos 1000  // "Wet" position (closing for rain)
#define openPos  2000  // "Dry" position (opening for sun)
const int servoChannel = 1;

// --- 4. !!! NEW CALIBRATION (from user calibration) !!! ---
// *** NEW CALIBRATION DATA ***
// openPos (1900) is at 20mm
// closePos (1050) is at 99mm
#define TOF_DIST_AT_OPEN_POS  10  // (mm) User calibrated
#define TOF_DIST_AT_CLOSE_POS 80  // (mm) User calibrated

// --- 5. SERVO DEADZONE (REMOVED) ---
// const int TOF_DEADZONE_MM = 15; // <-- We no longer need this

// --- 6. SET SERVO SPEED (NON-BLOCKING) ---
#define SERVO_MOVE_INTERVAL 50  // Move servo every 20ms (50 steps/sec)
#define SERVO_STEP_SIZE     15   // Move 5 units each step (tune this for speed)

// =======================================================================
// ===               HARDWARE & STATE OBJECTS                          ===
// =======================================================================

// --- Hardware Objects (from your file) ---
TinyScreen display = TinyScreen(TinyScreenPlus);
ATtiny25     moistureSensor;
VL53L0X      distanceSensor;
ServoDriver  servo(NO_R_REMOVED);

// --- System State Variables ---
enum SystemMode {
  MODE_MANUAL,
  MODE_AUTO
};
SystemMode currentSystemMode = MODE_MANUAL;

// --- NEW, SIMPLIFIED STATES ---
enum AutoState {
  STATE_AUTO_ACTIVE,  // This state now does all the work
  STATE_INTERRUPTED
};
AutoState currentAutoState = STATE_AUTO_ACTIVE; // Start in this state

// --- Global Sensor/Control Variables ---
int   currentMoistureRaw     = 0;
int   currentMoisturePercent = 0;
int   currentDistance        = 0;
bool  interruptBtnPressed    = false;
int   currentServoPos        = openPos; // Start at the "Dry" (open) position
bool  tofSensorOK            = false;
bool  hasSyncedFromManual    = false; // Flag to sync position once

// --- Non-Blocking Timer & Button Latch Variables ---
unsigned long lastServoMoveTime = 0;
bool lastModeBtnState     = false;
bool lastInterruptBtnState = false;

// =======================================================================
// ===                           SETUP                                 ===
// =======================================================================
void setup() {
  // --- Initialize I2C and Hardware ---
  Wire.begin();
  display.begin();
  Wireling.begin();
  
  display.setBrightness(10);
  display.setFont(thinPixel7_10ptFontInfo); // Set font
  display.fontColor(TS_8b_White, TS_8b_Black);

  // --- Initialize All Wireling Components ---
  Wireling.selectPort(PORT_MOISTURE);
  Wireling.selectPort(PORT_BUTTON);
  Wireling.selectPort(PORT_DISTANCE);
  
  if (distanceSensor.init()) {
    tofSensorOK = true;
    distanceSensor.setTimeout(500);
    distanceSensor.setMeasurementTimingBudget(200000);
    distanceSensor.startContinuous();
  } else {
    tofSensorOK = false;
  }

  // --- Initialize Stacked Servo Controller ---
  servo.useResetPin();
  servo.begin(20000);
  servo.setServo(servoChannel, openPos); // Go to start pos
  currentServoPos = openPos;

  // --- Relax servo on startup ---
  servo.setServo(servoChannel, 0); 
}

// =======================================================================
// ===                         MAIN LOOP                               ===
// =======================================================================
void loop() {
  // 1. Check for a mode change (Manual/Auto)
  checkModeButton();

  // 2. Run logic based on the current mode
  if (currentSystemMode == MODE_AUTO) {
    readAllSensors();
    
    // --- "STUCK IN HOMING" BUG FIX ---
    Wireling.selectPort(0xFFFF); 
    
    runAutoStateMachine();
  } else {
    runManualMode();
  }

  // 3. Update the screen with the latest info
  updateScreen();
}

// =======================================================================
// ===                    HELPER FUNCTIONS                             ===
// =======================================================================

/**
 * @brief Checks the TinyScreen+ top-right button for a mode change.
 */
void checkModeButton() {
  bool modeBtnState = display.getButtons(TSButtonUpperRight);
  if (modeBtnState == true && lastModeBtnState == false) {
    if (currentSystemMode == MODE_MANUAL) {
      currentSystemMode = MODE_AUTO;
      // --- NEW ---
      // We are entering AUTO, so we must sync.
      currentAutoState = STATE_AUTO_ACTIVE; 
      hasSyncedFromManual = false; // Flag that we need to sync
    } else {
      currentSystemMode = MODE_MANUAL;
      // Send a "pulse off" command (0) to relax the servo.
      servo.setServo(servoChannel, 0); 
    }
  }
  lastModeBtnState = modeBtnState;
}

/**
 * @brief Reads all sensors and stores values in global variables.
 */
void readAllSensors() {
  // --- Read Moisture ---
  Wireling.selectPort(PORT_MOISTURE);
  currentMoistureRaw = moistureSensor.readMoisture();
  currentMoisturePercent = map(currentMoistureRaw, 0, 1023, 0, 100);
  currentMoisturePercent = constrain(currentMoisturePercent, 0, 100);

  // --- Read ToF Sensor (Continuous, Non-Blocking) ---
  if (tofSensorOK) {
    Wireling.selectPort(PORT_DISTANCE);
    int distance = distanceSensor.readRangeContinuousMillimeters();
    if (distance > 0 && distance < 8190) { 
      currentDistance = distance; // Update the global variable
    }
  } else {
    currentDistance = 0;
  }

  // --- Read Interrupt Button (Analog method) ---
  Wireling.selectPort(PORT_BUTTON);
  int buttonValue = analogRead(A0); 
  bool interruptBtnState = (buttonValue < 100);
  
  if (interruptBtnState == true && lastInterruptBtnState == false) {
    interruptBtnPressed = true;
  } else {
    interruptBtnPressed = false;
  }
  lastInterruptBtnState = interruptBtnState;
}

/**
 * @brief Logic for when the system is in MANUAL mode. (Does nothing)
 */
void runManualMode() {
  // We do nothing, servo is relaxed.
}

/**
 * @brief The main state machine logic for AUTO mode.
 */
void runAutoStateMachine() {
  
  unsigned long currentTime = millis();
  bool servoMoveTime = (currentTime - lastServoMoveTime >= SERVO_MOVE_INTERVAL);

  // --- State Machine Logic ---
  switch (currentAutoState) {

    // --- THIS IS NOW THE ONLY "ACTIVE" STATE ---
    case STATE_AUTO_ACTIVE:
      // --- Step 1: Sync position from ToF (runs ONCE) ---
      if (!hasSyncedFromManual) {
        if (tofSensorOK && currentDistance > 0) { 
          // The read is good! Use it to find our position.
          // *** NEW MAPPING ***
          // (dist, 20, 99, 1900, 1050)
          currentServoPos = map(currentDistance, TOF_DIST_AT_OPEN_POS, TOF_DIST_AT_CLOSE_POS, openPos, closePos);
          currentServoPos = constrain(currentServoPos, min(closePos, openPos), max(closePos, openPos));
        }
        // If ToF failed, we just use the last known currentServoPos,
        // which prevents the "snap".
        hasSyncedFromManual = true; // We are now synced!
      }

      // --- Step 2: Check for interrupt button ---
      if (interruptBtnPressed) {
        currentAutoState = STATE_INTERRUPTED;
        break; // Exit this state
      }

      // --- Step 3: Determine our target position based on moisture ---
      int targetPos;
      if (currentMoisturePercent >= MOISTURE_WET_PERCENT) {
        targetPos = closePos; // Target is 1050 (Wet)
      } else {
        targetPos = openPos;  // Target is 1900 (Dry)
      }

      // --- Step 4: Seek the target position ---
      if (servoMoveTime) {
        
        if (currentServoPos < targetPos) { // Need to move "up" (to openPos)
          currentServoPos += SERVO_STEP_SIZE;
          if (currentServoPos > targetPos) currentServoPos = targetPos; // Cap
          servo.setServo(servoChannel, currentServoPos);
          lastServoMoveTime = currentTime;
          
        } else if (currentServoPos > targetPos) { // Need to move "down" (to closePos)
          currentServoPos -= SERVO_STEP_SIZE;
          if (currentServoPos < targetPos) currentServoPos = targetPos; // Cap
          servo.setServo(servoChannel, currentServoPos);
          lastServoMoveTime = currentTime;
          
        } else {
          // We have arrived at the target!
          // We do nothing. The servo is now "holding" this position
          // because we will send the same `setServo` command next loop.
          // We DO NOT relax the servo.
        }
      }
      break;

    case STATE_INTERRUPTED:
      // Action: Hold servo at current position (do nothing)
      // Check: Was the button pressed again to reset?
      if (interruptBtnPressed) {
        // Go back to the active state, and force a re-sync
        currentAutoState = STATE_AUTO_ACTIVE;
        hasSyncedFromManual = false; // Force a new ToF read
      }
      break;
  }
}

/**
 * @brief Updates the TinyScreen+ display with the current system status.
 */
void updateScreen() {
  display.clearScreen();
  display.fontColor(TS_8b_White, TS_8b_Black);

  // --- Display Mode ---
  display.setCursor(0, 0);
  if (currentSystemMode == MODE_MANUAL) {
    display.print("Mode: MANUAL");
  } else {
    display.print("Mode: AUTO");
    
    // --- Display Auto State ---
    display.setCursor(0, 10);
    display.print("State: ");
    switch (currentAutoState) {
      case STATE_AUTO_ACTIVE:
        // Show if we are moving or holding
        if (currentMoisturePercent >= MOISTURE_WET_PERCENT) {
          if (currentServoPos == closePos) display.print("CLOSED (Wet)");
          else display.print("CLOSING...");
        } else {
          if (currentServoPos == openPos) display.print("OPEN (Dry)");
          else display.print("OPENING...");
        }
        break;
      case STATE_INTERRUPTED:    display.print("STOPPED"); break;
    }

    // --- Display Sensor Values ---
    display.setCursor(0, 30);
    display.print("Moisture: ");
    display.print(currentMoisturePercent);
    display.print("%");
    
    display.setCursor(0, 40);
    display.print("Position: "); 
    if (tofSensorOK) {
      display.print(currentDistance);
      display.print("mm");
    } else {
      display.print("FAILED"); 
    }

    display.setCursor(0, 50);
    display.print("Servo: ");
    display.print(currentServoPos);
  }
}