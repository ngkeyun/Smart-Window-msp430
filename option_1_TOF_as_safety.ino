/************************************************************************
 * Complete Servo/Sensor Project
 *
 * This sketch implements the full state machine logic.
 *
 * LIBRARIES USED (from user's correct file):
 * - TinyScreen.h
 * - ATtiny25.h (Moisture Sensor)
 * - VL53L0X.h (Time of Flight Sensor)
 * - ServoDriver.h (Servo Controller ASD2303)
 *
 * HARDWARE ASSUMPTIONS:
 * - TinyScreen+
 * - ASD2303 Servo Controller is a stacked TinyShield (NOT a Wireling)
 * - Port 0: AST1028 Button Wireling
 * - Port 1: ATtiny25 Moisture Sensor Wireling
 * - Port 3: VL53L0X Distance Sensor Wireling
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
// #include <AST1028.h>   // <-- REMOVE THIS LINE (it doesn't exist)


// =======================================================================
// ===           !!! USER CONFIGURATION !!!                       ===
// =======================================================================

// --- 1. PORT NUMBERS (from your file) ---
#define PORT_BUTTON   0
#define PORT_MOISTURE 1
#define PORT_DISTANCE 3
// NOTE: PORT_SERVO is not needed as it's a stacked TinyShield

// --- 2. SET SENSOR THRESHOLDS (from your file) ---
#define MOISTURE_WET_PERCENT 5   // "isRaining = (moisturePercent > 5)"
#define MOISTURE_DRY_PERCENT 4   // Assumed dry is just below wet
#define TOF_THRESHOLD_MM  100    // 10cm = 100mm

// --- 3. SET SERVO CONSTANTS (LOGIC REVERSED) ---
const int closePos = 1000;  // "Wet" position
const int openPos  = 2000;  // Default "dry" position
const int servoChannel = 1; // Servo channel (1-4) on the ASD2303

// --- 4. SET SERVO SPEED (NON-BLOCKING) ---
#define SERVO_MOVE_INTERVAL 20  // Move servo every 20ms (50 steps/sec)
#define SERVO_STEP_SIZE     5   // Move 5 units each step (tune this for speed)

// --- 5. SET FLASHING ALERT SPEED (NEW) ---
#define FLASH_INTERVAL 400 // Time in ms for one half of the blink (400ms on, 400ms off)

// =======================================================================
// ===           HARDWARE & STATE OBJECTS                           ===
// =======================================================================

// --- Hardware Objects (from your file) ---
TinyScreen display = TinyScreen(TinyScreenPlus);
ATtiny25     moistureSensor;
VL53L0X      distanceSensor;
ServoDriver  servo(NO_R_REMOVED); // For ASD2303 (PCA9685)
// AST1028     interruptButton;     // <-- REMOVE THIS LINE


// --- System State Variables ---
enum SystemMode {
  MODE_MANUAL,
  MODE_AUTO
};
SystemMode currentSystemMode = MODE_MANUAL;

// --- REFACTORED: State names are now logical ---
enum AutoState {
  STATE_HOMING,           // Moving to openPos on startup
  STATE_AT_OPEN,          // Holding at openPos (2000), "Idle (Dry)"
  STATE_MOVING_TO_CLOSED, // Moving to closePos (1000)
  STATE_AT_CLOSED,        // Holding at closePos (1000), "Closed (Wet)"
  STATE_INTERRUPTED,      // Stopped by user/ToF
  STATE_MOVING_TO_OPEN    // Moving to openPos (2000)
};
AutoState currentAutoState = STATE_AT_OPEN; // Start in the default "open" state

// --- Global Sensor/Control Variables ---
int   currentMoistureRaw    = 0;
int   currentMoisturePercent = 0;
int   currentDistance       = 0;
bool  interruptBtnPressed   = false; // "Flag" for a single button press
int   currentServoPos       = openPos; // Start at the open pos
bool  tofSensorOK           = false; // <-- ADD THIS to track if the ToF sensor is working

// --- Non-Blocking Timer & Button Latch Variables ---
unsigned long lastServoMoveTime = 0;
bool lastModeBtnState     = false;
bool lastInterruptBtnState = false;

// =======================================================================
// ===                           SETUP                                ===
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
  
  // Moisture (no begin())
  Wireling.selectPort(PORT_MOISTURE);

  // Button (from your 'half-working' code)
  Wireling.selectPort(PORT_BUTTON);
  // pinMode(0, INPUT_PULLUP); // <-- REMOVE THIS LINE. The AST1028 is an analog button.
  // interruptButton.begin(); // <-- REMOVE THIS LINE

  // ToF Distance Sensor (from your file)
  Wireling.selectPort(PORT_DISTANCE);
  
  // --- FIX 1: Check if the ToF sensor actually initialized ---
  if (distanceSensor.init()) {
    tofSensorOK = true; // It works!
    distanceSensor.setTimeout(500);
    distanceSensor.setMeasurementTimingBudget(200000);
    distanceSensor.startContinuous();
  } else {
    tofSensorOK = false; // It failed.
  }

  // --- Initialize Stacked Servo Controller ---
  // (No port selection needed)
  servo.useResetPin(); // <-- ADD THIS LINE (from your 'half-working' code)
  servo.begin(20000);  // <-- FIX: Needs a value (e.g., 20000 from your other file)
  
  // --- MODIFIED: Start at the "open" (dry) position ---
  servo.setServo(servoChannel, openPos);
  currentServoPos = openPos;
}

// =======================================================================
// ===                           MAIN LOOP                            ===
// =======================================================================
void loop() {
  // 1. Check for a mode change (Manual/Auto)
  checkModeButton();

  // 2. Run logic based on the current mode
  if (currentSystemMode == MODE_AUTO) {
    readAllSensors();
    runAutoStateMachine();
  } else {
    runManualMode();
  }

  // 3. Update the screen with the latest info
  updateScreen();
  
  // We do NOT use delay() here so the loop runs fast
  // and stays responsive to sensors.
}

// =======================================================================
// ===                       HELPER FUNCTIONS                       ===
// =======================================================================

/**
 * @brief Checks the TinyScreen+ top-right button for a mode change.
 * Uses "edge detection" to toggle only ONCE per press.
 *
 * *** MODIFIED: Now relaxes the servo when switching to MANUAL mode. ***
 */
void checkModeButton() {
  // Read the physical button state
  bool modeBtnState = display.getButtons(TSButtonUpperRight);

  // Check if the button was JUST pressed (state changed from false to true)
  if (modeBtnState == true && lastModeBtnState == false) {
    // Toggle the mode
    if (currentSystemMode == MODE_MANUAL) {
      // Switching from MANUAL to AUTO
      currentSystemMode = MODE_AUTO;
      // When switching to Auto, always re-home the servo
      currentAutoState = STATE_HOMING; 
    } else {
      // Switching from AUTO to MANUAL
      currentSystemMode = MODE_MANUAL;
      
      // --- THIS IS THE FIX (based on your info) ---
      // Send a "pulse off" command (0) to relax the servo.
      servo.setServo(servoChannel, 0); 
    }
  }
  lastModeBtnState = modeBtnState; // Remember the state for next loop
}

/**
 * @brief Reads all sensors and stores values in global variables.
 * Manages port selection and button "edge detection".
 */
void readAllSensors() {
  // --- Read Moisture ---
  Wireling.selectPort(PORT_MOISTURE);
  currentMoistureRaw = moistureSensor.readMoisture();
  // Map raw reading to a rough 0–100% range (from your file)
  currentMoisturePercent = map(currentMoistureRaw, 0, 1023, 0, 100);
  currentMoisturePercent = constrain(currentMoisturePercent, 0, 100);

  // --- Read ToF Sensor ---
  // --- FIX 1 (cont'd): Only read the sensor if it's working ---
  if (tofSensorOK) {
    Wireling.selectPort(PORT_DISTANCE);
    currentDistance = distanceSensor.readRangeContinuousMillimeters();
  } else {
    currentDistance = 0; // Otherwise, report 0
  }

  // --- Read Interrupt Button (Analog method) ---
  Wireling.selectPort(PORT_BUTTON);
  
  // --- FIX: The AST1028 button is ANALOG, not digital ---
  // We read the A0 pin (standard for Wireling) and check against a threshold.
  // Pulled high (~1023) when idle, pulled low (~0) when pressed.
  int buttonValue = analogRead(A0); 
  bool interruptBtnState = (buttonValue < 100); // Check if < 100
  
  // Set the "pressed" flag only on the rising edge (false -> true)
  if (interruptBtnState == true && lastInterruptBtnState == false) {
    interruptBtnPressed = true;
  } else {
    interruptBtnPressed = false; // Clear the flag in all other cases
  }
  lastInterruptBtnState = interruptBtnState;
}

/**
 * @brief Logic for when the system is in MANUAL mode. (Does nothing)
 */
void runManualMode() {
  // Manual mode: all sensors and motors are inactive.
  // The servo was already relaxed by checkModeButton().
  // We just let the screen update.
}

/**
 * @brief The main state machine logic for AUTO mode.
 * *** MODIFIED: Logic reversed to Open (Dry) / Close (Wet) ***
 */
void runAutoStateMachine() {
  
  // This is a "non-blocking" servo move.
  unsigned long currentTime = millis();
  bool servoMoveTime = (currentTime - lastServoMoveTime >= SERVO_MOVE_INTERVAL);

  // --- State Machine Logic (using new logical names) ---
  switch (currentAutoState) {

    case STATE_HOMING:
      // This state runs once when switching from Manual to Auto.
      // It safely moves the servo to the 'openPos' (2000)
      
      // Check for interrupts FIRST
      if (interruptBtnPressed || (currentDistance < TOF_THRESHOLD_MM && currentDistance > 0)) {
        currentAutoState = STATE_INTERRUPTED;
        break; // Stop all other logic this loop
      }

      // If no interrupt, check if it's time to move the servo
      if (servoMoveTime) {
        // We need to move towards openPos (2000)
        
        if (currentServoPos > openPos) {
          // We are (probably) past the open pos
          currentServoPos -= SERVO_STEP_SIZE;
          if (currentServoPos < openPos) currentServoPos = openPos; // Cap
          
        } else if (currentServoPos < openPos) {
          // We are (probably) closed (1000), so move to open (2000)
          currentServoPos += SERVO_STEP_SIZE;
          if (currentServoPos > openPos) currentServoPos = openPos; // Cap
          
        } else {
          // We have arrived at the known 'openPos'
          // Now we can safely start the normal logic.
          currentAutoState = STATE_AT_OPEN;
        }
        
        // Only send servo command if we are not yet at the destination
        if (currentAutoState == STATE_HOMING) {
          servo.setServo(servoChannel, currentServoPos);
          lastServoMoveTime = currentTime;
        }
      }
      break;

    case STATE_AT_OPEN:
      // Action: Hold servo at 'openPos' (2000)
      // Check: Is it wet?
      if (currentMoisturePercent >= MOISTURE_WET_PERCENT) {
        currentAutoState = STATE_MOVING_TO_CLOSED;
      }
      break;

    case STATE_MOVING_TO_CLOSED: // This state "closes" (moves to closePos 1000)
      // Check for interrupts FIRST
      if (interruptBtnPressed || (currentDistance < TOF_THRESHOLD_MM && currentDistance > 0)) {
        currentAutoState = STATE_INTERRUPTED;
        break; // Stop all other logic this loop
      }

      // If no interrupt, check if it's time to move the servo
      if (servoMoveTime) {
        // We need to move towards closePos (1000)
        
        if (currentServoPos > closePos) {
          // We are (probably) open (2000), so move to closed (1000)
          currentServoPos -= SERVO_STEP_SIZE;
          if (currentServoPos < closePos) currentServoPos = closePos; // Cap
          
        } else if (currentServoPos < closePos) {
          // We are past the closed pos
          currentServoPos += SERVO_STEP_SIZE;
          if (currentServoPos > closePos) currentServoPos = closePos; // Cap
        } else {
          // We've arrived!
          currentAutoState = STATE_AT_CLOSED;
        }

        // Only send servo command if we are not yet at the destination
        if(currentAutoState == STATE_MOVING_TO_CLOSED) {
          servo.setServo(servoChannel, currentServoPos);
          lastServoMoveTime = currentTime;
        }
      }
      break;

    case STATE_AT_CLOSED: // This state is "at closePos"
      // Action: Hold servo at 'closePos' (1000)
      // Check: Is it dry?
      if (currentMoisturePercent < MOISTURE_DRY_PERCENT) {
        currentAutoState = STATE_MOVING_TO_OPEN;
      }
      break;

    case STATE_INTERRUPTED:
      // Action: Hold servo at current position (do nothing)
      // Check: Was the button pressed again to reset?
      if (interruptBtnPressed) {
        // Reset by moving back to the "open" position
        currentAutoState = STATE_MOVING_TO_OPEN;
      }
      break;

    case STATE_MOVING_TO_OPEN: // This state "opens" (moves to openPos 2000)
      // Check if it's time to move the servo
      if (servoMoveTime) {
        // We need to move towards openPos (2000)

        if (currentServoPos > openPos) {
          // We are past the open pos
          currentServoPos -= SERVO_STEP_SIZE;
          if (currentServoPos < openPos) currentServoPos = openPos; // Cap
          
        } else if (currentServoPos < openPos) {
          // We are (probably) closed (1000), so move to open (2000)
          currentServoPos += SERVO_STEP_SIZE;
          if (currentServoPos > openPos) currentServoPos = openPos; // Cap
          
        } else {
          // We've arrived!
          currentAutoState = STATE_AT_OPEN;
        }
        
        // Only send servo command if we are not yet at the destination
        if(currentAutoState == STATE_MOVING_TO_OPEN) {
          servo.setServo(servoChannel, currentServoPos);
          lastServoMoveTime = currentTime;
        }
      }
      break;
  }
}

/**
 * @brief Updates the TinyScreen+ display with the current system status.
 * *** MODIFIED: Full screen flashing alert implemented. ***
 */
void updateScreen() {
  display.setFont(thinPixel7_10ptFontInfo); // Set font for all cases

  // --- Check for "Flashing Alert" states ---
  if (currentSystemMode == MODE_AUTO &&
      (currentAutoState == STATE_MOVING_TO_CLOSED || currentAutoState == STATE_MOVING_TO_OPEN))
  {
    // --- This is the new "full screen flashing" display logic ---

    // 1. Determine the message
    const char* message = (currentAutoState == STATE_MOVING_TO_CLOSED) ? "CLOSING" : "OPENING";

    // 2. Determine flash colors and fill the entire screen
    bool showRed = (millis() % (FLASH_INTERVAL * 2)) < FLASH_INTERVAL;
    
    if (showRed) {
      // Set background to RED, text to WHITE
      display.fontColor(TS_8b_White, TS_8b_Red);
      display.clearScreen(); // This will now fill the screen with RED
    } else {
      // Set background to BLACK, text to WHITE
      display.fontColor(TS_8b_White, TS_8b_Black);
      display.clearScreen(); // This will now fill the screen with BLACK
    }

    // 3. Center the text (using estimated coordinates)
    // Screen is 96x64. Font is ~10px high. "CLOSING" is 7 chars.
    // Assuming char width of ~6px: 7 * 6 = 42px.
    int x_pos = (96 - 42) / 2; // (96-42)/2 = 27
    int y_pos = (64 - 10) / 2; // (64-10)/2 = 27
    
    display.setCursor(x_pos, y_pos);
    display.print(message);

  }
  else
  {
    // --- This is the "normal" display logic for all other states ---
    
    // Set default colors and clear the screen
    display.fontColor(TS_8b_White, TS_8b_Black);
    display.clearScreen(); 

    // --- Display Mode ---
    display.setCursor(0, 0);
    if (currentSystemMode == MODE_MANUAL) {
      display.print("Mode: MANUAL");
      
      // --- NEW: Show servo is relaxed in manual mode ---
      display.setCursor(0, 10);
      display.print("State: SERVO OFF");

    } else {
      // We must be in MODE_AUTO
      display.print("Mode: AUTO");
      
      // --- Display Auto State (using new logical names) ---
      display.setCursor(0, 10);
      display.print("State: ");
      switch (currentAutoState) {
        case STATE_HOMING:           display.print("HOMING..."); break;
        case STATE_AT_OPEN:          display.print("IDLE (Dry)"); break;
        case STATE_MOVING_TO_CLOSED: display.print("CLOSING"); break; // Should not hit
        case STATE_AT_CLOSED:        display.print("CLOSED (Wet)"); break;
        case STATE_INTERRUPTED:      display.print("STOPPED"); break;
        case STATE_MOVING_TO_OPEN:   display.print("OPENING"); break; // Should not hit
      }

      // --- Display Sensor Values ---
      display.setCursor(0, 30);
      display.print("Moisture: ");
      display.print(currentMoisturePercent);
      display.print("%");
      
      display.setCursor(0, 40);
      display.print("Distance: ");
      if (tofSensorOK) {
        display.print(currentDistance);
        display.print("mm");
      } else {
        display.print("FAILED"); // <-- Show if ToF sensor failed
      }

      display.setCursor(0, 50);
      display.print("Servo: ");
      display.print(currentServoPos);
    }
  }
}