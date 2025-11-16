//Core Libraries
#include <Wire.h>
#include <SPI.h>
#include <Wireling.h>

//Screen Library
#include <TinyScreen.h>

//Component Libraries
#include <ATtiny25.h>     // For Moisture Sensor
#include <VL53L0X.h>      // For Time of Flight Sensor
#include <ServoDriver.h>  // For Servo Controller

//PORT NUMBERS
#define PORT_BUTTON   3
#define PORT_MOISTURE 1
#define PORT_DISTANCE 0

//SENSOR THRESHOLDS
#define MOISTURE_WET_PERCENT 5
#define MOISTURE_DRY_PERCENT 4

//SERVO CONSTANTS
#define closePos 1090 //for when it's raining
#define openPos  1950 // for when it's dry
const int servoChannel = 1;

//TOF CONSTANTS
#define TOF_DIST_AT_OPEN_POS  10  //distance in mm
#define TOF_DIST_AT_CLOSE_POS 75

//SERVO SPEED
#define SERVO_MOVE_INTERVAL 50  // Move servo every 20ms (50 steps/sec)
#define SERVO_STEP_SIZE     15   // Move 15 units each step

//SCREEN CONSTANTS
#define FLASH_INTERVAL 200  //flashing rate of opening and closing alerts, in ms

//Hardware Objects
TinyScreen display = TinyScreen(TinyScreenPlus);
ATtiny25     moistureSensor;
VL53L0X      distanceSensor;
ServoDriver  servo(NO_R_REMOVED);

//System State Variables
enum SystemMode {
  MODE_MANUAL,
  MODE_AUTO
};
SystemMode currentSystemMode = MODE_MANUAL;

enum AutoState {
  STATE_AUTO_ACTIVE,  // This state now does all the work
  STATE_INTERRUPTED
};
AutoState currentAutoState = STATE_AUTO_ACTIVE; // Start in this state

//global Sensor/Control Variables
int   currentMoistureRaw     = 0;
int   currentMoisturePercent = 0;
int   currentDistance        = 0;
bool  interruptBtnPressed    = false;
int   currentServoPos        = openPos; //start at open position (dry)
bool  tofSensorOK            = false;
bool  hasSyncedFromManual    = false; // Flag to sync position once

//Non-Blocking Timer & Button Latch Variables
unsigned long lastServoMoveTime = 0;
bool lastModeBtnState     = false;
bool lastInterruptBtnState = false;





void setup() {
  //Initialize Hardware
  Wire.begin();
  display.begin();
  Wireling.begin();
  
  display.setBrightness(10);
  display.setFont(thinPixel7_10ptFontInfo); // Set font
  display.fontColor(TS_8b_White, TS_8b_Black);

  //Initialize all wireling components
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

  //Initialise servo
  servo.useResetPin();
  servo.begin(20000);
  servo.setServo(servoChannel, openPos); // Go to start position, which is open (dry)
  currentServoPos = openPos;

  //Relax servo on startup by having 0 output on servo channel
  servo.setServo(servoChannel, 0); 
}


//main loop
void loop() {
  //Check for a mode change (Manual/Auto)
  checkModeButton();

  //Run logic based on the current mode
  if (currentSystemMode == MODE_AUTO) {
    readAllSensors();
    
    Wireling.selectPort(0xFFFF); 
    
    runAutoStateMachine();
  } else {
    runManualMode();
  }

  //Update the screen with the latest info
  updateScreen();
}

//Checks the TinyScreen+ top-right button for a mode change.
void checkModeButton() {
  bool modeBtnState = display.getButtons(TSButtonUpperRight);
  if (modeBtnState == true && lastModeBtnState == false) {
    if (currentSystemMode == MODE_MANUAL) {
      currentSystemMode = MODE_AUTO;
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

//Reads all sensors and stores values in global variables.
void readAllSensors() {
  //Moisture
  Wireling.selectPort(PORT_MOISTURE);
  currentMoistureRaw = moistureSensor.readMoisture();
  currentMoisturePercent = map(currentMoistureRaw, 0, 1023, 0, 100);
  currentMoisturePercent = constrain(currentMoisturePercent, 0, 100);

  //Read ToF Sensor (Continuous, Non-Blocking)
  if(tofSensorOK){
    Wireling.selectPort(PORT_DISTANCE);
    int distance = distanceSensor.readRangeContinuousMillimeters();
    if (distance > 0 && distance < 8190) {  //remove common invalid readings, usually 8191 means out of range
      currentDistance = distance; // Update the global variable
    }
  } else {
    currentDistance = 0;
  }

  const int interruptBtnport[] = {A0, A1, A2, A3}; //lookup table, allows easy changing of emergency button port number at the top of code


  //Read Interrupt Button
  Wireling.selectPort(PORT_BUTTON);
  int buttonValue = analogRead(interruptBtnport[PORT_BUTTON]);  //PORT_BUTTON is the port number for the emergency button, can be changed and found at the top of the code
  bool interruptBtnState = (buttonValue < 100); 
  //since button reading is active low. when button is not pressed, value is >1000. when button is pressed, value is much lower, but may not be zero, so we use 100 as a safer threshold

  if (interruptBtnState == true && lastInterruptBtnState == false) {
    interruptBtnPressed = true;
  } else {
    interruptBtnPressed = false;
  }
  lastInterruptBtnState = interruptBtnState;
}

//Logic for when the system is in MANUAL mode
void runManualMode() {
  //do nothing, let servo move freely
}

//The main state machine logic for AUTO mode
void runAutoStateMachine() {
  
  unsigned long currentTime = millis();
  bool servoMoveTime = (currentTime - lastServoMoveTime >= SERVO_MOVE_INTERVAL);

  //state Machine Logic
  switch (currentAutoState) {

    case STATE_AUTO_ACTIVE:
      //Sync position from ToF
      if (!hasSyncedFromManual) { //if has NOT synced from manual
        if (tofSensorOK && currentDistance > 0) {   //and if TOF sensor is up and working
          // The read is good! Use it to find our position.
          currentServoPos = map(currentDistance, TOF_DIST_AT_OPEN_POS, TOF_DIST_AT_CLOSE_POS, openPos, closePos); 
          currentServoPos = constrain(currentServoPos, min(closePos, openPos), max(closePos, openPos));
        }
        // If ToF failed, we just use the last known currentServoPos so servo doesn't jump position
        hasSyncedFromManual = true;
      }

      //Step 2: Check for interrupt button
      if (interruptBtnPressed) {
        currentAutoState = STATE_INTERRUPTED;
        break; // Exit this state
      }

      //Step 3: Determine our target position based on moisture
      int targetPos;
      if (currentMoisturePercent >= MOISTURE_WET_PERCENT) {
        targetPos = closePos; // Target is 1050 (Wet)
      } else {
        targetPos = openPos;  // Target is 1900 (Dry)
      }

      //Step 4: Seek the target position
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

//Updates the TinyScreen+ display with the current system status.
void updateScreen() {
  display.clearScreen();
  display.fontColor(TS_8b_White, TS_8b_Black);

  //Display Mode
  display.setCursor(0, 0);
  if (currentSystemMode == MODE_MANUAL) {
    display.print("Mode: MANUAL");
  } else {
    display.print("Mode: AUTO");
    
    //Display Auto State
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

    //Display Sensor Values
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
