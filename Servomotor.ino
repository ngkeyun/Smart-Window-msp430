#include <Wire.h>
#include <ServoDriver.h>

#define SerialMonitorInterface SerialUSB   // TinyScreen+ uses SerialUSB communication

ServoDriver servo(NO_R_REMOVED);   // Default I2C address for ASD2303

// Servo Settings
const int servoChannel = 1;   // Servo on Port 1
const int closePos = 1000;    // microseconds to fully close to around 0°
const int openPos  = 2000;    // microseconds toho fully open to around 180°

// Initialize
String initMotor() {
  // Start I2C communication & enable hardware reset line
  Wire.begin();
  servo.useResetPin();

 // Check for servo connection, it expects one pulse every 20 milliseconds
  if (servo.begin(20000)) {  
    SerialMonitorInterface.println("Servo driver not detected!");
    return "ERROR"; // return error if board not found
  }

// if ASD2023 does not receive any command for 1000 milliseconds, 
// it will turn off output, prevent continuously running motor
  servo.setFailsafe(1000);
  SerialMonitorInterface.println("Servo driver initialized!");
  return "OK";
}

// Motor Control Functions
// Open window
String openWindow() {
  servo.setServo(servoChannel, openPos);
  SerialMonitorInterface.println("Opening window...");
  return "OPEN";
}

// Close window
String closeWindow() {
  servo.setServo(servoChannel, closePos);
  SerialMonitorInterface.println("Closing window...");
  return "CLOSED";
}

// Stop the servo motor
String stopWindow() {
  servo.setServo(servoChannel, 1500); 
  return "STOPPED";
}
