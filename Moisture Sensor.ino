/************************************************************************
 * Soil Moisture Sensor Wireling Example Sketch
 * This program uses one of the Wirelings included with the Starter Kit:
 * Port 0: Soil Moisture Sensor Wireling
 * 
 * This program will display the temperature and moisture detected by the
 * soil moisture sensor Wireling on the serial monitor.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose and Hunter Hykes for TinyCircuits
 *
 * Initiated: 12/26/2019 
 * Updated: 01/05/2021
 ************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Wireling.h>
#include <ATtiny25.h>      // For ATtiny25 sensor

// Make compatible with all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/* * * * * * MOISTURE SENSOR * * * * * * */
#define MOISTURE_PORT 0 //the soil moisture sensor is connected to Port 0 on the TinyCircuits board.
ATtiny25 moisture_sensor; //creates an object representing the soil moisture sensor. You use this object to read moisture and temperature.

//Motor 
#define MOTOR_PORT 1
#define MOTOR_PIN 0
#define WET_THRESHOLD 95

void setup() {
  Wire.begin();
  Wireling.begin();
  delay(10);
  Wireling.selectPort(MOISTURE_PORT); //tells the board which port the sensor is connected to.
  SerialMonitorInterface.begin(9600);

  Wireling.selectPort(MOTOR_PORT); //tells the board which port the motor is connected to.
}

void loop() {
  int moisture = moisture_sensor.readMoisture(); 

  SerialMonitorInterface.print("M: ");
  SerialMonitorInterface.print(moisture_sensor.readMoisture()); //moisture_sensor.readMoisture() → reads the moisture level from the soil sensor.
  SerialMonitorInterface.print("\tT: ");
  SerialMonitorInterface.println(moisture_sensor.readTemp()); //moisture_sensor.readTemp() → reads the temperature from the sensor.
  delay(1000);
}
