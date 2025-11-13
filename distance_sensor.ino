/*************************************************************************
 * Time of Flight Distance Sensor Wireling Example
 * This example shows how to use continuous mode to take, and print
 * range measurements with the VL53L0X Wireling.
 *
 * The range readings are in units of mm.
 *
 * Hardware by: TinyCircuits
 * Modified by: Laveréna Wienclaw for TinyCircuits
 * Last Modified: 12/18/19
 *************************************************************************/

#include <Wire.h>
#include "VL53L0X.h"
#include <Wireling.h>

VL53L0X distanceSensor;
const int tofPort = 3;
const int averageCount = 1;
int average[averageCount];
int averagePos = 0;

#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

void setup() {
  delay(200);
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  
  // Enable power & select port
  Wireling.begin();
  Wireling.selectPort(tofPort);
  
  // Initialize the distance sensor and set a timeout
  distanceSensor.init();
  distanceSensor.setTimeout(500);
  distanceSensor.setMeasurementTimingBudget(200000);
  distanceSensor.startContinuous();
}

void loop() {
  // Calculate the average position of the distance sensor
  unsigned long averageRead = 0;
  average[averagePos] = distanceSensor.readRangeContinuousMillimeters();
  averagePos++;
  if (averagePos >= averageCount) averagePos = 0;
  for (int i = 0; i < averageCount; i++) {
    averageRead += (unsigned long)average[i];
  }
  averageRead /= (unsigned long)averageCount;
  
  // Print the average position to the Serial Monitor
  SerialMonitorInterface.print(averageRead);
  SerialMonitorInterface.println("mm");
}