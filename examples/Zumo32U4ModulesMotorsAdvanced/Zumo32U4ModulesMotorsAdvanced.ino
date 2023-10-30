/*
 * This program shows how to make use of the other, more advanced, Zumo32U4 motor functions
 * Either try PIDLineFollower() by pressing button A,
 * read the angle of Zumo32U4 based on gyrometer,
 * or set Zumo32U4 to drive 10cm/s (NOTE: inconsitencies can be adjusted with e.g. zumoBot.CPR) 
 * and read proximity sensors, motor velocity, and acceleration.
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesLCD zumoBot;

int32_t angle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  zumoBot.displayMenu();
  zumoBot.calibrateLineSensor();
  zumoBot.initIMU();
  zumoBot.gyroOffset = zumoBot.calibrateIMU('g', 2, 1000);
  zumoBot.buttonBootupSound();
  zumoBot.displayPrint("", true, false); // Simply clear display
}

void loop() {
  switch(zumoBot.buttonRelease) {
    case 'A':
      zumoBot.PIDLineFollower(1, 0, 0, 200, true, true, true); // Runs in a loop. If Zumo32U4 is turned upside down, the program ends due to IMUEndCondition().
      zumoBot.getLineSensorValueCalibrated(); // Whilst PIDLineFollower() is not running, both proximity sensors read max value, print line sensor values calibrated...
      Serial.println((String)zumoBot.lineSensorValue[0] + "\t" + (String)zumoBot.lineSensorValue[1] + "\t" + 
                     (String)zumoBot.lineSensorValue[2] + "\t" + (String)zumoBot.lineSensorValue[3] + "\t" + 
                     (String)zumoBot.lineSensorValue[4]);
      zumoBot.getLineSensorValue(); // ... and uncalibrated
      Serial.println((String)zumoBot.lineSensorValue[0] + "\t" + (String)zumoBot.lineSensorValue[1] + "\t" + 
                     (String)zumoBot.lineSensorValue[2] + "\t" + (String)zumoBot.lineSensorValue[3] + "\t" + 
                     (String)zumoBot.lineSensorValue[4]);
      zumoBot.displayPrint((String)zumoBot.lineSensorValue[1], true);
      zumoBot.displayPrint((String)zumoBot.lineSensorValue[3]);
      delay(100);
      break;
    case 'B':
      angle = zumoBot.gyroAngle(2);
      Serial.println(angle);
      zumoBot.displayLine = -1; // Neat trick: Because dAngle() is intended to run without any form of delay and without clearing the display, simply overwriting previous input will display the correct value.
      zumoBot.displayPrint((String)angle); // Since going to next line sets the display x-axis to 0, this is done to overwrite previous value.
      break;
    case 'C':
      zumoBot.getProximitySensorValue();
      zumoBot.setMotorVelocity(10, 10); // No need to update motor acceleration nor velocity as this function does both
      Serial.println("PROXY: " + (String)zumoBot.proximitySensorValue[0] + "\t" + zumoBot.proximitySensorValue[1] + "\t" + 
                     "VEL: " + zumoBot.motorVelocity[0] + "\t" + zumoBot.motorVelocity[1] + "\t" + 
                     "ACC: " + zumoBot.motorAcceleration[0] + "\t" + zumoBot.motorAcceleration[1]);
      zumoBot.displayPrint((String)zumoBot.motorVelocity[0], true);
      zumoBot.displayPrint((String)zumoBot.motorVelocity[1]);
      delay(100);
      break;
  }
}