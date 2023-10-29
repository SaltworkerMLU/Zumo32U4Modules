/*
 * This program shows how to use the class Zumo32U4ModulesEncoders.
 * Print distance, velocity and acceleration of both encoders/motors.
 * Check whether motorDistance({bool}) returns a value above 10cm for each encoder. If so, reset either encoder
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesEncoders zumoBot;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //zumoBot.CPR = 900; // Adjust this value if distance in cm is inprecise
}

void loop() {
  // put your main code here, to run repeatedly:
  //zumoBot.getMotorVelocity(); // Get only Zumo32U4 motor velocity
  zumoBot.getMotorAcceleration(); // Get both Zumo32U4 motor velocity and acceleration
  Serial.println("Distance: " + (String)zumoBot.motorDistance[0] + "\t" + (String)zumoBot.motorDistance[0] + 
                "\tVelocity: " + zumoBot.motorVelocity[0] + "\t" + zumoBot.motorVelocity[1] + 
                "\tAcceleration: " + zumoBot.motorAcceleration[0] + "\t" + zumoBot.motorAcceleration[1]);
  if (abs(zumoBot.motorDistance[0]) > 10 || abs(zumoBot.motorDistance[1]) > 10) { zumoBot.getMotorDistanceReset(); } // When distance reaches above 10cm, reset the encoder value surpassing 10cm
  delay(100);
}