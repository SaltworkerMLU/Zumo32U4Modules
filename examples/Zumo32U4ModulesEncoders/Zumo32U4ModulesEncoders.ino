/*
 * This program shows how to use the class Zumo32U4ModulesEncoders
 * Check whether motorDistance({bool}) returns a value above 10cm for each encoder.
 * If this condition is met for either encoder, this encoder value is printed one last time and then reset.
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesEncoders zumoBot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (abs(zumoBot.motorDistance(0)) > 10) { Serial.print((String)zumoBot.motorDistanceReset(0) + "\t"); }
  else { Serial.print((String)zumoBot.motorDistance(0) + "\t"); }
  if (abs(zumoBot.motorDistance(1)) > 10) { Serial.println(zumoBot.motorDistanceReset(1)); }
  else { Serial.println(zumoBot.motorDistance(1)); }
  delay(100);
}