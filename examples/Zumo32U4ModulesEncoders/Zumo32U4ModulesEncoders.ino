/*
 * Dette program viser hvordan klassen Zumo32U4ModulesEncoders benyttes.
 * Der tjekkes om motorOmdrejninger({bool}) returnerer en værdi over 10cm for begge mulige parametre.
 * Såfremt betingelsen opfyldes for en eller begge encoders, 
 * printes omdrejningerne en sidste gang hvorefter de respektive encoders resettes.
 */

#include "Zumo32U4Modules.h"
Zumo32U4ModulesEncoders zumoBot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (abs(zumoBot.motorOmdrejninger(0)) > 10) { Serial.print((String)zumoBot.motorOmdrejningerReset(0) + "\t"); }
  else { Serial.print((String)zumoBot.motorOmdrejninger(0) + "\t"); }
  if (abs(zumoBot.motorOmdrejninger(1)) > 10) { Serial.println(zumoBot.motorOmdrejningerReset(1)); }
  else { Serial.println(zumoBot.motorOmdrejninger(1)); }
  delay(100);
}