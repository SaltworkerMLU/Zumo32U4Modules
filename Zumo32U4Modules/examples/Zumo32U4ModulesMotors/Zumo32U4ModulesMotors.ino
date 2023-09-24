/*
 * Dette program viser hvordan klassen Zumo32U4ModulesMotros benyttes.
 * Der køres med tilfældige hastigheder med de 2 motorer vha. funktionen motorDrive()
 * Efterfølgende køres der tilbage til startpositionen ved at vende om på værdierne.
 */

#include "Zumo32U4Modules.h"
Zumo32U4ModulesMotors zumoBot;

int left, right;
bool bakgear = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  left = random(-400, 401); // Motorer kan gives værdier fra -400 til 400
  right = random(-400, 401);
  bakgear = !bakgear;
  zumoBot.motorDrive(left, right, bakgear);
  Serial.println((String)left + "\t" + (String)right);
  delay(1000);
  bakgear = !bakgear;
  zumoBot.motorDrive(left, right, bakgear);
  Serial.println((String)-left + "\t" + (String)-right);
  delay(1000);
}