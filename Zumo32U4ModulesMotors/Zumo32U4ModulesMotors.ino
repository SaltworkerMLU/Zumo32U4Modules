/*
 * This program shows how to use the class Zumo32U4ModulesMotors
 * The 2 motors are set to different speeds using random()
 * Aterwards, the ZUmo32U4 drives back to its original position
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesMotors zumoBot;

int left, right;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  left = random(-400, 401); // The motors take input(s) from -400 to 400
  right = random(-400, 401);
  zumoBot.motorFlip();
  zumoBot.motorDrive(left, right);
  Serial.println((String)left + "\t" + (String)right);
  delay(1000);
  zumoBot.motorFlip();
  zumoBot.motorDrive(left, right);
  Serial.println((String)-left + "\t" + (String)-right);
  delay(1000);
}