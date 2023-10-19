#include <Zumo32U4Modules.h>
Zumo32U4ModulesOLED zumoBot;

int protocol;
bool bakgear = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  zumoBot.initIMU();
  zumoBot.displayMenu();
  protocol = zumoBot.buttonBootup();
}
void loop() {
  // put your main code here, to run repeatedly:
  switch(protocol) {
    case 1: {
      zumoBot.LEDblink(300);
      zumoBot.getIMUvalue('a');
      Serial.println(*zumoBot.acc[2]);
      zumoBot.displayPrint((String)*zumoBot.acc[2], true, false);
      break;
    }
    case 2: {
      zumoBot.getLineSensorValue();
      Serial.println((String)zumoBot.lineSensorValues[0] + "\t" + (String)zumoBot.lineSensorValues[1] + "\t" + (String)zumoBot.lineSensorValues[2]);
      zumoBot.displayPrint("  " + (String)zumoBot.lineSensorValues[0], true, true);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[2], false, false);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[1], false, false);
      zumoBot.LEDblink(300);
      break;
    }
    case 3: {
      if (abs(zumoBot.motorDistance(0)) > 10)  {Serial.println(zumoBot.motorDistanceReset(0)); }
      if (abs(zumoBot.motorDistance(1)) > 10)  {Serial.println(zumoBot.motorDistanceReset(1)); }
      zumoBot.motorDrive(200, 0, bakgear);
      zumoBot.LEDblink(500);
      bakgear = !bakgear;
      zumoBot.motorDrive(200, 0, bakgear);
      zumoBot.IMUEndCondition();
      Serial.println(*zumoBot.acc[2]);
      Serial.println((String)zumoBot.motorDistance(0) + "\t" + (String)zumoBot.motorDistance(1));
      zumoBot.displayPrint((String)zumoBot.motorDistance(0), true, true);
      zumoBot.displayPrint((String)zumoBot.motorDistance(1), false, false);
      break;
    }
    default:
      zumoBot.displayPrint("Press", false, true);
      zumoBot.displayPrint(">RESET<", true, false);
      zumoBot.LEDblink(100);
  }
}