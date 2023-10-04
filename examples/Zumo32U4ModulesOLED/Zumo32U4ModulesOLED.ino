#include <Zumo32U4Modules.h>
Zumo32U4ModulesOLED zumoBot;

int protocol;
bool bakgear = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
      zumoBot.displayPrint((String)*zumoBot.acc[2], 0, false, true);
      break;
    }
    case 2: {
      zumoBot.getLineSensorValue();
      Serial.println((String)zumoBot.lineSensorValues[0] + "\t" + (String)zumoBot.lineSensorValues[1] + "\t" + (String)zumoBot.lineSensorValues[2]);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[0], 0, false, true);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[2], 4, true, false);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[1], 2, false, false);
      zumoBot.LEDblink(300);
      break;
    }
    case 3: {
      if (abs(zumoBot.motorOmdrejninger(0)) > 10)  {Serial.println(zumoBot.motorOmdrejningerReset(0)); }
      if (abs(zumoBot.motorOmdrejninger(1)) > 10)  {Serial.println(zumoBot.motorOmdrejningerReset(1)); }
      zumoBot.motorDrive(200);
      zumoBot.LEDblink(500);
      zumoBot.motorReverse(bakgear);
      bakgear = !bakgear;
      zumoBot.IMUEndCondition();
      Serial.println((String)zumoBot.motorOmdrejninger(0) + "\t" + (String)zumoBot.motorOmdrejninger(1));
      zumoBot.displayPrint((String)zumoBot.motorOmdrejninger(0), 2, true, true);
      zumoBot.displayPrint((String)zumoBot.motorOmdrejninger(1), 0, false, false);
      break;
    }
    default:
      zumoBot.displayPrint("Press", 1, false, true);
      zumoBot.displayPrint(">RESET<", 0,  true, false);
      zumoBot.LEDblink(100);
  }
}