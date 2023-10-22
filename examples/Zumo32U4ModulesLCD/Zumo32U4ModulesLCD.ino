#include <Zumo32U4Modules.h>
Zumo32U4ModulesLCD zumoBot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  zumoBot.initIMU();
  zumoBot.displayMenu();
  zumoBot.buttonBootup();
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(zumoBot.buttonRelease) {
    case 'A': {
      zumoBot.LEDblink(300);
      zumoBot.getIMUvalue('a');
      Serial.println(*zumoBot.acc[2]);
      String displayInput = (String)*zumoBot.acc[2]; // Value must be loaded in string buffer to be printed on display
      zumoBot.displayPrint(displayInput, true, false);
      break;
    }
    case 'B': {
      zumoBot.getLineSensorValue();
      Serial.println((String)zumoBot.lineSensorValues[0] + "\t" + (String)zumoBot.lineSensorValues[1] + "\t" + 
                     (String)zumoBot.lineSensorValues[2] + "\t" + (String)zumoBot.lineSensorValues[3] + "\t" + 
                     (String)zumoBot.lineSensorValues[4]);
      zumoBot.displayPrint("  " + (String)zumoBot.lineSensorValues[2], true, true);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[3], false, false);
      zumoBot.displayPrint((String)zumoBot.lineSensorValues[1], false, false);
      zumoBot.LEDblink(300);
      break;
    }
    case 'C': {
      if (abs(zumoBot.motorDistance(0)) > 10)  {Serial.println(zumoBot.motorDistanceReset(0)); }
      if (abs(zumoBot.motorDistance(1)) > 10)  {Serial.println(zumoBot.motorDistanceReset(1)); }
      zumoBot.motorDrive(200, 0);
      zumoBot.LEDblink(500);
      zumoBot.reverse = !zumoBot.reverse;
      zumoBot.motorDrive(200, 0);
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