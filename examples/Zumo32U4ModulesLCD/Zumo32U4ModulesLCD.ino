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
      delay(300);
      zumoBot.getIMUvalue('a');
      Serial.println(*zumoBot.acc[2]);
      String displayInput = (String)*zumoBot.acc[2]; // Value must be loaded in string buffer to be printed on display
      zumoBot.displayPrint(displayInput, true, false);
      break;
    }
    case 'B': {
      zumoBot.getLineSensorValue();
      Serial.println((String)zumoBot.lineSensorValue[0] + "\t" + (String)zumoBot.lineSensorValue[1] + "\t" + 
                     (String)zumoBot.lineSensorValue[2] + "\t" + (String)zumoBot.lineSensorValue[3] + "\t" + 
                     (String)zumoBot.lineSensorValue[4]);
      zumoBot.displayPrint("  " + (String)zumoBot.lineSensorValue[2], true, true);
      zumoBot.displayPrint((String)zumoBot.lineSensorValue[3], false, false);
      zumoBot.displayPrint((String)zumoBot.lineSensorValue[1], false, false);
      delay(300);
      break;
    }
    case 'C': {
      zumoBot.getMotorDistance();
      if (abs(zumoBot.motorDistance[0]) > 10 || abs(zumoBot.motorDistance[1]) > 10) {zumoBot.getMotorDistanceReset(); } // Resets motor encoders
      zumoBot.motorDrive(200, 0);
      Serial.println((String)zumoBot.motorDistance[0] + "\t" + (String)zumoBot.motorDistance[1] + "\t" + (String)*zumoBot.acc[2]);
      zumoBot.displayPrint((String)zumoBot.motorDistance[0], true, true);
      zumoBot.displayPrint((String)zumoBot.motorDistance[1], false, false);
      delay(500);
      zumoBot.motorFlip();
      zumoBot.IMUEndCondition();
      break;
    }
    default:
      zumoBot.displayPrint("Press", false, true);
      zumoBot.displayPrint(">RESET<", true, false);
      delay(100);
  }
}