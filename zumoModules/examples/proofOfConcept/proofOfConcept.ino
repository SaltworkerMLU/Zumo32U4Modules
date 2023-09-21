#include <zumoModules.h>

zumoModules zumoBot;

bool bakgear = true;
int protocol;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  zumoBot.sensorBootup();
  zumoBot.OLEDdisplayMenu("LEDBlink", "Read LineSensor", "Read Encoder");
  protocol = zumoBot.buttonBootup();
}

void loop() {
  // put your main  code here, to run repeatedly:
  switch(protocol) {
    case 1: {
      zumoBot.LEDblink(300);
      break;
    }
    case 2: {
      zumoBot.getLineSensorValue();
      Serial.print(zumoBot.lineSensorValues[0]);
      Serial.print("\t" + (String)zumoBot.lineSensorValues[1]);
      Serial.println("\t" + (String)zumoBot.lineSensorValues[2]);
      zumoBot.OLED.clear();
      zumoBot.OLED.gotoXY(2, 2);
      zumoBot.OLED.print(zumoBot.lineSensorValues[0]);
      zumoBot.OLED.gotoXY(8, 2);
      zumoBot.OLED.print(zumoBot.lineSensorValues[1]);
      zumoBot.OLED.gotoXY(14, 2);
      zumoBot.OLED.print(zumoBot.lineSensorValues[2]);
      delay(50);
      break;
    }
    case 3: {
      if (abs(zumoBot.motorOmdrejninger(0)) > 10)  {Serial.println(zumoBot.motorOmdrejningerReset(0)); }
      else { Serial.println(zumoBot.motorOmdrejninger(0)); }
      zumoBot.motorDrive(200);
      delay(500);
      zumoBot.motorReverse(bakgear);
      bakgear = !bakgear;
      zumoBot.gyroEndCondition();
      break;
    }
    default:
      zumoBot.OLED.gotoXY(7, 0);
      zumoBot.OLED.print(F("Error"));
      zumoBot.OLED.gotoXY(1, 1);
      zumoBot.OLED.print(F("Press RESET button"));
      delay(1000);
  }
}