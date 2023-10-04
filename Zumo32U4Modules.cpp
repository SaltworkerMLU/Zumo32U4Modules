#include "Zumo32U4Modules.h" // Functions initialized, and not deinfed, in the .h file are defined here

int Zumo32U4ModulesButtonBuzzer::checkButtonPress() {
  return Zumo32U4ButtonA::isPressed() + 
         Zumo32U4ButtonB::isPressed() * 2 + 
         Zumo32U4ButtonC::isPressed() * 4;
}

int Zumo32U4ModulesButtonBuzzer::getButtonRelease() {
  while (1) { // No buttons have been pressed and released yet.
      if (Zumo32U4ButtonA::getSingleDebouncedRelease() == true) { return 1; }
      else if (Zumo32U4ButtonB::getSingleDebouncedRelease() == true) { return 2; }
      else if (Zumo32U4ButtonC::getSingleDebouncedRelease() == true) { return 3; }
    }
}

void Zumo32U4ModulesButtonBuzzer::buzzer(int frequency=400, int duration=50, int volume=10) {
  Zumo32U4Buzzer::playFrequency(frequency, duration, volume); 
}

int Zumo32U4ModulesButtonBuzzer::buttonBootup(int attention=10, int windup=800) {
  int buttonPress = 0;
  buttonPress = getButtonRelease();
  ledRed(1);    // Turn on red LED
  Zumo32U4Buzzer::playFrequency(400, windup/2, attention); // 400Hz sound, ms/2, 15=max volume & 0 = min volume
  delay(windup/2);                                // Prevent the prior function from being overwritten
  ledYellow(1); // Turn on yellow LED
  Zumo32U4Buzzer::playFrequency(800, windup/2, attention); // 800Hz sound
  delay(windup/2);                                // Prevent the prior function from being overwritten
  ledRed(0);    // Turn off red LED
  ledYellow(0); // Turn off yellow LED
  ledGreen(1);  // Turn on green LED. No need to turn it off. Green LED lights up when Serial monitor active
  Zumo32U4Buzzer::playFrequency(1200, windup, attention);  // 1200Hz lyd
  delayMicroseconds(1); // Frekvens spilles i {windup}. efter 1us eksekveres næste linje i programmet
  return buttonPress;
}

void Zumo32U4ModulesMotors::motorDrive(int left=0, int right=0, bool reverse=false) {
  Zumo32U4ModulesMotors::flipRightMotor(reverse); // IF reverse == true: {right} is reversed
  Zumo32U4ModulesMotors::flipLeftMotor(reverse); // IF reverse == true: {left} is reversed
  if (left==0 && right!=0) { Zumo32U4ModulesMotors::setRightSpeed(right); }
  else if (left!=0 && right==0) { Zumo32U4ModulesMotors::setLeftSpeed(left); }
  else { Zumo32U4ModulesMotors::setSpeeds(left, right); } // Executes setLeftSpeed(int) & setRightSpeed(int) one after another
}

float Zumo32U4ModulesEncoders::motorDistance(bool index) {
  int count; // Omdrejninger = 2*r*PI, hvor r = 2cm, PI = 2*acos(0.0) <=> 2 * 2 * 2*acos(0.0) * count / cpr, hvor cpr = gearRatio * 12 ~ 900 CPR [...]
  if (index == true) { count = Zumo32U4ModulesEncoders::getCountsRight(); }
  if (index == false) { count = Zumo32U4ModulesEncoders::getCountsLeft(); }
  return 8 * acos(0.0) * count / 900; // [...] = 8 * acos(0.0) * count / 900... Kilde på CPR: https://www.pololu.com/docs/0J63/3.4
}

float Zumo32U4ModulesEncoders::motorDistanceReset(bool index) {
  int count;
  if (index == true) { count = Zumo32U4ModulesEncoders::getCountsAndResetRight(); }
  if (index == false) { count = Zumo32U4ModulesEncoders::getCountsAndResetLeft(); }
  return  8 * acos(0.0) * count / 900;
}

Zumo32U4ModulesSensors::Zumo32U4ModulesSensors() {
  Zumo32U4ProximitySensors::initThreeSensors();
  Zumo32U4LineSensors::initThreeSensors();
}

int Zumo32U4ModulesSensors::getProximitySensorValue(bool index) {
  Zumo32U4ProximitySensors::read();
  if (index == true) { return Zumo32U4ProximitySensors::countsFrontWithLeftLeds(); }
  else { return Zumo32U4ProximitySensors::countsFrontWithRightLeds(); }
}

int Zumo32U4ModulesSensors::getLineSensorValue(int index=-1) {
  Zumo32U4LineSensors::read(lineSensorValues, QTR_EMITTERS_ON);
  if (index == 0 || index == 1 || index == 2) { return lineSensorValues[index]; }
}

int16_t Zumo32U4ModulesSensors::getIMUvalue(char m_a_g='_') {
  if (accDataReady() == false) { // If accelerometer is not initialized:
    Wire.begin();
    Zumo32U4IMU::init();          // "Activate" accelerometer a, magnetometer m, gyrometer g på x, y & z-akse
    Zumo32U4IMU::enableDefault(); // Needed when connected via. USB (don't know why)
    Serial.println("IMU initialized!");
  }
  switch (m_a_g) {
    case 'm':
      Zumo32U4IMU::readMag();
      return;
    case 'a':
      Zumo32U4IMU::readAcc();
      return;
    case 'g':
      Zumo32U4IMU::readGyro();
      return;
    default:
      Zumo32U4IMU::read();
  }
}

int16_t Zumo32U4ModulesSensors::calibrate(char m_a_g, int index, int iterations=1000) {
  int32_t total = 0; // Sum of read values
  Wire.begin();                 // Calibration is only needed to be done once in setup...
  Zumo32U4IMU::init();          // thus this IMU initialization is inevitable
  Zumo32U4IMU::enableDefault(); // Needed when connected via. USB (don't know why)
  Zumo32U4IMU::configureForTurnSensing(); // Needed for gyrometer calibration
  switch(m_a_g) {
    case 'm':
      for (int i = 0; i < iterations; i++) {
        while(!magDataReady()) {}
        Zumo32U4IMU::readMag();
        total += *mag[index];
      }
    case 'a':
      for (int i = 0; i < iterations; i++) {
        while(!accDataReady()) {}
        Zumo32U4IMU::readAcc();
        total += *acc[index];
      }
    case 'g':
      for (int i = 0; i < iterations; i++) {
        while(!gyroDataReady()) {}
        Zumo32U4IMU::readGyro();
        total += *gyro[index]; // *gyro[index]
      }
  }
  LastUpdate = micros(); // micros() gives precise time. Used for change in time
  turnAngle = 0;
  return total / iterations; // Average of read values is returned
}

uint32_t Zumo32U4ModulesSensors::dAngle(int index) {
  Zumo32U4IMU::readGyro(); // Only gyrometer is needed for Zumo32U4 angle readings
  int16_t turnRate = acc[index] - gyroOffset;
  uint16_t m = micros(); // "New tid"
  uint16_t dt = m - LastUpdate;
  LastUpdate = m; // "New tid" becomes "old time"
  int32_t d = (int32_t)turnRate * dt; // Angle = Angular_velocity * time
  turnAngle += (int64_t)d * 14680064 / 17578125; // Fancy maths
  return (((uint32_t)turnAngle >> 16) * 360) >> 16; // Even more fancy maths
}

void Zumo32U4Modules::LEDblink(int interval) {
  ledRed(1);    // Turn on red LED
  delay(interval/3);
  ledRed(0);    // Turn off red LED
  ledYellow(1); // Turn on yellow LED
  delay(interval/3);
  ledYellow(0); // Turn off yellow LED
  ledGreen(1);  // Turn on green LED
  delay(interval/3);
  ledGreen(0);  // Turn off green LED
}

void Zumo32U4Modules::IMUEndCondition() {
  Zumo32U4IMU::readAcc();    // Only accelerometer is needed for readings of Zumo32U4 gravitaional pull
  if (a.z < 0 || a.z >= 20000) {    // If (Zumo32U4 is turned upside down || lifted up)
    Zumo32U4ModulesMotors::setSpeeds(0, 0);   // Motors must stop when program stops
    exit(0); // Stop the program in its entierty
  }
}

Zumo32U4ModulesLCD::Zumo32U4ModulesLCD() {
  displayCustomCharacters(forwardArrows, forwardArrowsSolid, reverseArrows, reverseArrowsSolid,
                          leftArrow, rightArrow, backArrow, backArrowReverse); // Default configuration of custom characters
  Zumo32U4LCD::clear();
  Zumo32U4LCD::print(F("Zumo32U4"));
  Zumo32U4LCD::gotoXY(0, 1);
  Zumo32U4LCD::print(F("Modules"));
}

void Zumo32U4ModulesLCD::displayMenu() {
  int battery = readBatteryMillivolts(); // NOTE: 2.7V = 0% og 5.0V = 100%
  Zumo32U4LCD::clear();
  Zumo32U4LCD::print(F("USB: "));
  if (usbPowerPresent() == true) { Zumo32U4LCD::print(F("YES")); } // Is Zumo32U4 connected to another unit?
  else { Zumo32U4LCD::print(F(" NO")); }
  Zumo32U4LCD::gotoXY(0, 1);
  Zumo32U4LCD::print(F("U:"));
  Zumo32U4LCD::gotoXY(6-String(battery).length(), 1);
  Zumo32U4LCD::print((String)battery);
  Zumo32U4LCD::gotoXY(6, 1);
  Zumo32U4LCD::print(F("mV"));
}

void Zumo32U4ModulesLCD::displayPrint(char input[], bool clear=false, bool newLine=true) {
  if (clear==true) {  // Clear display?
    Zumo32U4LCD::clear();
    Zumo32U4Modules::displayLine = 0;
  }
  if (strlen(input) == 0) { Zumo32U4LCD::print(input[0]); } // If {input} == "\0" or {input} == "", then "\0" is printed to display
  else { Zumo32U4LCD::print(input); }
  if (newLine==true) { // Should the next displayPrint(char input[]) be printed on the next line?
    Zumo32U4Modules::displayLine++;
    Zumo32U4LCD::gotoXY(0, Zumo32U4Modules::displayLine);
  }
}

void Zumo32U4ModulesLCD::displayCustomCharacters(char custom0[]={}, char custom1[]={}, char custom2[]={}, char custom3[]={}, 
                                                 char custom4[]={}, char custom5[]={}, char custom6[]={}, char custom7[]={}) {
  Zumo32U4LCD::loadCustomCharacter(custom0, 0);
  Zumo32U4LCD::loadCustomCharacter(custom1, 1);
  Zumo32U4LCD::loadCustomCharacter(custom2, 2);
  Zumo32U4LCD::loadCustomCharacter(custom3, 3);
  Zumo32U4LCD::loadCustomCharacter(custom4, 4);
  Zumo32U4LCD::loadCustomCharacter(custom5, 5);
  Zumo32U4LCD::loadCustomCharacter(custom6, 6);
  Zumo32U4LCD::loadCustomCharacter(custom7, 7);
}

Zumo32U4ModulesOLED::Zumo32U4ModulesOLED() {
  displayCustomCharacters(forwardArrows, forwardArrowsSolid, reverseArrows, reverseArrowsSolid,
                          leftArrow, rightArrow, backArrow, backArrowReverse);  // Default configuration of custom characters
  Zumo32U4OLED::clear();
  Zumo32U4OLED::setLayout8x2();
  Zumo32U4OLED::print(F("Zumo32U4"));
  Zumo32U4OLED::gotoXY(0, 1);
  Zumo32U4OLED::print(F("Modules"));
  Zumo32U4OLED::setLayout21x8();
}

void Zumo32U4ModulesOLED::displayMenu() {
  int battery = readBatteryMillivolts(); // NOTE: 2.7V = 0% og 5.0V = 100%
  Zumo32U4OLED::clear();
  Zumo32U4OLED::setLayout8x2();
  Zumo32U4OLED::gotoXY(0, 0);
  Zumo32U4OLED::print(F("USB: "));
  if (usbPowerPresent() == true) { Zumo32U4OLED::print(F("YES")); } // Is Zumo32U4 connected to another unit?
  else { Zumo32U4OLED::print(F(" NO")); }
  Zumo32U4OLED::gotoXY(0, 1);
  Zumo32U4OLED::print(F("U:"));
  Zumo32U4OLED::gotoXY(6-String(battery).length(), 1);
  Zumo32U4OLED::print((String)battery);
  Zumo32U4OLED::gotoXY(6, 1);
  Zumo32U4OLED::print(F("mV"));
  Zumo32U4OLED::setLayout21x8();
}

void Zumo32U4ModulesOLED::displayPrint(char input[], bool clear=false, bool newLine=true) {
  if (clear==true) {  // Clear display?
    Zumo32U4OLED::clear();
    Zumo32U4Modules::displayLine = 0;
  }

  if (strlen(input) == 0) { Zumo32U4OLED::print(input[0]); } // If {input} == "\0" or {input} == "", then "\0" is printed to display
  else { Zumo32U4OLED::print(input); }

  if (newLine==true) { // Should the next displayPrint(char input[]) be printed on the next line?
    Zumo32U4Modules::displayLine++;
    Zumo32U4OLED::gotoXY(0, Zumo32U4Modules::displayLine);
  }
}

void Zumo32U4ModulesOLED::displayCustomCharacters(char custom0[]={}, char custom1[]={}, char custom2[]={}, char custom3[]={}, 
                                                  char custom4[]={}, char custom5[]={}, char custom6[]={}, char custom7[]={}) {
  Zumo32U4OLED::loadCustomCharacter(custom0, 0);
  Zumo32U4OLED::loadCustomCharacter(custom1, 1);
  Zumo32U4OLED::loadCustomCharacter(custom2, 2);
  Zumo32U4OLED::loadCustomCharacter(custom3, 3);
  Zumo32U4OLED::loadCustomCharacter(custom4, 4);
  Zumo32U4OLED::loadCustomCharacter(custom5, 5);
  Zumo32U4OLED::loadCustomCharacter(custom6, 6);
  Zumo32U4OLED::loadCustomCharacter(custom7, 7);
}
