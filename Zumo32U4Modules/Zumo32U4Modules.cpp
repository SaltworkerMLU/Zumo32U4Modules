#include "Zumo32U4ModulesExperiment.h" // I .cpp filen definéres funktionerne initialiseret i .h filen

int Zumo32U4ModulesButtonBuzzer::buttonBootup(int attention=10, int windup=800) {
  int buttonPress = 0;
  while ( // Ingen knapper er blevet trykket endnu
    !Zumo32U4ButtonA::isPressed() &&
    !Zumo32U4ButtonB::isPressed() && 
    !Zumo32U4ButtonC::isPressed() ) {}
  if (Zumo32U4ButtonA::isPressed() == true) { buttonPress = 1; }
  else if (Zumo32U4ButtonB::isPressed() == true) { buttonPress = 2; }
  else if (Zumo32U4ButtonC::isPressed() == true) { buttonPress = 3; }
  ledRed(1);    // Tænd rød LED
  Zumo32U4Buzzer::playFrequency(400, windup/2, attention); // 400Hz lyd, ms/2, 15=max volume & 0 = min volume
  delay(windup/2);                                // Undgå overwriting af ovenstående funktion
  ledYellow(1); // Tænd gul LED
  Zumo32U4Buzzer::playFrequency(800, windup/2, attention); // 800Hz lyd
  delay(windup/2);                                // Undgå overwriting af ovenstående funktion
  ledRed(0);    // Tænd rød LED
  ledYellow(0); // Tænd gul LED
  ledGreen(1);  // Tænd grøn LED. Ikke sluk den. Lyser når serial monitor er aktiv
  Zumo32U4Buzzer::playFrequency(1200, windup, attention);  // 1200Hz lyd
  delayMicroseconds(1); // Frekvens spilles i {windup}. efter 1us eksekveres næste linje i programmet
  return buttonPress;
}

void Zumo32U4ModulesMotors::motorDrive(int left=0, int right=0) {
  if (left==0 && right!=0) { Zumo32U4ModulesMotors::setRightSpeed(right); }
  else if (left!=0 && right==0) { Zumo32U4ModulesMotors::setLeftSpeed(left); }
  else { Zumo32U4ModulesMotors::setSpeeds(left, right); } // Eksekverer setLeftSpeed(int) & setRightSpeed(int) samtidigt
}

void Zumo32U4ModulesMotors::motorReverse(bool bakgear) {
  Zumo32U4ModulesMotors::flipRightMotor(bakgear);
  Zumo32U4ModulesMotors::flipLeftMotor(bakgear);
}

float Zumo32U4ModulesEncoders::motorOmdrejninger(bool index) {
  int count; // Omdrejninger = 2*r*PI, hvor r = 2cm, PI = 2*acos(0.0) <=> 2 * 2 * 2*acos(0.0) * count / cpr, hvor cpr = gearRatio * 12 ~ 900 CPR [...]
  if (index == true) { count = Zumo32U4ModulesEncoders::getCountsRight(); }
  if (index == false) { count = Zumo32U4ModulesEncoders::getCountsLeft(); }
  return 8 * acos(0.0) * count / 900; // [...] = 8 * acos(0.0) * count / 900... Kilde på CPR: https://www.pololu.com/docs/0J63/3.4
}

float Zumo32U4ModulesEncoders::motorOmdrejningerReset(bool index) {
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
  switch (index) {
    case 0 && 1 && 2: // Hvis der gives en valid værdi...
      return lineSensorValues[index];
    default: // Ellers...
      Zumo32U4LineSensors::read(lineSensorValues, QTR_EMITTERS_ON);
  }
}

int16_t Zumo32U4ModulesSensors::getIMUvalue(char mag='_') {
  if (accDataReady() == false) { // Hvis accelerometer ikke er initialiseret:
    Wire.begin();                 // Skal benyttes inden IMU.init()
    Zumo32U4IMU::init();          // "Aktivér" accelerometer a, magnetometer m, gyrometer g på x, y & z-akse
    Zumo32U4IMU::enableDefault(); // Skal benyttes når ikke forbundet til USB (don't know why)
    Serial.println("IMU initialiseret!");
  }
  switch (mag) {
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

void Zumo32U4Modules::LEDblink(int interval) {
  ledRed(1);    // Tænd rød LED
  delay(interval/3);
  ledRed(0);    // Sluk rød LED
  ledYellow(1); // Tænd gul LED
  delay(interval/3);
  ledYellow(0); // Sluk gul LED
  ledGreen(1);  // Tænd grøn LED
  delay(interval/3);
  ledGreen(0);  // Sluk grøn LED
}

void Zumo32U4Modules::IMUEndCondition() {
  Zumo32U4IMU::read();    // Aflæs IMU (accelerometer z-akse inkluderer tyngdekraft g)
  if (a.z < 0 || a.z >= 20000) {    // Hvis (Zumo vendes på hoved || løftes op hurtigt)
    Zumo32U4ModulesMotors::setSpeeds(0, 0);   // Sørg for motorer stopper når programmet stopper
    exit(0); // Afslut programmet
  }
}

Zumo32U4ModulesLCD::Zumo32U4ModulesLCD() {
  Zumo32U4LCD::clear();
  Zumo32U4LCD::print(F("Zumo32U4"));
  Zumo32U4LCD::gotoXY(0, 1);
  Zumo32U4LCD::print(F("Modules"));
}

void Zumo32U4ModulesLCD::LCDDisplayMenu() {
  int battery = readBatteryMillivolts(); // NOTE: 2.7V = 0% og 5.0V = 100%
  Zumo32U4LCD::clear();
  Zumo32U4LCD::print(F("USB: "));
  if (usbPowerPresent() == true) { Zumo32U4LCD::print(F("YES")); } // Er Zumo forbundet til seperat enhed?
  else { Zumo32U4LCD::print(F(" NO")); }
  Zumo32U4LCD::gotoXY(0, 1);
  Zumo32U4LCD::print(F("U:"));
  Zumo32U4LCD::gotoXY(6-String(battery).length(), 2);
  Zumo32U4LCD::print((String)battery);
  Zumo32U4LCD::gotoXY(6, 1);
  Zumo32U4LCD::print(F("mV"));
}

void Zumo32U4ModulesLCD::LCDprint(String input, bool newLine=false, bool clear=false) {
  if (newLine==true) { // Skal {input} printes på næste linje?
    Zumo32U4Modules::displayLine++;
    Zumo32U4LCD::gotoXY(0, Zumo32U4Modules::displayLine);
  }
  if (clear==true) {  // Skal LCD-display ryddes?
    Zumo32U4LCD::clear(); 
    Zumo32U4Modules::displayLine = 0;
  }
  Zumo32U4LCD::print(input);
}

Zumo32U4ModulesOLED::Zumo32U4ModulesOLED() {
  Zumo32U4OLED::clear();
  Zumo32U4OLED::setLayout8x2();
  Zumo32U4OLED::print(F("Zumo32U4"));
  Zumo32U4OLED::gotoXY(0, 1);
  Zumo32U4OLED::print(F("Modules"));
}

void Zumo32U4ModulesOLED::OLEDDisplayMenu() {
  int battery = readBatteryMillivolts(); // NOTE: 2.7V = 0% og 5.0V = 100%
  Zumo32U4OLED::clear();
  Zumo32U4OLED::setLayout11x4();
  Zumo32U4OLED::gotoXY(1, 1);
  Zumo32U4OLED::print(F("USB: "));
  if (usbPowerPresent() == true) { Zumo32U4OLED::print(F("YES")); } // Er Zumo forbundet til seperat enhed?
  else { Zumo32U4OLED::print(F(" NO")); }
  Zumo32U4OLED::gotoXY(1, 2);
  Zumo32U4OLED::print(F("U:"));
  Zumo32U4OLED::gotoXY(7-String(battery).length(), 2);
  Zumo32U4OLED::print((String)battery);
  Zumo32U4OLED::gotoXY(7, 2);
  Zumo32U4OLED::print(F("mV"));
}

void Zumo32U4ModulesOLED::OLEDprint(String input, bool newLine=false, bool clear=false) {
  if (newLine==true) { // Skal {input} printes på næste linje?
    Zumo32U4Modules::displayLine++;
    Zumo32U4OLED::gotoXY(0, Zumo32U4Modules::displayLine);
  }
  if (clear==true) {  // Skal OLED-display ryddes?
    Zumo32U4OLED::clear();
    Zumo32U4Modules::displayLine = 0;
  }
  Zumo32U4OLED::print(input);
}
