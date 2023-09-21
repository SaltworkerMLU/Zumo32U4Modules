#include "zumoModules.h" // I .cpp filen definéres funktionerne initialiseret i .h filen

void zumoModules::LEDblink(int interval) {
  /*
  Blink med rød, gul & grøn i intervaller svarende til {interval}
  Se funktionen som en mere fancy delay() funktion
  */
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

void zumoModules::OLEDdisplayMenu(String A, String B, String C) {
  /*
  Vis diverse informationer såsom:
    - Om Zumo er forbundet til seperat enhed
    - Batteriniveau
  */
  float battery = readBatteryMillivolts();
  OLED.clear();
  OLED.setLayout21x8();
  OLED.print(F("Hello World! "));
  OLED.gotoXY(0, 1);
  if (usbPowerPresent() == true) { OLED.print(F("CONNECTED")); }
  else { OLED.print(F("DISCONNECTED")); }
  OLED.gotoXY(0, 2);
  OLED.print(battery);
  OLED.print(F("mV "));
  if (battery < 2700) { battery = 2700; }
  if (battery > 5000) { battery = 5000; }
  OLED.print(map(battery, 2700, 5000, 0, 100)); // 2.7V = 0% og 5.0V = 100%
  OLED.print(F("%"));
  OLED.gotoXY(0, 4);
  OLED.print(F("Press any button..."));
  OLED.gotoXY(0, 5);
  OLED.print(F("A: "));
  OLED.print(A);
  OLED.gotoXY(0, 6);
  OLED.print(F("B: "));
  OLED.print(B);
  OLED.gotoXY(0, 7);
  OLED.print(F("C: "));
  OLED.print(C);
}

int zumoModules::buttonBootup(int attention=10, int windup=800) {
  /*
  Vent til hverken af knapperne A, B eller C trykkes og slippes. Vent derefter {windup} tid. 
  Under windup spilles lyd med lydstyrke {attention} og blinkes med rød, gul og grøn LED i trafiklys sekvens
  Ved den tredje lyd som spilles, er funktionen færdig men spiller stadig lyden i {windup} tid

  Returnerer den knap som der trykkes hvor A = 1, B = 2, C = 3
  */
  int buttonPress = 0;
  while ( // Ingen knapper er blevet trykket endnu
    !buttonA.isPressed() &&
    !buttonB.isPressed() && 
    !buttonC.isPressed() ) {}
  if (buttonA.isPressed() == true) { buttonPress = 1; }
  else if (buttonB.isPressed() == true) { buttonPress = 2; }
  else if (buttonC.isPressed() == true) { buttonPress = 3; }
  ledRed(1);    // Tænd rød LED
  buzzer.playFrequency(400, windup/2, attention); // 400Hz lyd, ms/2, 15=max volume & 0 = min volume
  delay(windup/2);                                // Undgå overwriting af ovenstående funktion
  ledYellow(1); // Tænd gul LED
  buzzer.playFrequency(800, windup/2, attention); // 800Hz lyd
  delay(windup/2);                                // Undgå overwriting af ovenstående funktion
  ledRed(0);    // Tænd rød LED
  ledYellow(0); // Tænd gul LED
  ledGreen(1);  // Tænd grøn LED. Ikke sluk den. Lyser når serial monitor er aktiv
  buzzer.playFrequency(1200, windup, attention);  // 1200Hz lyd
  delayMicroseconds(1); // Frekvens spilles i {windup}. efter 1us eksekveres næste linje i programmet
  return buttonPress;
}

void zumoModules::sensorBootup(bool line_sensors=true, bool IMU=true, bool prox_sensors=true) {
  /* 
  Initialisér sensorerne som de står under parametrene afhængigt af deres boolean
  */
  if (line_sensors == true) { lineSensors.initThreeSensors(); }
  if (IMU == true) {
    Wire.begin();         // Skal benyttes inden IMU.init()
    imu.init();           // Gør det muligt at måle accelerometer a, magnetometer m, gyrometer g på x, y & z-akse
    imu.enableDefault();  // Skal benyttes når ikke forbundet til USB (don't know why)
  }
  if (prox_sensors == true) { proxSensors.initThreeSensors(); }
}

int zumoModules::getLineSensorValue(int index=-1) {
  /*
  Gives parameter fra 0-2: Udvind et element fra {lineSensorValues}
  Ellers: Aflæs blot {lineSensor}. I så fald kan hele array tilgås i .ino
  */
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  switch (index) {
    case 0 && 1 && 2: // Hvis der gives en valid værdi...
    return lineSensorValues[index];
  } // ... Ellers intet
}

int zumoModules::getProximitySensorValue(bool index) {
  /*
  Udvind ét element fra {proxSensorValues}.
  */
  proxSensors.read();
  if (index == true) { return proxSensors.countsFrontWithLeftLeds(); }
  else { return proxSensors.countsFrontWithRightLeds(); }
}

void zumoModules::gyroEndCondition() {
  /*
  Måder at vurdere om Zumo skal stoppe programmet:
    1. Hvis Zumo vendes på hovedet (imu.a.z < 0)
    2. Hvis Zumo løftes op hurtigt (imu.a.z >= 20000)
  */
  imu.read();                             // Aflæs IMU (accelerometer z-akse inkluderer tyngdekraft g)
  if (imu.a.z < 0 || imu.a.z >= 20000) {  // Hvis (Zumo vendes på hoved || løftes op hurtigt)
    motors.setSpeeds(0, 0);               // Sørg for motorer stopper når programmet stopper
    exit(0); // Afslut programmet
  }
}

void zumoModules::motorDrive(int left=0, int right=0) {
  /*
  Kør med Zumo mere proceskraftventligt.
  OVERFLOW POLYMORFI:
    - 0 parametre: Motorer stopper
    - 1 parameter: Venstre motor kører
    - 2 parametre: Begge motorer kører
  */
  if (left==0 && right!=0) { motors.setRightSpeed(right); }
  else if (left!=0 && right==0) { motors.setLeftSpeed(left); }
  else { motors.setSpeeds(left, right); }
}

void zumoModules::motorReverse(bool bakgear) {
  /*
  Basically skal Zumo motorer i {bakgear}?
  Dette gøres med funktioner som kræver mindre proceskraft
  */
  motors.flipRightMotor(bakgear);
  motors.flipLeftMotor(bakgear);
}

float zumoModules::motorOmdrejninger(bool index) {
  /*
  Unvind omdrejninger antal motor omdrejninger baseret på formel:
  O = 2*r*PI, hvor r = 2cm, PI = 2*acos(0.0)
  Hvor clicks per rotation = gearRatio * 12 ~ 910 CPR. Altså:
  Omdrejninger = 2 * 2 * 2*acos(0.0) * c / cpr = 8 * acos(0.0) * c / cpr
  Kilde på CPR: https://www.pololu.com/docs/0J63/3.4
  */
  int count;
  if (index == true) { count = encoders.getCountsLeft(); }
  if (index == false) { count = encoders.getCountsRight(); }
  return  8 * acos(0.0) * count / 900;
}

float zumoModules::motorOmdrejningerReset(bool index) {
  /*
  Basically {motorOmdrejninger(bool index)} 
  bare encoders aka. omdrejninger resettes
  */
  int count;
  if (index == true) { count = encoders.getCountsAndResetLeft(); }
  if (index == false) { count = encoders.getCountsAndResetRight(); }
  return  8 * acos(0.0) * count / 900;
}