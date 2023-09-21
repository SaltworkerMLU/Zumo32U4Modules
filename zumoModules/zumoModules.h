/********************************************************************************************
* zumoModules.h                                                                             *
* Et library beregnet til at gøre programmeringen af en Zumo32U4 hurtigere og nemmere.      *
* Dvs. at man med librariet ikke behøver at inkludere <Zumo32U.h> & <Wire.h>,               *
* konstruere objekter beregnet til de respektive komponenter,                               *
* setup sensorer som kræver yderligere mén & foretage handlinger med sagte komponenter      *
* zumoModules.h lagrer følgende data:                                                       *
* - Importerede library <Zumo32U4.h>, herunder libraries som <Zumo32U4.h> er afhængig af    *
* - Klasse objekter fra <Zumo32U4.h>                                                        *
* - Klassen "zumoModules", herunder                                                         *
*   (I)   Klasse variabler                                                                  *
*   (II)  Klasse funktioner (kun funktionsnavn & parametre)                                 *
*********************************************************************************************
* Opdateret: 21. september 2023                                                             *
********************************************************************************************/

#ifndef zumoModules_h   /* Denne linje kommer altid først i en header fil */
#define zumoModules_h

#include <Arduino.h>  // En header fil, oprindeligt fra programmeringssproget C, påkræver librariet som gør .ino hvad det er.
#include <Wire.h>     // Zumo32U4.h er afhængig af følgende library
#include <Zumo32U4.h> // Tilgå Zumo32U4.h library her: https://pololu.github.io/zumo-32u4-arduino-library/

class zumoModules
{
public: // Alt nedenfor kan tilgås i .ino fil
  void LEDblink(int interval);
  
  Zumo32U4LCD LCD;                      // DISPLAY 1: Det grønne display
  Zumo32U4OLED OLED;                    // DISPLAY 2: Det blå display

  void OLEDdisplayMenu(String A, String B, String C);
  
  Zumo32U4ButtonA buttonA;
  Zumo32U4ButtonB buttonB;
  Zumo32U4ButtonC buttonC;
  Zumo32U4Buzzer buzzer;                // Lav støj med zumo
  int buttonBootup(int attention=10, int windup=800);
  
  Zumo32U4LineSensors lineSensors;      // Mål reflektion af pladen Zumo kører på med forskellige sensorer.
  Zumo32U4IMU imu;                      // Måler Acceleration, vinkelacceleration, magnetfelt
  Zumo32U4ProximitySensors proxSensors; // Mål proximitet. IKKE afstand. Værdier går fra 0-6
  void sensorBootup(bool line_sensors=true, bool IMU=true, bool prox_sensors=true);
  uint16_t lineSensorValues[3]; // array af lineSensorValues
  int getLineSensorValue(int index=-1);
  int getProximitySensorValue(bool index);
  void gyroEndCondition();

  Zumo32U4Motors motors;                // Motorer. Kør med zumo
  void motorDrive(int left=0, int right=0);
  void motorReverse(bool bakgear);

  Zumo32U4Encoders encoders;            // Omdrejningsmåler på motorer
  float motorOmdrejninger(bool index);
  float motorOmdrejningerReset(bool index);

  Zumo32U4IRPulses IRPulses;            // Send IR-pulses. Objektet gør ikke mere end det.
};

#endif // Denne linje kommer altid sidst i en header fil