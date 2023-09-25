#ifndef Zumo32U4Modules_h   /* Denne linje kommer altid først i en header fil */

#include <Arduino.h>  // En header fil, oprindeligt fra C, påkræver librariet som gør .ino hvad det er.
#include <Wire.h>     // Zumo32U4.h er afhængig af følgende library
#include <Zumo32U4.h> // Tilgå Zumo32U4.h library her: https://pololu.github.io/zumo-32u4-arduino-library/

#define Zumo32U4ModulesButtonBuzzer_h
/*! \file Zumo32U4ModulesButtonBuzzer.h */

/*! \brief Denne klasse repræsenterer knapperne A, B & C samt Buzzeren
 *
 * Gør det muligt at anvende knapperne A, B & C og spille frekvenser af lyd med Buzzeren.
 * Ud fra disse komponenter der der blevet lavet denne funktion:
 * buttonBootup(attention, windup): Vent på enten A, B eller C trykkes. 
   Efterfølgende går der {windup} ms inden der gås videre til næste sekvens i programmet.
   Undervejs lyses der med LEDs og spilles med Buzzer for at indikere programmets start.
   Og nå jo: Det er muligt at indstille hvad programmet skal gøre afhængigt af hvilken knap trykkes!
   A returnerer 1; B returnerer 2; C returnerer 3;
 */
class Zumo32U4ModulesButtonBuzzer : protected Zumo32U4ButtonA, 
                                    protected Zumo32U4ButtonB,
                                    protected Zumo32U4ButtonC, 
                                    protected Zumo32U4Buzzer { 
public: 
/*! Vent på enten A, B eller C trykkes. 
   Efterfølgende går der {windup} ms inden der gås videre til næste sekvens i programmet.
   Undervejs lyses der med LEDs og spilles med Buzzer for at indikere programmets start.
   Og nå jo: Det er muligt at indstille hvad programmet skal gøre afhængigt af hvilken knap trykkes!
   A returnerer 1; B returnerer 2; C returnerer 3; */
  int buttonBootup(int attention=10, int windup=800); };

#define Zumo32U4ModulesMotors_h
/*! \file Zumo32U4ModulesMotors.h */

/*! \brief Denne klasse repræsenterer de 2 motorer.
 *
 * Kørsel med Zumo32U4 sker med disse 2 funktioner:
 * * motorDrive(left=0, right=0): Kør mellem -400 og 400 med {left} og {right}
 * * motorReverse(bakgear): Skal Zumo32U4 gå i bakgear - bytte rundt på værdierne på begge motorer?
 */
class Zumo32U4ModulesMotors : protected Zumo32U4Motors { 
public: 
  /*! Kør mellem -400 og 400 med {left} og {right} */
  void motorDrive(int left=0, int right=0); 

  /*! Skal Zumo32U4 gå i bakgear - bytte rundt på værdierne på begge motorer? */
  void motorReverse(bool bakgear); 
};

#define Zumo32U4ModulesEncoders_h
/*! \file Zumo32U4ModulesEncoders.h */

/*! \brief Denne klasse repræsenterer encoders, aka. motorernes omdrejningsmåller
 *
 * Aflæs omdrejninger på Zumo32U4's hjul. Kan ske på 2 måder:
 * * motorOmdrejninger(index): Udvind Omdrejninger på false=venstre & true=højre motor
 * * motorOmdrejningerReset(index): Ligesom motorOmdrejninger(index), bare denne encoder også nulstilles
 */
class Zumo32U4ModulesEncoders : protected Zumo32U4Encoders { 
public: 
  /*! \brief Udvind Omdrejninger på false=venstre & true=højre motor */
  float motorOmdrejninger(bool index);

  /*! \brief Ligesom motorOmdrejninger(index), bare denne encoder også nulstilles */
  float motorOmdrejningerReset(bool index);
};

#define Zumo32U4ModulesSensors_h
/*! \file Zumo32U4ModulesSensors.h */

/*! \brief Denne klasse repræsenterer sensorerne LineSensors, IMU & ProximitySensors
 *
 * Gør det muligt at anvende LineSensors, IMU & ProximitySensors ved blot at konstruere et objekt:
 * * Lave et objekt ud fra denne klasse: Zumo32U4ModulesSensors {OBJEKTNAVN};
 * * Lave et objekt ud fra en klasse hvor nedarvning af denne funktion foretages.
 * Efter objektet er konstrueret vha. constructoren er det blot et spørgsmål om at udvinde værdierne.
 * * getProximitySensorValue(index): Udvind integer fra 0-6 af "proximitet" ift Zumo32U4´s front.
 * * getLineSensorValue(index): Udvind lineSensorValues[{index}], hvor {index} ligger mellem 0 og 2. Ellers: Aflæs LineSensor
 * * getIMUValue(mag): Opdatér alle, hvis ingen parametre indtastes, eller kun én af følgende aflæste sensorer:
 *   - Accelerometer {acc[3]}
 *   - Gyrometer {gyro[3]}
 *   - Magnetometer {mag[3]}
 */
class Zumo32U4ModulesSensors :  protected Zumo32U4ProximitySensors,
                                protected Zumo32U4LineSensors, 
                                protected Zumo32U4IMU {
public:
  uint16_t lineSensorValues[3];
  int16_t* acc[3] = {&a.x, &a.y, &a.z}; 
  int16_t* gyro[3] = {&g.x, &g.y, &g.z};
  int16_t* mag[3] = {&m.x, &m.y, &m.z};
  /*! Eksekveres funktionen {imu.init();} forårsages USB enumeration failure (en dårlig ting).
   *  Zumo32U4ModulesSensors::type = Zumo32U4IMUType::LSM303D_L3GD20H; // WHY imu.type PRIVATE???*/
  Zumo32U4ModulesSensors();
  
  /*! Udvind lineSensorValues[{index}], hvor {index} ligger mellem 0 og 2. Ellers: Aflæs LineSensor */
  int getProximitySensorValue(bool index);
  
  /*! Udvind lineSensorValues[{index}], hvor {index} ligger mellem 0 og 2. Ellers: Aflæs LineSensor */
  int getLineSensorValue(int index=-1);

  /*! Opdatér alle, hvis ingen parametre indtastes, eller kun én af følgende aflæste sensorer:
 *   - Accelerometer {acc[3]}
 *   - Gyrometer {gyro[3]}
 *   - Magnetometer {mag[3]}
 */
  int16_t getIMUvalue(char mag='_');
};

#define Zumo32U4ModulesSensors_h
/*! \file Zumo32U4Modules.h */

/*! \brief En samlet nedarvning - samling - af klasserne for de forskellige Zumo32U4-komponenter.
 *
 * Benytter de 3 knapper, buzzer, motors, encoders, lineSensor, IMU & proximitySensor.
 * Derudover er der følgende funktioner tilkoblet:
 * * LEDBlink(interval): Blink med de 3 LEDs i intervaller af {interval}
 * * IMUEndCondition(): Afslut programmet hvis enten Zumo løftes hurtigt op eller vendes på hovedet
 */
class Zumo32U4Modules : public Zumo32U4ModulesButtonBuzzer, 
                        public Zumo32U4ModulesMotors, 
                        public Zumo32U4ModulesEncoders, 
                        public Zumo32U4ModulesSensors
{
protected:
  int displayLine = 0; // Bruges eksklusivt til class Zumo32U4ModulesLCD & class Zumo32U4ModulesOLED
public:
  /*! Blink med de 3 LEDs i intervaller af {interval}.
   *  Se funktionen som en mere fancy delay() funktion */
  void LEDblink(int interval); 

  /*! Afslut programmet hvis enten Zumo løftes hurtigt op eller vendes på hovedet */
  void IMUEndCondition();
};

#define Zumo32U4ModulesLCD_h
/*! \file Zumo32U4ModulesLCD.h */

/*! \brief Benytter komponenterne fra {Zumo32U4Modules} samt LCD-displayet
 *
 * Til at interagere med OLED-displayet er der 2 funktioner klar til brug:
 * * LCDDisplayMenu(): Viser batteriets spænding samt hvorvidt Zumo32U4 er forbundet til en seperat enhed
 * * LCDprint(input, newLine, clear): Print et {input}. En ny linje kan laves inden da med {NewLine}, 
 *   og efterfølgende kan displayet ryddes med {clear}.
 * Eftersom LCD-display er begrænset til 8x2 tegn, kan der vitterligt kun stå 16 tegn på displayet ad gangen.
 */
class Zumo32U4ModulesLCD : public Zumo32U4Modules, protected Zumo32U4LCD {
public:
  /*! Eftersom LCD-display er begrænset til 8x2 tegn, kan der vitterligt kun stå 16 tegn på displayet ad gangen. */
  Zumo32U4ModulesLCD();

  /*! Viser batteriets spænding samt hvorvidt Zumo32U4 er forbundet til en seperat enhed. */
  void LCDDisplayMenu();

  /*! Print et {input}. En ny linje kan laves inden da med {NewLine}, og efterfølgende kan displayet ryddes med {clear}. */
  void LCDprint(String input, bool newLine=false, bool clear=false);
};

#define Zumo32U4ModulesOLED_h
/*! \file Zumo32U4ModulesOLED.h */

/*! \brief Benytter komponenterne fra {Zumo32U4Modules} samt OLED-displayet
 *
 * Til at interagere med OLED-displayet er der 2 funktioner klar til brug:
 * * OLEDDisplayMenu(): Viser batteriets spænding samt hvorvidt Zumo32U4 er forbundet til en seperat enhed
 * * OLEDprint(input, newLine, clear): Print et {input}. En ny linje kan laves inden da med {NewLine}, 
 *   og efterfølgende kan displayet ryddes med {clear}.
 * Desuden er der muligheden for at lave vfx (graphics) [WORK IN PROGRESS]
 */


class Zumo32U4ModulesOLED : public Zumo32U4Modules, protected Zumo32U4OLED {
public:
  /*! OLED-displayet kan have op til 21x8 (ellers 11x4 & 8x2) tegn på displayet på samme tid! 
   *  Desuden er der muligheden for at lave vfx (graphics) [WORK IN PROGRESS] */
  Zumo32U4ModulesOLED();

  /*! Viser batteriets spænding samt hvorvidt Zumo32U4 er forbundet til en seperat enhed. */
  void OLEDDisplayMenu();

  /*! Print et {input}. En ny linje kan laves inden da med {NewLine}, og efterfølgende kan displayet ryddes med {clear}. */
  void OLEDprint(String input, bool newLine=false, bool clear=false);
};

#endif // Denne linje kommer altid sidst i en header fil