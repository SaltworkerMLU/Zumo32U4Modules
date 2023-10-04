#ifndef Zumo32U4Modules_h   /* This line always comes first in a header file */

#include <Arduino.h>  // A header file, originally from C, required the library that makes .ino what it is.
#include <Wire.h>     // Zumo32U4.h depends on this library to function properly
#include <Zumo32U4.h> // Access Zumo32U4.h library here: https://pololu.github.io/zumo-32u4-arduino-library/

#define Zumo32U4ModulesButtonBuzzer_h
/*! \file Zumo32U4ModulesButtonBuzzer.h */

/*! \brief This class represents the Zumo32U4 buttons A, B & C aswell as the buzzer */
class Zumo32U4ModulesButtonBuzzer : protected Zumo32U4ButtonA, 
                                    protected Zumo32U4ButtonB,
                                    protected Zumo32U4ButtonC, 
                                    protected Zumo32U4Buzzer { 
public: 
/*! Check button press state.
 *  NONE=0; A=1; B=2; A+B=3; C=4; A+C=5; B+C=6; A+B+C=7; */
  int checkButtonPress();

/*! Wait until either A, B or C has been pressed and released.
    A returns 1; B returns 2; C returns 3 */
  int getButtonRelease();

/*! Play {frequency} with the buzzer. You can set {duration} and {volume} of the frequency */
  void buzzer(int frequency=400, int duration=50, int volume=10);

/*! Wait until either A, B or C has been pressed and released.
   Afterwards, {windup} ms of a "fancy delay" follows.
   This "fancy delay" consists of the LEDs lighting like a traffic light and the buzzer speaking accordingly.
   Furthermore: You can set a program protocol depending on button press&release! 
   Again, A returns 1; B returns 2; C returns 3*/
  int buttonBootup(int attention=10, int windup=800); 
};

#define Zumo32U4ModulesMotors_h
/*! \file Zumo32U4ModulesMotors.h */

/*! \brief This class represents the 2 Zumo32U4 motors. */
class Zumo32U4ModulesMotors : protected Zumo32U4Motors { 
public: 
  /*! Drive between -400 and 400 with {left} and {right}. If {reverse} is true, motor speeds are reversed */
  void motorDrive(int left=0, int right=0, bool reverse=false); 
};

#define Zumo32U4ModulesEncoders_h
/*! \file Zumo32U4ModulesEncoders.h */

/*! \brief This class represents the encoders, aka. the motor turns, based on wheel circumference, measurer */
class Zumo32U4ModulesEncoders : protected Zumo32U4Encoders { 
public: 
  /*! \brief Get distance in cm of {index}==false: left motor; index==true: right motor */
  float motorDistance(bool index);

  /*! \brief Like motorOmdrejninger(index), except both encoders are reset */
  float motorDistanceReset(bool index);
};

#define Zumo32U4ModulesSensors_h
/*! \file Zumo32U4ModulesSensors.h */

/*! \brief This class represents the LineSensors, IMU & ProximitySensors
 * Class constructor enables lineSensors, IMU & ProximitySensors. */
class Zumo32U4ModulesSensors :  protected Zumo32U4ProximitySensors,
                                protected Zumo32U4LineSensors, 
                                protected Zumo32U4IMU {
public:
  uint16_t lineSensorValues[3];
  int16_t* mag[3] = {&m.x, &m.y, &m.z};
  int16_t* acc[3] = {&a.x, &a.y, &a.z}; 
  int16_t* gyro[3] = {&g.x, &g.y, &g.z};
  int16_t gyroOffset;  // When gyro[index] = gyroOffset: No change in angle
  uint16_t LastUpdate; // Earlier measurement of time with micros(). "Old time"
  uint32_t turnAngle = 0; // Current calibrated angle of Zumo32U4

  /*! {imu.init();}, a command needed to actually initialize the IMU, causes USB enumeration failure when executed in constructor (Don't try it).
   *  Zumo32U4ModulesSensors::type = Zumo32U4IMUType::LSM303D_L3GD20H; // Zumo32U4IMUType is private, thus cannot be changed outside the class */
  Zumo32U4ModulesSensors();
  
  /*! Read proximitySensor. Get right proximitySensor if {index} == false and left if {index} == true */
  int getProximitySensorValue(bool index);
  
  /*! Read lineSensor. Get lineSensorValues[{index}] provided {index} lies between 0 and 2 */
  int getLineSensorValue(int index=-1);

  /*! Update all, if no parameter is given, otherwise update one of the following:
 *   - Accelerometer {acc[3]}
 *   - Gyrometer {gyro[3]}
 *   - Magnetometer {mag[3]}
 */
  int16_t getIMUvalue(char m_a_g='_');

  /*! Needed to get actual angle of Zumo32U4 in setup(). */
  int16_t calibrate(char m_a_g, int index, int iterations=1000); // EXPERIMENTAL

  /*! After calibrate(char m_a_g, int index, int iterations) has been executed, get actual angle using gyrometer */
  uint32_t dAngle(int index); // EXPERIMENTAL
};

#define Zumo32U4ModulesSensors_h
/*! \file Zumo32U4Modules.h */

/*! \brief A class collectively inheriting from the prior classes, essentially uniting all classes into one.
 * Uses the 3 buttons, buzzer, motors, encoders, lineSensor, IMU & proximitySensor. */
class Zumo32U4Modules : public Zumo32U4ModulesButtonBuzzer, 
                        public Zumo32U4ModulesMotors, 
                        public Zumo32U4ModulesEncoders, 
                        public Zumo32U4ModulesSensors {
public:
  int displayLine = 0; // Exclusively used for class Zumo32U4ModulesLCD & class Zumo32U4ModulesOLED

  /*! Blink with the 3 LEDs in intervals of {interval}.
   *  Consider this function a fancy equivalent to delay() */
  void LEDblink(int interval); 

  /*! End the program if either Zumo is lifted up or turned upside down */
  void IMUEndCondition();
};

#define Zumo32U4ModulesLCD_h
/*! \file Zumo32U4ModulesLCD.h */

/*! \brief Use components from {Zumo32U4Modules} aswell as the LCD-display.
 * The LCD-display is limited to 8x2 dimensions, meaning only 16 signs can be displayed once at a time. */
class Zumo32U4ModulesLCD : public Zumo32U4Modules, protected Zumo32U4LCD {
public:
  /*! Equivalent to a "Hello World!" displayed on the LCD. */
  Zumo32U4ModulesLCD();

  /*! Show battery voltage aswell as whether Zumo32U4 is connected to another unit. */
  void displayMenu();

  /*! Print {input}. {clear} determines whether the display should be cleared prior. {NewLine} makes the next displayPrint(char input[]) be printed on the next line */
  void displayPrint(char input[], bool clear=false, bool newLine=true); // Example: LCDprint("Hello World!");

  /*! Set up to 8 custom characters on the dispaly. By default, 8 custom characters are defined in class constructor.
   *  Try defining your own custom character by inserting the following into your .ino file: 
   *  const char FULLBAR[] PROGMEM = {63, 63, 63, 63, 63, 63, 63, 63}; */
  void displayCustomCharacters(char custom1[]={}, char custom2[]={}, char custom3[]={}, char custom4[]={}, 
                               char custom5[]={}, char custom6[]={}, char custom7[]={}, char custom8[]={});
};

#define Zumo32U4ModulesOLED_h
/*! \file Zumo32U4ModulesOLED.h */

/*! \brief Use components from {Zumo32U4Modules} aswell as the OLED-display.
 * The OLED-display can change the display dimensions to 8x2, 11x4 & 21x8.
 * Furthermore, the OLED-display is capable of making vfx graphics [WORK IN PROGRESS] */
class Zumo32U4ModulesOLED : public Zumo32U4Modules, protected Zumo32U4OLED {
public:
  /*! Equivalent to a "Hello World!" displayed on the LCD. */
  Zumo32U4ModulesOLED();

  /*! Show battery voltage aswell as whether Zumo32U4 is connected to another unit. */
  void displayMenu();

  /*! Print {input}. {clear} determines whether the display should be cleared prior. {NewLine} makes the next displayPrint(char input[]) be printed on the next line */
  void displayPrint(char input[], bool clear=false, bool newLine=true); // Eksempel: OLEDprint("Hello World!");

  /*! Set up to 8 custom characters on the dispaly. By default, 8 custom characters are defined in class constructor.
   *  Try defining your own custom character by inserting the following into your .ino file: 
   *  const char FULLBAR[] PROGMEM = {63, 63, 63, 63, 63, 63, 63, 63}; */
  void displayCustomCharacters(char custom1[]={}, char custom2[]={}, char custom3[]={}, char custom4[]={}, 
                               char custom5[]={}, char custom6[]={}, char custom7[]={}, char custom8[]={});
};

// This character is a back arrow ultimately pointing to the left.
const char backArrow[] PROGMEM = {
  0b00000,
  0b00010,
  0b00001,
  0b00101,
  0b01001,
  0b11110,
  0b01000,
  0b00100,
};

// This character is a back arrow ultimately pointing to the right.
const char backArrowReverse[] PROGMEM = {
  0b00000,
  0b01000,
  0b10000,
  0b10100,
  0b10010,
  0b01111,
  0b00010,
  0b00100,
};

// This character is two chevrons pointing up.
const char forwardArrows[] PROGMEM = {
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
};

// This character is two chevrons pointing down.
const char reverseArrows[] PROGMEM = {
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b10001,
  0b01010,
  0b00100,
  0b00000,
};

// This character is two solid arrows pointing up.
const char forwardArrowsSolid[] PROGMEM = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b01110,
  0b11111,
  0b00000,
};

// This character is two solid arrows pointing down.
const char reverseArrowsSolid[] PROGMEM = {
  0b00000,
  0b11111,
  0b01110,
  0b00100,
  0b11111,
  0b01110,
  0b00100,
  0b00000,
};

// This character is a solid arrow pointing to the right.
const char rightArrow[] PROGMEM = {
  0b01000,
  0b01100,
  0b01110,
  0b01111,
  0b01110,
  0b01100,
  0b01000,
  0b00000,
};

// This character is a solid arrow pointing to the left.
const char leftArrow[] PROGMEM = {
  0b00010,
  0b00110,
  0b01110,
  0b11110,
  0b01110,
  0b00110,
  0b00010,
  0b00000,
};

#endif // This line always comes last in a header file
