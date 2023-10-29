#ifndef Zumo32U4Modules_h   // This line always comes first in a header file

#include <Arduino.h>  // A header file, originally from C, required the library that makes .ino what it is.
#include <Wire.h>     // Zumo32U4.h depends on this library to function properly
#include <Zumo32U4.h> // Access Zumo32U4.h library here: https://pololu.github.io/zumo-32u4-arduino-library/

#define Zumo32U4ModulesButtons_h
/*! \file Zumo32U4ModulesButtonBuzzer.h */

/*! \brief This class represents the Zumo32U4 buttons A, B & C */
class Zumo32U4ModulesButtons : protected Zumo32U4ButtonA, 
                               protected Zumo32U4ButtonB,
                               protected Zumo32U4ButtonC { 
public: 
  /*! Set a program protocol depending on button release (or button press if we're feeling fancy).
   *  This variable is automatically updated when buttonBootup() or buttonBootupSound() is executed.
   *  One use case is to seperate 1 program into 3 protocols using switch-cases. Use your creativity. */
  char buttonRelease;

/*! Check button press state.
 *  NONE=0; A=1; B=2; A+B=3; C=4; A+C=5; B+C=6; A+B+C=7; */
  int checkButtonPress();

/*! Wait until either A, B or C has been pressed and released.
    Return the specific character of the button pressed and released. */
  char getButtonRelease();

/*! Wait until either A, B or C has been pressed and released.
   Afterwards, {windup} ms of a "fancy delay" follows.
   This "fancy delay" consists of the LEDs lighting like a traffic light.
   Furthermore: Class variable buttonRelease is updated since getButtonRelease() is used. */
  void buttonBootup(int windup=800); 
};

#define Zumo32U4ModulesBuzzer_h
/*! \file Zumo32U4ModulesMotors.h */

/*! \brief This class represents the Zumo32U4 buzzer */
class Zumo32U4ModulesBuzzer : public Zumo32U4Buzzer { // Imports Zumo32U4Buzzer for direct use in .ino file
public:
  /*! Play {frequency} with the buzzer. You can set {duration} and {volume} of the frequency */
  void buzzer(int frequency=400, int duration=50, int volume=10);
};

#define Zumo32U4ModulesMotors_h
/*! \file Zumo32U4ModulesMotors.h */

/*! \brief This class represents the 2 Zumo32U4 motors. */
class Zumo32U4ModulesMotors : protected Zumo32U4Motors { 
private:
  bool reverse=false; // Are both motor values reversed?
public: 
  /*! Simply stops both Zumo32U4 motors more byte-efficiently than other functions. */
  void motorDrive();

  /*! Drive between -400 and 400 with {left} and {right}. If {reverse} is true, motor speeds are reversed */
  void motorDrive(int left, int right); 

  /*! Simply reverses motor speed values. */
  void motorFlip();

};

#define Zumo32U4ModulesEncoders_h
/*! \file Zumo32U4ModulesEncoders.h */

/*! \brief This class represents the encoders, aka. the motor turns, based on wheel circumference, measurer */
class Zumo32U4ModulesEncoders : protected Zumo32U4Encoders { 
private:
  long oldTime[2]; // Old time for both encoders
  float oldDistance[2]; // Old distance for both encoders
public: 
  /*! \brief Motor distance in unit cm.
   *   Call function motorDistance() [alternatively either getMotorVelocity() or getMotorAcceleration]*/
  float motorDistance[2];

  /*! \brief Motor velocity in unit cm/s.
   *   Call function getMotorVelocity() [alternatively getMotorAcceleration()] before using this array */
  float motorVelocity[2];

  /*! \brief Motor acceleration in unit cm/s^2.
   *   Call function getMotorAcceleration() before using this array */
  float motorAcceleration[2];

  /*! \brief Clicks per rotation. 
   *   Adjust this value if encoder values give inprecise distance in cm */
  int CPR=900;

  /*! \brief Get distance in cm of both motors using encoders */
  void getMotorDistance();

  /*! \brief Like getMotorDistance(), except both encoders are reset */
  void getMotorDistanceReset();

  /*! \brief Use both encoders to get Zumo32U4 velocity as array motorVelocity[2] */
  void getMotorVelocity();

  /*! \brief Use both encoders to get Zumo32U4 acceleration as array motorAcceleration[2].
   *   Aswell as velocity as array motorVelocity[2] */
  void getMotorAcceleration();
};

#define Zumo32U4ModulesLineSensors_h
/*! \file Zumo32U4ModulesLineSensors.h */

/*! \brief This class represents the Zumo32U4 Line Follower Sensors.
     Class constructor initializes LineSensors from the get go */
class Zumo32U4ModulesLineSensors : protected Zumo32U4LineSensors {
public:
  uint16_t lineSensorValue[5];

  /*! Assume Zumo32U4 has 5 lineSensors */
  Zumo32U4ModulesLineSensors();

  /*! Read lineSensor. Afterwards, access these values using lineSensorValues[]. */
  void getLineSensorValue();

  /*! Calibrate lineSensor. Afterwards, the function getLineSensorValueCalibrated() can be executed. */
  void calibrateLineSensor();

  /*! Calibrate lineSensor. Afterwards, the function getLineSensorValueCalibrated() can be executed. */
  void getLineSensorValueCalibrated();
};

#define Zumo32U4ModulesProximitySensors_h
/*! \file Zumo32U4ModulesProximitySensors.h */

/*! \brief This class represents the Zumo32U4 Proximity Sensors.
     Class constructor initializes Protimity Sensors from the get to*/
class Zumo32U4ModulesProximitySensors : protected Zumo32U4ProximitySensors {
public:
  uint8_t proximitySensorValue[2];

  /*! Assume Zumo32U4 has 2 Proximity Sensors */
  Zumo32U4ModulesProximitySensors();

  /*! Read proximitySensor. Proximity Sensor values are stored in uint8_t proximitySensorValues[2] */
  void getProximitySensorValue();
};

#define Zumo32U4ModulesIMU_h
/*! \file Zumo32U4ModulesIMU.h */

/*! \brief This class represents the LineSensors, IMU & ProximitySensors
 * Class constructor enables lineSensors, IMU & ProximitySensors. */
class Zumo32U4ModulesIMU : protected Zumo32U4IMU {
public:
  int16_t* mag[3] = {&m.x, &m.y, &m.z};
  int16_t* acc[3] = {&a.x, &a.y, &a.z}; 
  int16_t* gyro[3] = {&g.x, &g.y, &g.z};
  int16_t gyroOffset;  // When gyro[index] = gyroOffset: No change in angle
  uint16_t LastUpdate; // Earlier measurement of time with micros(). "Old time"
  uint32_t turnAngle = 0; // Current calibrated angle of Zumo32U4

  /*! \brief Call this function before using IMU-values. 
        Cannot be replaced by constructor.
        Attempting so will cause a USB enumeration failure (a bad thing) */
  void initIMU();

  /*! Update all, if no parameter is given, otherwise update one of the following:
 *   - Accelerometer {acc[3]}
 *   - Gyrometer {gyro[3]}
 *   - Magnetometer {mag[3]}
 */
  int16_t getIMUvalue(char m_a_g='_');

  /*! Needed to get actual angle of Zumo32U4 in setup(). */
  int16_t calibrateIMU(char m_a_g, int index, int iterations=1000);

  /*! After calibrate(char m_a_g, int index, int iterations) has been executed, get actual angle using gyrometer. 
   *  Make sure to run this function in a loop without any forced delays like delay() as this will cause errors in angle calculations */
  int32_t gyroAngle(int index);
};

#define Zumo32U4Modules_h
/*! \file Zumo32U4Modules.h */

/*! \brief A class collectively inheriting from the prior classes, essentially uniting all classes into one.
 * Uses the 3 buttons, buzzer, motors, encoders, lineSensor, IMU & proximitySensor. */
class Zumo32U4Modules : public Zumo32U4ModulesButtons, 
                        public Zumo32U4ModulesBuzzer,
                        public Zumo32U4ModulesMotors, 
                        public Zumo32U4ModulesEncoders, 
                        public Zumo32U4ModulesLineSensors,
                        public Zumo32U4ModulesProximitySensors,
                        public Zumo32U4ModulesIMU {
public:
  uint8_t leftSpeed=0, rightSpeed=0; // Used for the function setMotorvelocity(float velocityLeft=0, float velocityRight=0)

  /*! Wait until either A, B or C has been pressed and released.
   Afterwards, {windup} ms of a "fancy delay" follows.
   This "fancy delay" consists of the LEDs lighting like a traffic light and the buzzer speaking accordingly.
   Furthermore: You can set a program protocol depending on button press&release! 
   Again, A returns 1; B returns 2; C returns 3*/
  void buttonBootupSound(int windup=800, int attention=10); 

  /*! End the program if either Zumo is lifted up or turned upside down */
  void IMUEndCondition();

  /*! Make the motors drive at a specific velocity. Makes for a smooth acceleration.
   * Velocity should be set between 0cm/s (motor stop) and 65cm/s (motor full speed) */
  void setMotorVelocity(float velocityLeft=0, float velocityRight=0);

  /*! Use a PID-controller, using line follower sensors as input and motors as output, to control Zumo32U4 movement.
   *  The PID-controller runs on a loop. To exit this loop, make both proximity sensors get max value. Sticking your foot in front of it should to the trick.
   *  Furthermore, a failsafe option is provived: IMUEndCondition(). */
  void PIDLineFollower(float P, float I, float D, int speed, bool fiveLineSensors, bool dir, bool blackLine=false);
};

#define Zumo32U4ModulesLCD_h
/*! \file Zumo32U4ModulesLCD.h */

/*! \brief Use components from {Zumo32U4Modules} aswell as the LCD-display.
 * The LCD-display is limited to 8x2 dimensions, meaning only 16 signs can be displayed once at a time. */
class Zumo32U4ModulesLCD : public Zumo32U4Modules, protected Zumo32U4LCD {
public:
  int displayLine = 0; // Set specific display line if we're feeling fancy

  /*! Equivalent to a "Hello World!" displayed on the LCD. */
  Zumo32U4ModulesLCD();

  /*! Show battery voltage aswell as whether Zumo32U4 is connected to another unit. */
  void displayMenu();

  /*! Print {input}. {clear} determines whether the display should be cleared prior. {NewLine} makes the next displayPrint(char input[]) be printed on the next line */
  void displayPrint(String input, bool clear=false, bool newLine=true); // Example: LCDprint("Hello World!");

  /*! Set up to 8 custom characters on the dispaly. By default, 8 custom characters are defined in class constructor.
   *  Try defining your own custom character by inserting the following into your .ino file: 
   *  const char FULLBAR[] PROGMEM = {31, 31, 31, 31, 31, 31, 31, 31}; */
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
  int displayLine = 0; // Set specific display line if we're feeling fancy

  /*! Equivalent to a "Hello World!" displayed on the LCD. */
  Zumo32U4ModulesOLED();

  /*! Show battery voltage aswell as whether Zumo32U4 is connected to another unit. */
  void displayMenu();

  /*! Print {input}. {clear} determines whether the display should be cleared prior. {NewLine} makes the next displayPrint(char input[]) be printed on the next line */
  void displayPrint(String input, bool clear=false, bool newLine=true); // Eksempel: OLEDprint("Hello World!");

  /*! Set up to 8 custom characters on the dispaly. By default, 8 custom characters are defined in class constructor.
   *  Try defining your own custom character by inserting the following into your .ino file: 
   *  const char FULLBAR[] PROGMEM = {31, 31, 31, 31, 31, 31, 31, 31}; */
  void displayCustomCharacters(char custom1[]={}, char custom2[]={}, char custom3[]={}, char custom4[]={}, 
                               char custom5[]={}, char custom6[]={}, char custom7[]={}, char custom8[]={});
};

const char backArrow[] PROGMEM = {0, 2, 1, 5, 9, 30, 8, 4}; // This character is a back arrow pointing to the left.
const char backArrowReverse[] PROGMEM = {0, 8, 16, 20, 18, 15, 2, 4}; // This character is a back arrow pointing to the right.
const char forwardArrows[] PROGMEM = {0, 4, 10, 17, 4, 10, 17, 0}; // This character is two chevrons pointing up.
const char reverseArrows[] PROGMEM = {0, 17, 10, 4, 17, 10, 4, 0}; // This character is two chevrons pointing down.
const char forwardArrowsSolid[] PROGMEM = {0, 4, 14, 31, 4, 14, 31, 0}; // This character is two solid arrows pointing up.
const char reverseArrowsSolid[] PROGMEM = {0, 31, 14, 4, 31, 14, 4, 0}; // This character is two solid arrows pointing down.
const char rightArrow[] PROGMEM = {8, 12, 14, 15, 14, 12, 8, 0}; // This character is a solid arrow pointing to the right.
const char leftArrow[] PROGMEM = {2, 6, 14, 30, 14, 6, 2, 0}; // This character is a solid arrow pointing to the left.

#endif // This line always comes last in a header file