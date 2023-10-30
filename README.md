# Table of contents
+ [WORK IN PROGRESS](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#WORK-IN-PROGRESS)
+ [Zumo32U4Modules](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#zumo32u4modules)
+ [Zumo32U4 documentation](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#Zumo32U4-documentation)
  + [Zumo32U4 pinout](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#zumo32u4-pinout)
  + [Installed jumper pins (proximity sensors vs. line sensors)](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#installed-jumper-pins-proximity-sensors-vs-line-sensors)
  + [Pinout alternatives](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#Pinout-alternatives)
+ [Zumo32U4Modules.h](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#zumo32u4modulesh)
+ [Get started](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#get-started)
  + [Method A: Download Zip with Arduino IDE](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#a-download-zip-with-arduino-ide)
  + [Method B: Manual insertion of extracted ZIP folder to Arduino sketch](https://github.com/SaltworkerMLU/Zumo32U4Modules/tree/main#b-manual-insertion-of-extracted-zip-folder-to-arduino-sketch)

# WORK IN PROGRESS
* Make use of OLED display graphics functionality (Currently limited to text)
* Add other sophisticated functions
   - IDEA: Add magnetometer-based compass function
   - IDEA: Add accelerometer-based movement
* Additional changes to library may occur
   - IDEA: Make initIMU() be executed as a class constructor (NOTE: More complicated than it seems. Zumo32U4.h source code manipulation [supposedly] needed to prevent USB enumeration failure [a bad thing])

# Zumo32U4Modules
Simplify the programming process of your comming Zumo32U4 project. Just import this library, create the nessecary object(s), and you're good to go to use the components in accordance to constructed object(s).

![image](Zumo32U4Modules_Media/Zumo32U4ModulesUML.jpg)

NOTE: These 8 custom characters come preloaded with Zumo32U4Modules.h
* forwardArrows
* forwardArrowsSolid
* reverseArrows
* reverseArrowsSolid
* leftArrow
* rightArrow
* backArrow
* backArrowReverse

They are used in the constructors Zumo32U4ModulesLCD & Zumo32U4ModulesOLED with the function:

```
  displayCustomCharacters(forwardArrows, forwardArrowsSolid, reverseArrows, reverseArrowsSolid,
                          leftArrow, rightArrow, backArrow, backArrowReverse); // Default configuration of custom characters
```

Furthermore, this library imports the library ```#include <Zumo32U4.h>``` which has the following functions outside of any classes therefore also accessible in this library attached:
* ledRed(bool)
* ledYellow(bool)
* ledGreen(bool)
* readBatteryMillivolts(): int
* isPowerPresent(): bool // is the Zumo32U4 connected to another device, e.g. computer, via. USB cable?

The fact that the library "Zumo32U4.h" is imported also means you still can create objects of the ordinary Zumo32U4 classes alongside Zumo32U4Modules classes.
# Zumo32U4 documentation
Zumo32U4 user's guide: https://www.pololu.com/docs/0J63

Zumo32U4.h source code (src): https://pololu.github.io/zumo-32u4-arduino-library/index.html
## Zumo32U4 Pinout
The Zumo32U4 user's guide provides the pinout of the Zumo32U4 (link: https://www.pololu.com/docs/0J63):

![image](https://a.pololu-files.com/picture/0J11462.1200.jpg?ead95d85e8f3f8b9f046e9c3e455067a)

![image](https://a.pololu-files.com/picture/0J11463.1200.jpg?d05e32ece41cc1e53560773cba275dea)

Note how pin 4 & 20 have conflicting assignments: 
* Pin 4 is assigned to both "Right prox sensor" & "Line sensor 4"
* Pin 20 is assigned to both "Left prox sensor" & "Line Sensor 2"
## Installed jumper pins (proximity sensors vs. line sensors)
To compensate for the lacking pins, 2 jumper pins (the blue ones) are provided to let the user switch between additional proximity and line sensors.

![image](Zumo32U4Modules_Media/Pololu0J8696irs08a_up&down.png)

Where:
* "Right prox sensor" = RGT
* "Left prox sensor" = LFT
* "Line sensor 2" = DN2
* "Line sensor 4" = DN4

This way, the Zumo32U4 configuration options using jumper pins are as follows:
* RGT -> 4 <- DN4
* LFT -> 20 <- DN2

## Pinout alternatives
In order to use all 3 proximity sensors & 5 line sensors simultaneously, sacrifices must be made.

2 pins must be made available for the remaining proximity sensor(s) and/or line sensor(s). One way to do this, as the documentation suggests, is to sacrifice the display to make use of pin 0 and 1. 

By soldering the remaining proximity sensor(s) and/or line sensor(s) to pin 0 and 1 and then assigning these pins to their sensors in the code, it will be possible to make use of all 3 proximity sensors & 5 line sensors simultaneously.

# Zumo32U4Modules.h
Here is "Zumo32U4Modules.h" with almost no comments to shorten it.
``` 
#ifndef Zumo32U4Modules_h   // This line always comes first in a header file

#include <Arduino.h>  // A header file, originally from C, required the library that makes .ino what it is.
#include <Wire.h>     // Zumo32U4.h depends on this library to function properly
#include <Zumo32U4.h> // Access Zumo32U4.h library here: https://pololu.github.io/zumo-32u4-arduino-library/

class Zumo32U4ModulesButtons : protected Zumo32U4ButtonA, 
                               protected Zumo32U4ButtonB,
                               protected Zumo32U4ButtonC { 
public: 
  char buttonRelease;

  int checkButtonPress();
  char getButtonRelease();
  void buttonBootup(int windup=800); 
};

class Zumo32U4ModulesBuzzer : public Zumo32U4Buzzer { // Imports Zumo32U4Buzzer for direct use in .ino file
public:
  void buzzer(int frequency=400, int duration=50, int volume=10);
};

class Zumo32U4ModulesMotors : protected Zumo32U4Motors { 
private:
  bool reverse=false; // Are both motor values reversed?
public: 
  void motorDrive();
  void motorDrive(int left, int right); 
  void motorFlip();

};

class Zumo32U4ModulesEncoders : protected Zumo32U4Encoders { 
private:
  long oldTime[2]; // Old time for both encoders
  float oldDistance[2]; // Old distance for both encoders
public: 
  float motorDistance[2];
  float motorVelocity[2];
  float motorAcceleration[2];
  int CPR=900;

  void getMotorDistance();
  void getMotorDistanceReset();
  void getMotorVelocity();
  void getMotorAcceleration();
};

class Zumo32U4ModulesLineSensors : protected Zumo32U4LineSensors {
public:
  uint16_t lineSensorValue[5];

  Zumo32U4ModulesLineSensors();
  void getLineSensorValue();
  void calibrateLineSensor();
  void getLineSensorValueCalibrated();
};

class Zumo32U4ModulesProximitySensors : protected Zumo32U4ProximitySensors {
public:
  uint8_t proximitySensorValue[6];

  Zumo32U4ModulesProximitySensors();
  void getProximitySensorValue();
};

class Zumo32U4ModulesIMU : protected Zumo32U4IMU {
public:
  int16_t* mag[3] = {&m.x, &m.y, &m.z};
  int16_t* acc[3] = {&a.x, &a.y, &a.z}; 
  int16_t* gyro[3] = {&g.x, &g.y, &g.z};
  int16_t gyroOffset;  // When gyro[index] = gyroOffset: No change in angle
  uint16_t LastUpdate; // Earlier measurement of time with micros(). "Old time"
  uint32_t turnAngle = 0; // Current calibrated angle of Zumo32U4

  void initIMU();
  int16_t getIMUvalue(char m_a_g='_');
  int16_t calibrateIMU(char m_a_g, int index, int iterations=1000);
  int32_t gyroAngle(int index);
};

class Zumo32U4Modules : public Zumo32U4ModulesButtons, 
                        public Zumo32U4ModulesBuzzer,
                        public Zumo32U4ModulesMotors, 
                        public Zumo32U4ModulesEncoders, 
                        public Zumo32U4ModulesLineSensors,
                        public Zumo32U4ModulesProximitySensors,
                        public Zumo32U4ModulesIMU {
public:
  uint8_t leftSpeed=0, rightSpeed=0; // Used for the function setMotorvelocity(float velocityLeft=0, float velocityRight=0)

  void buttonBootupSound(int windup=800, int attention=10); 
  void IMUEndCondition();
  void setMotorVelocity(float velocityLeft=0, float velocityRight=0);
  void PIDLineFollower(float P, float I, float D, int speed, bool fiveLineSensors, bool dir, bool blackLine=false);
};

class Zumo32U4ModulesLCD : public Zumo32U4Modules, protected Zumo32U4LCD {
public:
  int displayLine = 0; // Set specific display line if we're feeling fancy

  Zumo32U4ModulesLCD();
  void displayMenu();
  void displayPrint(String input, bool clear=false, bool newLine=true); // Example: LCDprint("Hello World!");
  void displayCustomCharacters(char custom1[]={}, char custom2[]={}, char custom3[]={}, char custom4[]={}, 
                               char custom5[]={}, char custom6[]={}, char custom7[]={}, char custom8[]={});
};

class Zumo32U4ModulesOLED : public Zumo32U4Modules, protected Zumo32U4OLED {
public:
  int displayLine = 0; // Set specific display line if we're feeling fancy

  Zumo32U4ModulesOLED();
  void displayMenu();
  void displayPrint(String input, bool clear=false, bool newLine=true); // Eksempel: OLEDprint("Hello World!");
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
```
("Zumo32U4Modules.h" last updated: 30. October 2023)

# Get Started
1.  Open Arduino IDE.
2.  Open library manager. Search for the library "Zumo32U4". Install it.
3.  Open File\Preferences and under "Additional board manager URLs" insert the following link: https://files.pololu.com/arduino/package_pololu_index.json
4.  Open boards manager. Search for the board "Pololu A-Star Boards". Install it.

You will now be able to use ```#include <Zumo32U4.h>```, which is required to use Zumo32U4Modules.

To install and use Zumo32U4Modules, here are 2 methods provided:
## A: Download Zip with Arduino IDE
1.  Download ZIP folder of Zumo32U4Modules
2.  Open Arduino IDE
3.  Add the downloaded ZIP to your libraries

![image](Zumo32U4Modules_Media/Zumo32U4Modules_Add.zip_Library.png)

Zumo32U4Modules can now be imported by using ```#include <Zumo32U4Modules.h>```
## B: Manual insertion of extracted ZIP folder to Arduino sketch
1.  Download ZIP folder of Zumo32U4Modules
2.  Like the title says, extract ZIP folder to your current Arduino sketch
   
Zumo32U4Modules can then be imported by using ```#include "Zumo32U4Modules.h"```
