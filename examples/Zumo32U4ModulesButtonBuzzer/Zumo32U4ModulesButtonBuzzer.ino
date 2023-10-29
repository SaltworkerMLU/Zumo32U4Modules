/*
 * This program shows how to use the class Zumo32U4ModulesButtonBuzzer
 * The program starts by waiting for the user to press and release either button.
 * One of these 3 sequences are played depending on which button is pressed and released:
 * 1. Print kombination of button(s) som trykkes ned.
 *    NONE = 0; A = 1; B = 2; A+B = 3; C = 4; A+C = 5; B+C = 6; A+B+C = 7
 * 2. Print released button to serial monitor. A=1; B=2; C=3
 * 3. Buzzer is used as an instrument. Depending on button press combination, a certain frequency is played.
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesButtons buttons;
Zumo32U4ModulesBuzzer buzzer;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  buttons.buttonBootup();
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(zumoBot.buttonRelease) {
    case 'A': // Print combination of buttonPresses
      Serial.println(buttons.checkButtonPress());
      delay(50);
      break;
    case 'B': // Print button as number when released
      Serial.println(buttons.getButtonRelease());
      break;
    case 'C':
        switch (buttons.checkButtonPress()) {
          case 0:
            break;
          case 1:
            buzzer.buzzer(); // 400Hz sound
            break;
          case 2:
            buzzer.buzzer(800); // 800Hz sound
            break;
          case 3:
            buzzer.buzzer(1200); // 1200Hz sound
            break;
          case 4:
            buzzer.buzzer(1600); // 1600Hz sound
            break;
          case 5:
            buzzer.buzzer(2000); // 2000Hz sound
            break;
          case 6:
            buzzer.buzzer(2400); // 2400Hz sound
            break;
          case 7:
            buzzer.buzzer(2800); // 2800Hz sound
            break;
          }
          delayMicroseconds(1);
          break;
  }
}