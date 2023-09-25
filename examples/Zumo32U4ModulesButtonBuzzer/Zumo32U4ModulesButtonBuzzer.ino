/*
 * Dette program viser eksempler på hvad class Zumo32U4ModulesButtonBuzzer kan alene.
 * Programmet starter ud med at vente til brugeren har trykket og sluppet en af de 3 knapper.
 * Efterfølgende vil programmet køre en af disse 3 sekvenser:
 * 1. Print kombination af de(n) knap(per) som trykkes ned. Kombinatorikken går således:
 *    INTET = 0; A = 1; B = 2; A+B = 3; C = 4; A+C = 5; B+C = 6; A+B+C = 7
 * 2. Print sluppet knap til Serial monitor. Hvor A=1; B=2; C=3
 * 3. Buzzer som instrument. Afhængigt af kombinationen af knapper som trykkes afspilles forskellige frekvenser.
 */

#include "Zumo32U4ModulesExperiment.h"
Zumo32U4ModulesButtonBuzzer zumoBot;

int protocol;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  protocol = zumoBot.buttonBootup();
}
void loop() {
  // put your main code here, to run repeatedly:
  switch(protocol) {
    }
    case 1: { // Aflæs mulige kombinationer af buttonPresses
      Serial.println(zumoBot.checkButtonPress());
      delay(50);
      break;
    }
    case 2: { // Print sluppet knap når sluppet
      Serial.println(zumoBot.getButtonRelease());
      break;
    }
    case 3: {
        switch (zumoBot.checkButtonPress()) {
          case 0:
            break;
          case 1:
            zumoBot.buzzer(); // 400Hz lyd
            break;
          case 2:
            zumoBot.buzzer(800); // 800Hz lyd
            break;
          case 3:
            zumoBot.buzzer(1200); // 1200Hz lyd
            break;
          case 4:
            zumoBot.buzzer(1600); // 1600Hz lyd
            break;
          case 5:
            zumoBot.buzzer(2000); // 2000Hz lyd
            break;
          case 6:
            zumoBot.buzzer(2400); // 2400Hz lyd
            break;
          case 7:
            zumoBot.buzzer(2800); // 2800Hz lyd
            break;
          }
          delayMicroseconds(1);
          break;
  }
}