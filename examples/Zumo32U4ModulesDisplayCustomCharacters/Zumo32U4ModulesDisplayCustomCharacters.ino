/*
 * This program shows how to make full use of custom characters with either display.
 * An example of how to define own custom character is provided.
 * By default, the 8 available custom characters are assigned to 8 different arrows.
 * 8 custom characters are at first shown together, then shown in order in an animation
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesOLED zumoBot;
// Zumo32U4ModulesLCD zumoBot; // If a LCD-display is used

int j;
const char FULLBAR[] PROGMEM = {31, 31, 31, 31, 31, 31, 31, 31}; // Literally makes a bar. Try adjusting the values
/*
const char FULLBAR[] PROGMEM = { // Do this alternatively. Compare this to the display... When you see it, you see it
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  zumoBot.displayMenu();
  delay(1000);
  zumoBot.displayCustomCharacters(forwardArrows, forwardArrowsSolid, reverseArrows, reverseArrowsSolid,
                          leftArrow, rightArrow, backArrow, FULLBAR); // OPTIONAL. Try using your own characters as parameters
  zumoBot.displayPrint("\0", true, false);
  zumoBot.displayPrint(" \1 \2 \3");
  zumoBot.displayPrint("\4 \5 \6 \7");
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = -3; i < 8; i++) {
    zumoBot.displayPrint("Hello!", true); 
    j = i;
    while (i < 0) { // Fill up void to make animation more fluent
        zumoBot.displayPrint("  ", false, false);
        i++;
    }
    switch(i) {
      case 0:
        zumoBot.displayPrint("\0", false, false); // Note that anything put before "\0" in this string will be ignored...
        zumoBot.displayPrint(" ", false, false); // ... thus this command is nessecary to make transition more fluent.
        i++;
      case 1:
        zumoBot.displayPrint("\1 ", false, false);
        i++;
      case 2:
        zumoBot.displayPrint("\2 ", false, false);
        i++;
      case 3:
        zumoBot.displayPrint("\3 ", false, false);
        i++;
      case 4:
        zumoBot.displayPrint("\4 ", false, false);
        i++;
      case 5:
        zumoBot.displayPrint("\5 ", false, false);
        i++;
      case 6:
        zumoBot.displayPrint("\6 ", false, false);
        i++;
      case 7:
        zumoBot.displayPrint("\7 ", false, false);
        i++;
    }
    i = j;
    Serial.println(i);
    delay(1000);
  }
}