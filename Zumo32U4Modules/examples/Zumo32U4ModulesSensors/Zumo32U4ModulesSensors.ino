/*
 * Dette program viser hvordan klassen Zumo32U4ModulesSensors benyttes
 * Der initialiseres først Proximity Sensor og Line Follower Sensor vha. constructoren (don't worry abou it)
 * IMU initialiseres on the go ved brug af getIMUValue() hvor magnetometer, accelerometer & gyrometer aflæses.
 * Desuden udvindes Line Follower Sensor values i array lineSensorValues[3] med getLineSensorValues().
 * 
 * Når værdierne er aflæst fra sensorerne, er det blot et spørgsmål om at udvinde disse aflæsninger.
 * Alle aflæste værdier printes nemlig til Serial monitor i formaten vist under sprintf().
 */

#include "Zumo32U4Modules.h"
Zumo32U4ModulesSensors zumoBot;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

static char buffer[80];

void loop() {
  // put your main code here, to run repeatedly:
  zumoBot.getLineSensorValue(); // Lagrer resultat heri i array lineSensorValues[3]
  zumoBot.getIMUValue(); // Hvor char 'm' kun giver mag[], 'a' kun acc[] & 'g' kun gyro[]
  sprintf(buffer, "%i %i   \t%i %i %i  \t%i %i %i   \t%i %i %i   \t%i %i %i\n",
    zumoBot.getProximitySensorValue(0),
    zumoBot.getProximitySensorValue(1),
    zumoBot.lineSensorValues[0],
    zumoBot.lineSensorValues[1],
    zumoBot.lineSensorValues[2],
    *zumoBot.mag[0], // Bemærk brugen af en pointer *. Denne skal være med grundet hvordan IMU-værdierne lagres i klassen
    *zumoBot.mag[1],
    *zumoBot.mag[2],
    *zumoBot.acc[0],
    *zumoBot.acc[1],
    *zumoBot.acc[2],
    *zumoBot.gyro[0],
    *zumoBot.gyro[1],
    *zumoBot.gyro[2]
  );
  Serial.print(buffer);
  delay(50);
}