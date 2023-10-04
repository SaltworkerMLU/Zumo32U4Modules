/*
 * This program shows how to use the class Zumo32U4ModulesSensors.
 * First, initialize proximity- and lineFollowerSensors (don't worry abou it)
 * IMU will be initialized on the go with getIMUValue() where magnetometer, accelerometer & gyrometer are read.
 * lineFollowerSensor values are stored in lineSensorValues[3] 
 * proximitySensor returns one value at a time using getProximitySensorValue(bool)
 * 
 * When these values are read, it's only a matter is printing all values to Serial.monitor using sprintf()
 */

#include <Zumo32U4Modules.h>
Zumo32U4ModulesSensors zumoBot;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

static char buffer[80];

void loop() {
  // put your main code here, to run repeatedly:
  zumoBot.getLineSensorValue(); // Store read values in lineSensorValues[3]
  zumoBot.getIMUValue(); // Where 'm' only gives mag[], 'a' only acc[] & 'g' only gyro[]
  sprintf(buffer, "%i %i   \t%i %i %i  \t%i %i %i   \t%i %i %i   \t%i %i %i\n",
    zumoBot.getProximitySensorValue(0),
    zumoBot.getProximitySensorValue(1),
    zumoBot.lineSensorValues[0],
    zumoBot.lineSensorValues[1],
    zumoBot.lineSensorValues[2],
    *zumoBot.mag[0], // The use of pointers for IMU-values are needed due to how the arrays are stored in the class.
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