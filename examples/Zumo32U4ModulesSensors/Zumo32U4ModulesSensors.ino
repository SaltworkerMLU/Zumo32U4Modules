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
Zumo32U4ModulesIMU IMU;
Zumo32U4ModulesLineSensors LineSensors;
Zumo32U4ModulesProximitySensors ProximitySensors;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.initIMU();
}

static char buffer[80];

void loop() {
  // put your main code here, to run repeatedly:
  LineSensors.getLineSensorValue(); // Store read values in lineSensorValues[3]
  IMU.getIMUvalue(); // Where 'm' only gives mag[], 'a' only acc[] & 'g' only gyro[]
  sprintf(buffer, "%i %i   \t%i %i %i  \t%i %i %i   \t%i %i %i   \t%i %i %i\n",
    ProximitySensors.getProximitySensorValue(0),
    ProximitySensors.getProximitySensorValue(1),
    LineSensors.lineSensorValues[0],
    LineSensors.lineSensorValues[1],
    LineSensors.lineSensorValues[2],
    *IMU.mag[0], // The use of pointers for IMU-values are needed due to how the arrays are stored in the class.
    *IMU.mag[1],
    *IMU.mag[2],
    *IMU.acc[0],
    *IMU.acc[1],
    *IMU.acc[2],
    *IMU.gyro[0],
    *IMU.gyro[1],
    *IMU.gyro[2]
  );
  Serial.print(buffer);
  delay(50);
}