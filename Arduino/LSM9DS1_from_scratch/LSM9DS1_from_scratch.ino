#include <Wire.h>
#include <Arduino_LSM9DS1.h> // can be downloaded through the arduino library manager

void setup() {
  Serial.begin(9600);
  IMU.begin();
}

void loop() {
  // get data from IMU
  float accelerationX, accelerationY, accelerationZ;
  IMU.readAcceleration(accelerationX, accelerationY, accelerationZ);
  float gyroscopeX, gyroscopeY, gyroscopeZ;
  IMU.readGyroscope(gyroscopeX, gyroscopeY, gyroscopeZ);
  float magneticFieldX, magneticFieldY, magneticFieldZ;
  IMU.readMagneticField(magneticFieldX, magneticFieldY, magneticFieldZ);

  float roll = atan2(accelerationX, -accelerationZ) * 180 / 3.14159 + 180; // correct
  float pitch = atan2(accelerationY, -accelerationZ) * 180 / 3.14159 + 180; // correct
  float headding = atan2(-magneticFieldX, -magneticFieldY) * 180 / 3.14159; // incorrect

  // set angle range to -180 to 180
  while (roll > 180) {
       roll -= 360;
  }
  while (roll < -180) {
       roll += 360;
  }
  while (pitch > 180) {
       pitch -= 360;
  }
  while (pitch < -180) {
       pitch += 360;
  }
  while (headding > 180) {
       headding -= 360;
  }
  while (headding < -180) {
       headding += 360;
  }

  // output
  Serial.print(magneticFieldX);
  Serial.print(" ");
  Serial.print(magneticFieldY);
  Serial.print(" ");
  Serial.print(magneticFieldZ);
  Serial.println();
}
