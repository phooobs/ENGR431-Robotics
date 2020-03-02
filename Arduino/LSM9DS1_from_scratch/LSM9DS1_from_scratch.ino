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

  // normalize acceleration
  float accelerationLength = sqrt(pow(accelerationX, 2) + pow(accelerationY, 2) + pow(accelerationZ, 2));
  accelerationX /= accelerationLength;
  accelerationY /= accelerationLength;
  accelerationZ /= accelerationLength;

  // centers generated from GyroFindCenters.py and GyroData.txt
  const float magneticFieldCenterX = 17.61;
  const float magneticFieldCenterY = -19.495;
  const float magneticFieldCenterZ = -7.47;

  // center data
  magneticFieldX += magneticFieldCenterX;
  magneticFieldY += magneticFieldCenterY;
  magneticFieldZ += magneticFieldCenterZ;

  // calculate roll pitch and headding
  float roll = atan2(accelerationX, -accelerationZ) * 180 / PI + 180;
  float pitch = atan2(accelerationY, -accelerationZ) * 180 / PI + 180;

  /*
  rx*ry =
  [         cos(r),      0,         sin(r)]
  [  sin(p)*sin(r), cos(p), -cos(r)*sin(p)]
  [ -cos(p)*sin(r), sin(p),  cos(p)*cos(r)]
 
  ry*rx =
  [  cos(r), sin(p)*sin(r), cos(p)*sin(r)]
  [       0,        cos(p),       -sin(p)]
  [ -sin(r), cos(r)*sin(p), cos(p)*cos(r)]
  */
  // rotate
  float magneticFieldXRotated = cos(roll * PI / 180) * magneticFieldX + sin(pitch * PI / 180)*sin(roll * PI / 180) * magneticFieldY + cos(pitch * PI / 180)*sin(roll * PI / 180) * magneticFieldZ;
  float magneticFieldYRotated = cos(pitch * PI / 180) * magneticFieldY + -sin(pitch * PI / 180) * magneticFieldZ;
  float magneticFieldZRotated = -sin(roll * PI / 180) * magneticFieldX + cos(roll * PI / 180)*sin(pitch * PI / 180) * magneticFieldY + cos(pitch * PI / 180)*cos(roll * PI / 180) * magneticFieldZ;
  float headding = atan2(-magneticFieldXRotated, -magneticFieldYRotated) * -360 / PI;

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
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(headding);
  Serial.println();
}
