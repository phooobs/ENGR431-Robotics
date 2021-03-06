#include <Wire.h>
#include <SparkFunLSM9DS1.h>

#define DECLINATION -9.16 // Declination (degrees) in Durango, CO.

const int switch0 = 10; // pin connected to switch 0
const int switch1 = 9; // pin connected to switch 1

const int dirL = 4; // left motor direction pin
const int dirR = 2; // right motor direction pin
const int pwmL = 11; // left motor speed pin
const int pwmR = 3; // right motor speed pin

const int led = 12;
const int leftSensor = A0;
const int rightSensor = A1;

void motor(int left, int right); // converts signals in range(-255, 255) to motor pon signals
void straightSpeedTest();
void read9DoF();

//  My global variable for accelerometer values (use the Sparkfun calibration)
float Ax, Ay, Az;

//  These are my global variables for the raw magnetometer values
//  (I changed them from the basic example to match a calibration routine I had)
int compass_x, compass_y, compass_z;

// These are the normalized unit vectors to use for the direction of the field
float compass_x_cos, compass_y_cos, compass_z_cos, compass_mag;

LSM9DS1 imu;

// CALABRATION
float compass_x_ave = 2100.0;
float compass_y_ave = -1590.0;
float compass_z_ave = -773.0;
float compass_x_mag = 3394.0;
float compass_y_mag = 3128.0;
float compass_z_mag = 3180.0;
float pitch, roll, heading;

void setup() {
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(switch0, INPUT);
  pinMode(switch1, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(led, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {} // wait till serial port has connected

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
}

void loop() {
  static int lastMode = -1; // keep track of what mode we are on so when there is a change it can be detected, -1 means no last mode

  if (digitalRead(switch0) == 0 && digitalRead(switch1) == 0) { // Sit
    if (lastMode != 0) {
      lastMode = 0;
      motor(0, 0);
      Serial.println("[00] Standing By...\n");
      delay(3000);
    }
  }
  else if (digitalRead(switch0) == 1 && digitalRead(switch1) == 0) { // PRY 
    if (lastMode != 2) {
      lastMode = 2;
      motor(0, 0);
      Serial.println("[10] Read Pitch/Roll/Heading\n");
      delay(500);
      Serial.println("Current Values:");
      Serial.println("X: Avg:: " + String(compass_x_ave) + " || Mag:: " + String(compass_x_mag));
      Serial.println("Y: Avg:: " + String(compass_y_ave) + " || Mag:: " + String(compass_y_mag));
      Serial.println("Z: Avg:: " + String(compass_z_ave) + " || Mag:: " + String(compass_z_mag) + "\n");
      delay(2500);
      Serial.println("Pitch || Roll || Heading");
      delay(100);
    }
    getOneTRHeading();
    
    Serial.print(pitch);
    Serial.print(" || ");
    Serial.print(roll);
    Serial.print(" || ");
    Serial.println(heading);
    //delay(500);
  }
  else if (digitalRead(switch0) == 0 && digitalRead(switch1) == 1) { // West
    if (lastMode != 1) { // mode change, print
      lastMode = 1;
      Serial.println("[01] Go West\n");
      motor(0, 0);
      delay(3000);
    }
    read9DoF();
    getCompass();

    if (compass_y_cos == 0)
      heading = (compass_x_cos < 0) ? PI : 0;
    else
      heading = -atan2(compass_y_cos, compass_x_cos);
    heading -= DECLINATION * PI / 180;

    float dir = -0.8;
    float error = heading - dir;

    while (error > PI) {
      error -= 2 * PI;
    }
    while (error < -PI) {
      error += 2 * PI;
    }
    if (analogRead(leftSensor) >= 650) {
      motor(-100, -50);
      delay(1550);
    }
    else if (analogRead(rightSensor) >= 650) {
      motor(-50, -100);
      delay(1550);
    }
    else {
      motor(min(100, max(0, 255 - error * 150)), min(100, max(0, 255 + error * 150)));
    }
  }
  else { // NA
     if (lastMode != 3)
      {
        lastMode = 3;
        motor(0, 0);
        Serial.println("[11] <!> EMPTY <!>\n");
        delay(3000);
      }
      Serial.println("...");
      delay(500);
  }
}

void motor(int left, int right) { // converts signals in range(-255, 255) to motor pon signals
  if (left < 0) {
    digitalWrite(dirL, HIGH);
    analogWrite(pwmL, abs(left * 0.98));
  } else {
    digitalWrite(dirL, LOW);
    analogWrite(pwmL, abs(left * 0.98));
  }

  if (right < 0) {
    digitalWrite(dirR, LOW);
    analogWrite(pwmR, abs(right));
  } else {
    digitalWrite(dirR, HIGH);
    analogWrite(pwmR, abs(right));
  }
}

void read9DoF() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    Ax = imu.calcAccel(imu.ax);
    Ay = imu.calcAccel(imu.ay);
    //    Az = imu.calcAccel(imu.az);
    //  Here is my change to try to get a right handed system. !!!
    Az = -imu.calcAccel(imu.az);
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    compass_x = imu.mx;
    compass_y = imu.my;
    compass_z = imu.mz;
  }
}

void getCompass() {
  if ( imu.magAvailable() )
  {
    imu.readMag();
    compass_x = imu.mx;
    compass_y = imu.my;
    compass_z = imu.mz;
    
    compass_x_cos = -(compass_x - compass_x_ave) / compass_x_mag;
    compass_y_cos = (compass_y - compass_y_ave) / compass_y_mag;
    compass_z_cos = -(compass_z - compass_z_ave) / compass_z_mag;
    compass_mag = sqrt(compass_x_cos * compass_x_cos + compass_y_cos * compass_y_cos + compass_z_cos * compass_z_cos);

    compass_x_cos = compass_x_cos / compass_mag;
    compass_y_cos = compass_y_cos / compass_mag;
    compass_z_cos = compass_z_cos / compass_mag;
    compass_mag = sqrt(compass_x_cos * compass_x_cos + compass_y_cos * compass_y_cos + compass_z_cos * compass_z_cos);
  }
}

void getOneHeading() {
  float tan_heading, z_sin;
  float dir_sin, dir_cos;

  getCompass();

  if (compass_y_cos == 0)
    heading = (compass_x_cos < 0) ? PI : 0;
  else
    heading = -atan2(compass_y_cos, compass_x_cos);

  heading -= DECLINATION * PI / 180;

  if (heading > 2 * PI) heading -= (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  heading *= 180.0 / PI;

}

void getOneTRHeading() {
  //  float tan_heading, z_sin;
  float dir_sin, dir_cos;
  //  float pitch,roll,pitchdeg,rolldeg;
  float magXcomp, magYcomp;

  read9DoF();      // for Acceleration
  getCompass();    // for magnetometer (normalization added)
  
  pitch = asin(Ax);
  
  roll = -asin(Ay / cos(pitch));
  
  magXcomp = compass_x_cos * cos(pitch) + compass_z_cos * sin(pitch);
  magYcomp = compass_x_cos * sin(roll) * sin(pitch) + compass_y_cos * cos(roll) - compass_z_cos * sin(roll) * cos(pitch);

  if (compass_y_cos == 0)
    heading = (compass_x_cos < 0) ? PI : 0;
  else
    heading = -atan2(magYcomp, magXcomp);

  heading -= DECLINATION * PI / 180;

  if (heading > 2 * PI) heading -= (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  // Convert from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180. / PI;   // for printing
  roll *= 180. / PI;   // for printing

}
