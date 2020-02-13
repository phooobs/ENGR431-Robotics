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
float Ax,Ay,Az;

//  These are my global variables for the raw magnetometer values
//  (I changed them from the basic example to match a calibration routine I had)
int compass_x, compass_y, compass_z;

// These are the normalized unit vectors to use for the direction of the field
float compass_x_cos, compass_y_cos, compass_z_cos, compass_mag;

LSM9DS1 imu;

/*
min_x: -1596         max_x: 7755
min_y: -5407         max_y: 3279
min_z: -4697         max_z: 3215
  
average_x: 3079.00   magnitude_x: 4675.00   x_cosine: 0.00
average_y: -1064.00   magnitude_y: 4343.00   y_cosine: 0.43
average_z: -741.00   magnitude_z: 3956.00   z_cosine: -0.93
*/

// CALABRATION
float compass_x_ave = 3079.00;
float compass_y_ave = -1064.00;
float compass_z_ave = -741.00;
float compass_x_mag = 4675.00;
float compass_y_mag = 4343.00;
float compass_z_mag = 3956.00;

void setup() {
  // setup pins
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(switch0, INPUT);
  pinMode(switch1, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(led, OUTPUT);
  
  Serial.begin(9600); // open serial port
  while(!Serial) {} // wait till serial port has connected

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
  switch(2 * digitalRead(switch1) + digitalRead(switch0)){ // get input from switches and convert to mode number
    case 0: // 00 Sit, wait and blink, repeat.
      if (lastMode != 0) { // mode change, print
        lastMode = 0;
        motor(0, 0);
        Serial.println("[00] blink and wait");
        delay(3000);
      }
      static bool toggle;
      if (toggle) {
        toggle = false;
      } else {
        toggle = true;
      }
      digitalWrite(led, toggle);
      motor(0, 0);
      delay(500);
      break;
    case 1: // 01 Straight and speed test.
      if (lastMode != 1) { // mode change, print
        lastMode = 1;
        Serial.println("[01] Go West");
        motor(0, 0);
        delay(3000);
      }
      read9DoF();
      getCompass();
      float heading;
      if (compass_y_cos == 0)
        heading = (compass_x_cos < 0) ? PI : 0;
      else
        heading = -atan2(compass_y_cos, compass_x_cos);
      heading -= DECLINATION * PI / 180;

      float dir = -PI/2;
      float error = heading - dir;
      if (error > PI) {
        error -= 2 * PI;
      }
      if (error < -PI) {
        error += 2 * PI;
      }
      Serial.print(heading);
      Serial.print(" ");
      Serial.println(error);
      motor(min(155, max(0, 255 - error * 50)), min(155, max(0, 255 + error * 50)));
      break;


























      
    case 2: // 10 Pivot and turn test
      if (lastMode != 2) { // mode change, print
        lastMode = 2;
        motor(0, 0);
        Serial.println("[10] Pivot and turn test");
        delay(3000);
        spinBoi();
      }
      break;
    case 3: // 11 Follow the wall test
      static bool goLeft = true;
      if (lastMode != 3) { // 11 Follow the wall test
        lastMode = 3;
        motor(0, 0);
        Serial.println("[11] Follow the wall test");
        delay(3000);
        while (analogRead(rightSensor) < 350 || analogRead(rightSensor) < 350) { // go forward untill see wall
          motor(100, 100);
        }
      }
      // follow wall
      if (goLeft) { // going left check right side
        if (analogRead(rightSensor) > 600) {
          motor(-30, 90); // turn away from right wall sharp
          delay(250);
        }
        if (analogRead(leftSensor) > 360) {
          motor(-100, -30); // turn away and back
          delay(750);
        } else if (analogRead(rightSensor) > 360) { // see right wall
          motor(70, 90); // turn away from right wall
          digitalWrite(led, HIGH);
        } else { // see no right wall
          motor(90, 70); // turn twords right wall
          digitalWrite(led, LOW);
        }
      } else { // going right check left side
        if (analogRead(rightSensor) > 600) {
          motor(90, -30); // turn away from left wall sharp
          delay(250);
        }
        if (analogRead(rightSensor) > 360) { // see wrong wall
          motor(-30, -100); // turn away and back
          delay(750);
        } else if (analogRead(leftSensor) > 360) { // see left wall
          motor(90, 70);
          digitalWrite(led, HIGH);
        } else { // see no left wall
          motor(70, 90);
          digitalWrite(led, LOW);
        }
      }
      break;
  }
}

void straightSpeedTest() {
  for (int i=0; i<100; i++){
    motor(i,i);
    delay(3);
  }
  delay(5400);
  for (int j=100; j>0; j--){
    motor(j,j);
    delay(3);
  }
  
  delay(3000);//Done, wait 3 sec
  
  for (int i=0; i<200; i++){
    motor(i,i);
    delay(3);
  }
  delay(2000);
  for (int j=200; j>0; j--){
    motor(j,j);
    delay(3);
  }

  delay(3000); // Wait
  
  //// Go Backwards
  for (int i=0; i<100; i++){
    motor(-i,-i);
    delay(3);
  }
  delay(5400);
  for (int j=100; j>0; j--){
    motor(-j,-j);
    delay(3);
  }
  
  delay(3000); // Wait, 3 sec
  
  for (int i=0; i<200; i++){
    motor(-i,-i);
    delay(3);
  }
  delay(2000);
  for (int j=200; j>0; j--){
    motor(-j,-j);
    delay(3);
  }
}

void spinBoi() {
  // Spin 360 degrees 3 times clockwise
  digitalWrite(dirL, LOW); //Left motor forward
  digitalWrite(dirR, LOW); //Right motor reverse
  analogWrite(pwmR, 100);
  analogWrite(pwmL, 100);
  delay(3700); // spin for about 3.405 seconds [ORIGINAL TIME]

  // Wait 1.5 second
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(4000);

  // Spin 360 degrees 3 times counterclockwise
  digitalWrite(dirL, HIGH); //Left motor reverse
  digitalWrite(dirR, HIGH); //Right motor forward
  analogWrite(pwmR, 100);
  analogWrite(pwmL, 100);
  delay(3800); // spin for about 3.405 seconds [ORIGINAL TIME]

  // Wait 1.5 second
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(4000);

  // turn 90 degrees right using left wheel
  digitalWrite(dirL, LOW); //Left motor forward
  analogWrite(pwmL, 100);
  analogWrite(pwmR, 0);
  delay(1000);
  
  // Wait 1.5 second
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(3500);

  // turn back -90 degrees using left wheel
  digitalWrite(dirL, HIGH); //Left motor reverse
  analogWrite(pwmL, 100);
  analogWrite(pwmR, 0);
  delay(810);

  // Wait 1.5 second
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(3500);

  // turn left using right wheel
  analogWrite(pwmL, 0);
  analogWrite(pwmR, 100);
  delay(1250);

  // Wait 1.5 second
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(4000);
  
  // turn back -90 using right wheel 
  digitalWrite(dirR,LOW); // Right motor reverse
  analogWrite(pwmL, 0);
  analogWrite(pwmR, 100);
  delay(815);

  // no power to wheels.
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
  delay(4000);
}

void motor(int left, int right) { // converts signals in range(-255, 255) to motor pon signals 
  if (left < 0) {
    digitalWrite(dirL, HIGH);
    analogWrite(pwmL, abs(left * 0.965));
  } else {
    digitalWrite(dirL, LOW);
    analogWrite(pwmL, abs(left * 0.965));
  }
  
  if (right < 0) {
    digitalWrite(dirR, LOW);
    analogWrite(pwmR, abs(right));
  } else {
    digitalWrite(dirR, HIGH);
    analogWrite(pwmR, abs(right));
  }
}

void read9DoF()
{
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

void getCompass()
{
  if ( imu.magAvailable() )
  {
    
// To read from the magnetometer, first call the
// readMag() function. When it exits, it'll update the
// mx, my, and mz variables with the most current data.
//
    imu.readMag();
    compass_x = imu.mx;
    compass_y = imu.my;
    compass_z = imu.mz;

//  Here is where the magnetic field gets converted to a unit vector
//  Note the various signs that force +X = forward, +Y = right, +Z = down
//  The orientation of your LSM9DS1 might make things different
//
//  But USE A RIGHT HANDED COORDINATE SYSTEM !!!!!
//
//  The global variables that end in _mag and _ave are from the calibration
//
    compass_x_cos = -(compass_x - compass_x_ave) / compass_x_mag;
    compass_y_cos = (compass_y - compass_y_ave) / compass_y_mag;
    compass_z_cos = -(compass_z - compass_z_ave) / compass_z_mag;
    compass_mag = sqrt(compass_x_cos*compass_x_cos+compass_y_cos*compass_y_cos+compass_z_cos*compass_z_cos);
    
    //Serial.print("***   Compass cosines before narmalizing: x = ");
    //Serial.print(compass_x_cos);
    //Serial.print(", y = ");
    //Serial.print(compass_y_cos);
    //Serial.print(", z = ");
    //Serial.print(compass_z_cos);
    //Serial.print(", Mag = ");
    //Serial.println(compass_mag);

//  Normalize it !!!  (Make a vector of length 1 pointing along Earth's field)
    compass_x_cos = compass_x_cos / compass_mag;
    compass_y_cos = compass_y_cos / compass_mag;
    compass_z_cos = compass_z_cos / compass_mag;
    compass_mag = sqrt(compass_x_cos*compass_x_cos+compass_y_cos*compass_y_cos+compass_z_cos*compass_z_cos);
    
  }
  
}
