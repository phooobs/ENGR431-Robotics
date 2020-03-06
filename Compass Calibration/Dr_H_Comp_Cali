//  These add some external libraries - I2C and tge 9DoF sensor stick
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

#define compassaddress 0x1E

// print flags copied from the Example Code, but I mostly don't use them
#define PRINT_CALCULATED
//#define PRINT_RAW
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
// #define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define DECLINATION -9.16 // Declination (degrees) in Durango, CO.

// Definitions left over from motion code

int PWM_RIGHTpin = 3;   // Right Wheel Speed
int PWM_LEFTpin = 11;  // Left Wheel Speed
int dir_right = 2;  // Right Wheel Direction
int dir_left = 4;  // Left Wheel Direction

int ledred = 5;
int ledyellow = 6;
int ledgreen = 7;
int ledblue = 8;

int CTRL0 = 9;
int CTRL1 = 10;
int control = 0;  //  This is the single variable 2*CTRL1 + CTRL0

int IRsensorPinL = A0;    // Define the input pin for the IR sensor
int IRsensorPinR = A1;    // Define the input pin for the IR sensor
int IRsensorValueL = 0;   // Variable to store the Integer value coming from the IR sensor
int IRsensorValueR = 0;   // Variable to store the Integer value coming from the IR sensor

int waittime = 1000;   // How long to wait between events.  Mostly for testing.  0 for running.

//=============================================================================//    
//  Calibrations for my specific motors.  
//  Slope and intercept depend slightly on voltage.
//  Can generally (approximately) just change the speedpercent with different voltages.
//
float speedpercent = 75;
float left_motor_slope = 1.8;
float right_motor_slope = 1.61;
float left_motor_slope_rev = 1.37;
float right_motor_slope_rev = 1.48;
float left_motor_int = 62.;
float right_motor_int = 52.;
float left_motor_int_rev = 75.;
float right_motor_int_rev = 55.;

// These just fill in some default starting values with 75%
int left_fwd = left_motor_slope*speedpercent + left_motor_int;
int right_fwd = right_motor_slope*speedpercent + right_motor_int;
int left_rev = left_motor_slope*speedpercent + left_motor_int;
int right_rev = right_motor_slope*speedpercent + right_motor_int;

//  Approximate scale factor from speedpercent to degrees rotation
float rotate_cal = 3.50;
// end of motor stuff
//=============================================================================//    

//  My global variable for accelerometer values (use the Sparkfun calibration)
float Ax,Ay,Az;

//  These are my global variables for the raw magnetometer values
//  (I changed them from the basic example to match a calibration routine I had)
int compass_x, compass_y, compass_z;

// values from try 5
float compass_x_ave = 1094.;
float compass_y_ave = 1095.;
float compass_z_ave = 361.;
float compass_x_mag = 2298.;
float compass_y_mag = 2462.;
float compass_z_mag = 2463.;

// These are the normalized unit vectors to use for the direction of the field
float compass_x_cos, compass_y_cos, compass_z_cos, compass_mag;
  
void setup()
{
  Serial.begin(9600);  

  pinMode(PWM_RIGHTpin, OUTPUT);
  pinMode(PWM_LEFTpin, OUTPUT);
  pinMode(dir_right, OUTPUT);
  pinMode(dir_left, OUTPUT);

  pinMode(IRsensorPinL, INPUT);
  pinMode(IRsensorPinR, INPUT);
  
  pinMode(CTRL0, INPUT);
  pinMode(CTRL1, INPUT);
  
  analogWrite(PWM_RIGHTpin, 0);
  analogWrite(PWM_LEFTpin, 0);

  //set up I2C comm
  Wire.begin();

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

//=============================================================================//    
void loop()
{

  getControl();    // These are from pins 9 and 10
  read9DoF();      // This is mostly from the LSM9DS1_Basic_I2C code example
  getCompass();    // This is my mod for the magnetometer only
  printAttitude(); // This is my mod of the example code to get 2D heading

  if (control == 1) calibrate_compass();
  else wait(waittime);

 }

//=============================================================================//    
// *** get the control bits ***

void getControl()
{
  control = 2*digitalRead(CTRL1) + digitalRead(CTRL0);
//  Serial.print("control: ");
//  Serial.println(control);
}

//=============================================================================//    
// read9DoF - Read the 9DoF sensors

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

printGyro();
printAccel();
printMag();
 
}

//=============================================================================//    
// WAIT
void wait(int thiswaittime)
{
  int countnow = 0;

  Serial.println("Waiting...");
  
  while (countnow <= thiswaittime)
  {
    countnow = countnow+400;
    delay(250);
    digitalWrite(ledblue, HIGH);
    delay(250);
    digitalWrite(ledblue, LOW);
  }
  
}
//=============================================================================//    
// Print Gyroscope Values - 
//
void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}


//=============================================================================//    
// Print Acceleration Values - 
//
void printAccel()
{
//  I just commented the Sparkfun print routines so I could print Ax, Ay, Az
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
  Serial.print(Ax);
  Serial.print(", ");
  Serial.print(Ay);
  Serial.print(", ");
  Serial.println(Az);
//#ifdef PRINT_CALCULATED
//  // If you want to print calculated values, you can use the
//  // calcAccel helper function to convert a raw ADC value to
//  // g's. Give the function the value that you want to convert.
//  Serial.print(imu.calcAccel(imu.ax), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.ay), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.az), 2);
//  Serial.println(" g");
//#elif defined PRINT_RAW
//  Serial.print(imu.ax);
//  Serial.print(", ");
//  Serial.print(imu.ay);
//  Serial.print(", ");
//  Serial.println(imu.az);
//#endif

}

//=============================================================================//    
// Print Magnetometer Values - 
//
void printMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
  Serial.print(compass_x);
  Serial.print(", ");
  Serial.print(compass_y);
  Serial.print(", ");
  Serial.println(compass_z);
//#ifdef PRINT_CALCULATED
//  // If you want to print calculated values, you can use the
//  // calcMag helper function to convert a raw ADC value to
//  // Gauss. Give the function the value that you want to convert.
//  Serial.print(imu.calcMag(imu.mx), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.my), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.mz), 2);
//  Serial.println(" gauss");
//#elif defined PRINT_RAW
//  Serial.print(imu.mx);
//  Serial.print(", ");
//  Serial.print(imu.my);
//  Serial.print(", ");
//  Serial.println(imu.mz);
//#endif
}

//=============================================================================//    
// calibrate the compass
// 
//  Here is the routine that just updates the maximum and minimum values
//  while you move the compass around.
//  At the end it prints out the average and the scale factors you can 
//  put in your code.

void calibrate_compass()
{
  int min_x,max_x,min_y,max_y,min_z,max_z;
  
  getCompass();

  min_x = compass_x;
  max_x = compass_x;
  min_y = compass_y;
  max_y = compass_y;
  min_z = compass_z;
  max_z = compass_z;

  while (control != 0)
  {
    // go get another compass reading
    getCompass();
//    getControl();
  control = 2*digitalRead(CTRL1) + digitalRead(CTRL0);

    
    if (compass_x <= min_x) min_x = compass_x;
    if (compass_x >= max_x) max_x = compass_x;
    if (compass_y <= min_y) min_y = compass_y;
    if (compass_y >= max_y) max_y = compass_y;
    if (compass_z <= min_z) min_z = compass_z;
    if (compass_z >= max_z) max_z = compass_z;
        
//    Serial.print("  x: ,");
    Serial.print(compass_x);
    Serial.print(",");
    Serial.print(compass_y);
    Serial.print(",");
    Serial.println(compass_z);
      
    delay(100);
  }
 
 // get the final compass heading for the direction
 
 getCompass();
 
 // do some calculations
 
 compass_x_ave = (max_x + min_x) / 2;
 compass_y_ave = (max_y + min_y) / 2;
 compass_z_ave = (max_z + min_z) / 2;

 compass_x_mag = (max_x - min_x) / 2;
 compass_y_mag = (max_y - min_y) / 2;
 compass_z_mag = (max_z - min_z) / 2;
    
 compass_x_cos = (compass_x - compass_x_ave) / compass_x_mag;
 compass_y_cos = (compass_y - compass_y_ave) / compass_y_mag;
 compass_z_cos = (compass_z - compass_z_ave) / compass_z_mag;
 
//  Print what you got

  Serial.println("  ");
  Serial.print("min_x: ");
  Serial.print(min_x);
  Serial.print("         max_x: ");
  Serial.println(max_x);
  Serial.print("min_y: ");
  Serial.print(min_y);
  Serial.print("         max_y: ");
  Serial.println(max_y);
  Serial.print("min_z: ");
  Serial.print(min_z);
  Serial.print("         max_z: ");
  Serial.println(max_z);
  Serial.println("  ");
  
  Serial.print("average_x: ");
  Serial.print(compass_x_ave);
  Serial.print("   magnitude_x: ");
  Serial.print(compass_x_mag);
  Serial.print("   x_cosine: ");
  Serial.println(compass_x_cos);
  Serial.print("average_y: ");
  Serial.print(compass_y_ave);
  Serial.print("   magnitude_y: ");
  Serial.print(compass_y_mag);
  Serial.print("   y_cosine: ");
  Serial.println(compass_y_cos);
  Serial.print("average_z: ");
  Serial.print(compass_z_ave);
  Serial.print("   magnitude_z: ");
  Serial.print(compass_z_mag);
  Serial.print("   z_cosine: ");
  Serial.println(compass_z_cos);
  Serial.println("  ");
  Serial.println("  ");
  
  wait(3*waittime);

}

//=============================================================================//    
//
// This gets one reading from the magnetometer, converts to robot frame,
// normalizes the magnetic field vector to unity, and prints the values
//
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
    
//    Serial.print("***   Compass cosines before narmalizing: x = ");
//    Serial.print(compass_x_cos);
//    Serial.print(", y = ");
//    Serial.print(compass_y_cos);
//    Serial.print(", z = ");
//    Serial.print(compass_z_cos);
//    Serial.print(", Mag = ");
//    Serial.println(compass_mag);

//  Normalize it !!!  (Make a vector of length 1 pointing along Earth's field)
    compass_x_cos = compass_x_cos / compass_mag;
    compass_y_cos = compass_y_cos / compass_mag;
    compass_z_cos = compass_z_cos / compass_mag;
    compass_mag = sqrt(compass_x_cos*compass_x_cos+compass_y_cos*compass_y_cos+compass_z_cos*compass_z_cos);
    
  }
  
}

//=============================================================================//    
//
//  This is a modification of what was in the LSM9DS1_Basic_I2C code
//  Much of the original code is included, but commented here.
//  I did not send in any values, since they are global variable.
//
void printAttitude()
{
  // Update to my variables and directions
  //
  // Heading is 2D only !!!  The robot must be on a flat surface.
  //
//  float roll = atan2(ay, az);
//  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float roll = -atan2(Ay, -Az);
  float pitch = -atan2(-Ax, sqrt(Ay * Ay + Az * Az));

  float heading;
//  if (my == 0)
//    heading = (mx < 0) ? PI : 0;
//  else
//    heading = atan2(mx, my);
  if (compass_y_cos == 0)
    heading = (compass_x_cos < 0) ? PI : 0;
  else
    heading = -atan2(compass_y_cos, compass_x_cos);

  heading -= DECLINATION * PI / 180;

//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//
//  The challenge beacon transmits 0-180 which you need to
//  multiply by 2 to get 0-360 (instead of -180 - 180)
//
  if (heading > 2*PI) heading -= (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); 
  Serial.println(heading, 2);
  Serial.println(" ");
}
