#include <PololuMaestro.h>

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

MiniMaestro maestro(maestroSerial);

class Leg {
  public:
    Leg(float hipRootX, float hipRootY, float hipRootZ, float hipRootAngle, float hipLength, float thighLength, float shinLength){
      hipRootX_ = hipRootX;
      hipRootY_ = hipRootY;
      hipRootZ_ = hipRootZ;
      hipRootAngle_ = hipRootAngle;
      hipLength_ = hipLength;
      thighLength_ = thighLength;
      shinLength_ = shinLength;
    }
    
    void pose(float x, float y, float z, float* hipYawl, float* hipPitch, float* kneePitch){
      *hipYawl = atan2(y - hipRootY_, x - hipRootX_) + hipRootAngle_ - PI / 2;
      float hipOffsetX = hipLength_ * cos(*hipYawl - hipRootAngle_ + PI / 2);
      float hipOffsetY = hipLength_ * sin(*hipYawl - hipRootAngle_ + PI / 2);
      float distanceToPoint = sqrt(pow(x - hipRootX_ - hipOffsetX, 2) + pow(y - hipRootY_ - hipOffsetY, 2) + pow(z - hipRootZ_, 2));
      Serial.print(hipOffsetX);
      Serial.print(" ");
      Serial.print(hipOffsetY);
      Serial.print(" ");
      Serial.print(distanceToPoint);
      Serial.print(" ");
      *hipYawl += PI / 2;
      *kneePitch = acos((pow(thighLength_, 2) + pow(shinLength_, 2) - pow(distanceToPoint, 2)) / (2 * thighLength_ * shinLength_));
      *hipPitch = PI - ((PI - atan2(z - hipRootZ_, sqrt(pow(x - hipRootX_ - hipOffsetX, 2) + pow(y - hipRootY_ - hipOffsetY, 2)))) + acos((pow(thighLength_, 2) + pow(distanceToPoint, 2) - pow(shinLength_, 2)) / (2 * thighLength_ * distanceToPoint)) - 3 * PI / 2) - PI / 2;
    }
    
  private:
    float hipRootX_;
    float hipRootY_;
    float hipRootZ_;
    float hipRootAngle_;
    float hipLength_;
    float thighLength_;
    float shinLength_;
};

int convertAngle(int angle) {
  float us = angle / 90.0 + 0.5; // convert angle to us
  //Serial.print(angle);
  //Serial.print(" ");
  //Serial.print(us);
  //Serial.print(" ");
  //Serial.println(us * 4000);
  return us * 4000; // convert us to MiniMaestro spec
}

void angleTest() {
  Serial.println("0");
  maestro.setTarget(2, convertAngle(0));
  maestro.setTarget(1, convertAngle(0));
  delay(2000);

  Serial.println("180");
  maestro.setTarget(2, convertAngle(180));
  maestro.setTarget(1, convertAngle(180));
  delay(2000);
}

void setup()
{
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);

  maestro.setTarget(0, convertAngle(180));
  maestro.setTarget(1, convertAngle(180));
  maestro.setTarget(2, convertAngle(180));

  delay(2000);
}


void loop() {
  //angleTest();
  Legline();
}

Leg testLeg(0, 0, 0, 0, 27, 83, 140);
void Legline () {
  static int pos;
  pos = pos % 100 + 1;
  float angle0, angle1, angle2;
  testLeg.pose(0, 0, -pos - 100, &angle0, &angle1, &angle2);

  angle0 *= 180 / 3.14;
  angle1 *= 180 / 3.14;
  angle2 *= 180 / 3.14;


  Serial.print(angle0);
  Serial.print(" ");
  Serial.print(angle1);
  Serial.print(" ");
  Serial.print(angle2);
  Serial.println();
  maestro.setTarget(0, convertAngle(angle0));
  maestro.setTarget(1, convertAngle(angle1));
  maestro.setTarget(2, convertAngle(angle2));
  delay(10);
}
