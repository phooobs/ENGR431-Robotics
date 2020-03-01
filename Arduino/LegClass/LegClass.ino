#include <PololuMaestro.h>

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

MiniMaestro maestro(maestroSerial);

void setup()
{
  // Set the serial baud rate.
  maestroSerial.begin(9600);
}

void loop()
{
  // Set the target of channel 0 to 1500 us, and wait 2 seconds.
  maestro.setTarget(0, 6000);
  delay(2000);

  // Set the target of channel 0 to 1750 us, and wait 2 seconds.
  maestro.setTarget(0, 7000);
  delay(2000);

  // Set the target of channel 0 to 1250 us, and wait 2 seconds.
  maestro.setTarget(0, 5000);
  delay(2000);
}


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
    
    void pose(float x, float y, float z, float &hipYawl, float &hipPitch, float &kneePitch){
      hipYawl = atan2(y - hipRootY_, x - hipRootX_) + hipRootAngle_;
      float hipOffsetX = hipLength_ * cos(hipYawl - hipRootAngle_);
      float hipOffsetY = hipLength_ * sin(hipYawl - hipRootAngle_);
      float distanceToPoint = sqrt(pow(x - hipRootX_ + hipOffsetX, 2) + pow(y - hipRootY_ + hipOffsetY, 2) + pow(z - hipRootZ_, 2));
      kneePitch = acos((pow(thighLength_, 2) + pow(shinLength_, 2) - pow(distanceToPoint, 2)) / (2 * thighLength_ * shinLength_));
      hipPitch = atan2(z - hipRootZ_, sqrt(pow(x - hipRootX_ - hipOffsetX, 2) + pow(y - hipRootY_ - hipOffsetY, 2))) + acos((pow(thighLength_, 2) - pow(shinLength_, 2) + pow(distanceToPoint, 2)) / (2 * thighLength_ * distanceToPoint));
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
