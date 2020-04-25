#include <PololuMaestro.h>

// class for poseing a leg
// +x : right, +y : forward, +z : up
class Leg {
  public:
    Leg(float hipRootX, float hipRootY, float hipRootZ, float hipRootAngle, float hipLength, float thighLength, float shinLength, MiniMaestro* maestro, int type, int hipYawlServo, int hipPitchServo, int kneePitchServo){
      hipRootX_ = hipRootX;
      hipRootY_ = hipRootY;
      hipRootZ_ = hipRootZ;
      hipRootAngle_ = hipRootAngle;
      hipLength_ = hipLength;
      thighLength_ = thighLength;
      shinLength_ = shinLength;
      maestro_ = maestro;
      type_ = type;
      hipYawlServo_ = hipYawlServo;
      hipPitchServo_ = hipPitchServo;
      kneePitchServo_ = kneePitchServo;
    }

    // calculates angles for a pose
    // +x : right, +y : forward, +z : up
    // angles are in radians
    void calculateAngles(float x, float y, float z, float* hipYawl, float* hipPitch, float* kneePitch){
      *hipYawl = atan2(y - hipRootY_, x - hipRootX_) + hipRootAngle_ - PI / 2;
      float hipOffsetX = hipLength_ * -sin(*hipYawl - hipRootAngle_);
      float hipOffsetY = hipLength_ * cos(*hipYawl - hipRootAngle_);
      float distanceToPoint = sqrt(pow(x - hipRootX_ - hipOffsetX, 2) + pow(y - hipRootY_ - hipOffsetY, 2) + pow(z - hipRootZ_, 2));
      int sign;
      if (sqrt(pow(x - hipRootX_, 2) + pow(y - hipRootY_, 2)) < hipLength_) {
        sign = -1;
      } else {
        sign = 1;
      }
      *hipYawl += PI / 2;
      *kneePitch = PI - acos((pow(thighLength_, 2) + pow(shinLength_, 2) - pow(distanceToPoint, 2)) / (2 * thighLength_ * shinLength_));
      *hipPitch = atan2(z - hipRootZ_, sign * sqrt(pow(x - hipRootX_ - hipOffsetX, 2) + pow(y - hipRootY_ - hipOffsetY, 2))) + acos((pow(thighLength_, 2) + pow(distanceToPoint, 2) - pow(shinLength_, 2)) / (2 * thighLength_ * distanceToPoint)) + PI / 2;
      if (type_ == 1) { // mirror all angles for mirrired leg
        *hipYawl = PI - *hipYawl;
        *hipPitch = PI - *hipPitch;
        *kneePitch = PI - *kneePitch;
      }
    }

    // calculates angles and then moves leg
    // +x : right, +y : forward, +z : up
    void pose (float x, float y, float z) {
      float hipYawl, hipPitch, kneePitch;
      calculateAngles(x, y, z, &hipYawl, &hipPitch, &kneePitch);
      maestro_->setTarget(hipYawlServo_, convertAngle(hipYawl));
      maestro_->setTarget(hipPitchServo_, convertAngle(hipPitch));
      maestro_->setTarget(kneePitchServo_, convertAngle(kneePitch));
    }

    // converts angle in radians to pololu signal
    // angle to us may be inacurate
    float convertAngle (float angle) {
      float us = 180 / PI * angle / 90.0 + 0.5; // convert angle to us
      return us * 4000; // convert us to MiniMaestro spec
    }
    
  private:
    float hipRootX_;
    float hipRootY_;
    float hipRootZ_;
    float hipRootAngle_;
    float hipLength_;
    float thighLength_;
    float shinLength_;
    MiniMaestro* maestro_;
    int type_;
    int hipYawlServo_, hipPitchServo_, kneePitchServo_;
};
