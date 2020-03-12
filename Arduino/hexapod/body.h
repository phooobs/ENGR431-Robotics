#include <PololuMaestro.h>
#include "leg.h"

class Body {
  public:
    Body (float hipLength, float thighLength, float shinLength, MiniMaestro* maestro, float phaseOfset[6], float legTargetCenter[6][3]) :
    legs_{Leg( 40, 150, 0,     0, hipLength, thighLength, shinLength, maestro, 0, 0, 1, 2),
          Leg( 80,   0, 0,  1.57, hipLength, thighLength, shinLength, maestro, 1, 3, 4, 5),
          Leg( 40, -70, 0,  1.74, hipLength, thighLength, shinLength, maestro, 0, 6, 7, 8),
          Leg(-40, -70, 0, -1.74, hipLength, thighLength, shinLength, maestro, 1, 9, 10, 11),
          Leg(-80,   0, 0, -1.57, hipLength, thighLength, shinLength, maestro, 0, 12, 13, 14),
          Leg(-40, 150, 0,     0, hipLength, thighLength, shinLength, maestro, 1, 15, 16, 17)} {
      for (int i =  0; i < 6; i++) {
        phaseOfset_[i] = phaseOfset[i];
        for (int j =  0; j < 3; j++) {
          legTargetCenter_[i][j] = legTargetCenter[i][j];  
        }
      }
    }

    void setSpeed (int speed) {
      speed_ = speed;
    }

    void setDirection (float direction) {
      direction_ = direction;
    }

    void setTurnRadius (float radius) {
      radius_ = radius;
    }

    void updatePose () {
      static int lastTime;
      int curentTime = millis();
      phase_ = (phase_ + speed_ * (curentTime - lastTime) / 1000.0);
      while (phase_ > 2 * PI) {
        phase_ -= 2 * PI;
      }
      lastTime = curentTime;

      for (int i = 0; i < 6; i++) {
        float phase = (phase_ + phaseOfset_[i]);
        while (phase > 2 * PI) {
          phase -= 2 * PI;
        }
        float height = -90 * max(PI / 3, abs(phase - PI));
        float position;
        if (abs(phase - PI) < PI / 3) {
          position = 50 * -3 / PI * (phase - PI);    
        } else {
          phase += PI;
          while (phase > 2 * PI) {
            phase -= 2 * PI;
          }
          phase -= PI;
          position = 50 * 3 / (4 * PI) * phase;
        }
        legs_[i].pose(position * cos(direction_ + PI / 2) - legTargetCenter_[i][0], position * sin(direction_ + PI / 2) - legTargetCenter_[i][1], height - legTargetCenter_[i][2]);
      }
    }
    
  private:
    enum legNames_ {frontRight, middleRight, backRight, backLeft, middleLeft, frontLeft};
    Leg legs_ [6];
    float phase_; // 0 to 2 * pi
    float legPhase_ [6];
    float speed_;
    float direction_;
    float radius_;
    float phaseOfset_ [6];
    float legTargetCenter_[6][3];
};
