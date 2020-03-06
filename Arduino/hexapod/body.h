#include <leg.h>

class Body {
  public:
    Body (float hipLength, float thighLength, float shinLength, MiniMaestro* maestro) :
    legs_{Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 0, 1, 2),
          Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 3, 4, 5),
          Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 6, 7, 8),
          Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 9, 10, 11),
          Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 12, 13, 14),
          Leg(hipRootX, hipRootY, hipRootZ, hipRootAngle, hipLength, thighLength, shinLength, maestro, type, 15, 16, 17)} {
      
    }

    void setSpeed () {
      
    }

    void setDirection () {
      
    }

    void setTurnRadius () {
      
    }

    void updatePose () {
      
    }
    
  private:
    enum legNames_ {frontRight, middleRight, backRight, backLeft, middleLeft, frontLeft};
    Leg legs_ [6];
    float plase_;
    float legPhase_ [6];
    float speed;
    float direction;
    float radius_;
};
