#include <PololuMaestro.h>
#include "body.h"

#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 11);
#endif

MiniMaestro maestro(maestroSerial);

float phase[6] = {0, 2 * PI / 3, 4 * PI / 3, 0, 2 * PI / 3, 4 * PI / 3};
float bases[6][3] = {{60,170,0},{80,0,0},{60,-90,0},{-60,-90,0},{-80,0,0},{-60,170,0}};

Body body(27, 83, 140, &maestro, phase, bases);

void setup () {
  body.setSpeed(1);
  body.setDirection(0);
}

void loop () {
  body.updatePose();
}
