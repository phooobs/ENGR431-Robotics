const int switch0 = 10; // pin connected to switch 0
const int switch1 = 9; // pin connected to switch 1

const int potPin = A0; // pin connected to potentiometer

const int dirL = 2; // left motor direction pin
const int dirR = 4; // right motor direction pin
const int pwmL = 11; // left motor speed pin
const int pwmR = 3; // right motor speed pin

void motor(int left, int right); // converts signals in range(-255, 255) to motor pon signals 

void setup() {
  // setup pins
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(switch0, INPUT);
  pinMode(switch1, INPUT);
  pinMode(potPin, INPUT);
  
  Serial.begin(9600); // open serial port
  while(!Serial) {} // wait till serial port has connected
}

void loop() {
  static int lastMode = -1; // keep track of what mode we are on so when there is a change it can be detected, -1 means no last mode
  switch(2 * digitalRead(switch1) + digitalRead(switch0)){ // get input from switches and convert to mode number
    case 0: // randomize all LEDs
      if (lastMode != 0) { // mode change, print
        lastMode = 0;
        Serial.println("off");
      }
      motor(0, 0);
      break;
    case 1: // turn on one LED at a time moving across all four LEDs
      if (lastMode != 1) { // mode change, print
        lastMode = 1;
        Serial.println("Wiggle");
      }
      motor(((millis() / 250) % 2 - 0.5) * 255 * 2, (((millis() + 250) / 250) % 2 - 0.5) * 255 * 2); // wiggle
      
      break;
    case 2: // turn LEDs in order to represent an increasing number changes speed depending on potentiometer
      if (lastMode != 2) { // mode change, print
        lastMode = 2;
        Serial.println("Straif");
      }
      Serial.print(sin(millis() / 200.0) * 255);
      Serial.println();
      motor(((millis() / 500) % 2 - 0.5) * 255 * 2, (((millis() + 250) / 500) % 2 - 0.5) * 255 * 2); // try  to drive sideways 
      break;
    case 3: // fade from green to blue depending on potentiometer
      if (lastMode != 3) { // mode change, print
        lastMode = 3;
        Serial.println("crappy obsticle avoid");
      }
      int left = analogRead(A1);
      int right = analogRead(A0);
      Serial.print(left);
      Serial.print(" ");
      Serial.print(right);
      Serial.println();
      motor(map(-left, -400, -300, -255, 255), map(-right, -400, -300, -255, 255));
      break;
  }
}

void motor(int left, int right) { // converts signals in range(-255, 255) to motor pon signals 
  if (left < 0) {
    digitalWrite(dirL, LOW);
    analogWrite(pwmL, abs(left));
  } else {
    digitalWrite(dirL, HIGH);
    analogWrite(pwmL, abs(left));
  }
  if (right < 0) {
    digitalWrite(dirR, HIGH);
    analogWrite(pwmR, abs(right));
  } else {
    digitalWrite(dirR, LOW);
    analogWrite(pwmR, abs(right));
  }
}
