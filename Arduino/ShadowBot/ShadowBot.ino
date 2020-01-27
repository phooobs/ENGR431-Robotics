const int switch0 = 10; // pin connected to switch 0
const int switch1 = 9; // pin connected to switch 1

const int potPin = A0; // pin connected to potentiometer

const int dirL = 2; // left motor direction pin
const int dirR = 4; // right motor direction pin
const int pwmL = 11; // left motor speed pin
const int pwmR = 3; // right motor speed pin

const int led = 12;

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
  pinMode(led, OUTPUT);
  
  Serial.begin(9600); // open serial port
  while(!Serial) {} // wait till serial port has connected
}

void loop() {
  static int lastMode = -1; // keep track of what mode we are on so when there is a change it can be detected, -1 means no last mode
  switch(2 * digitalRead(switch1) + digitalRead(switch0)){ // get input from switches and convert to mode number
    case 0: // 00 Sit, wait and blink, repeat.
      if (lastMode != 0) { // mode change, print
        lastMode = 0;
        Serial.println("off");
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
        Serial.println("Straight and speed");
        delay(3000);
      }
      motor(0, 0);
      break;
    case 2: // 10 Pivot and turn test
      if (lastMode != 2) { // mode change, print
        lastMode = 2;
        Serial.println("Pivot and turn test");
        delay(3000);
      }
      motor(0, 0);
      break;
    case 3: // 11 Follow the wall test
      if (lastMode != 3) { // 11 Follow the wall test
        lastMode = 3;
        Serial.println("Follow the wall test");
        delay(3000);
      }
      motor(0, 0);
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
