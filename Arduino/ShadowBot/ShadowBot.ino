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
        Serial.println("[01] Straight and speed");
        motor(0, 0);
        delay(3000);
        straightSpeedTest();
      }
      motor(0, 0);
      break;
    case 2: // 10 Pivot and turn test
      if (lastMode != 2) { // mode change, print
        lastMode = 2;
        motor(0, 0);
        Serial.println("[10] Pivot and turn test");
        delay(3000);
        spinBoi();
      }
      motor(0, 0);
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
