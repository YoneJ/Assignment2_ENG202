#include <TimerOne.h>
// Motor control pins
const int enL = 7;   // PWM pin for left motor
const int inL1 = 8;  // Motor direction pin 1 for left motor
const int inL2 = 9;  // Motor direction pin 2 for left motor
const int enR = 13;  // PWM pin for right motor
const int inR1 = 10; // Motor direction pin 1 for right motor
const int inR2 = 11; // Motor direction pin 2 for right motor

// Encoder pins
const int enLA = 2;  // Encoder A pin for left motor
const int enLB = 3;  // Encoder B pin for left motor
const int enRA = 18; // Encoder A pin for right motor
const int enRB = 19; // Encoder B pin for right motor

// Control constants
const float d = 0.189;    // Distance between wheels (meters)
const float r = 0.0225; // Radius of the wheels (meters)
const float T = 0.05;      // Time step (seconds)
const float h = 0.21;     // Offset in angle calculation

// Encoder constantss
const int encoderCountsPerRevolution = 700; // 1:50 encoder with 7 pulses per revolution
const float wheelDiameter = 0.0225*2; // Diameter of the wheel in meters
//const float M_PI = 3.14159265358979323846;
const float wheelCircumference = wheelDiameter * M_PI ; // Wheel circumference

// Robot state variables
float x = 0.0;     // Current X position (meters)
float y = 0.0;     // Current Y position (meters)
float theta = 0.0; // Current orientation (radians)

// Encoder counts and speed control
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
const int K = 30;  // Adjustment factor for speed control
float dl;
float dr; 
unsigned long previousMillis = 0;
const long interval = 100;
bool isMoving = false; 
void setup() {
  Timer1.initialize(T*1000000);
  Timer1.attachInterrupt(calculateCoordinates);
  Serial.begin(115200);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop() {
if (Serial.available()> 0){
    char command = Serial.read();
    if (command == '2'){
      moveForward();
//      calculateCoordinates();
//      printCoordinates();
    }
   
    else if(command =='4'){
      turnLeft();
    }
    else if(command =='5'){
      turnRight();
    }
    else if(command =='0'){
      stop();
    }
  }
  delay(10);
}
  



void moveForward() {
    analogWrite(enR, 107);
    analogWrite(enL, 85);
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);

}

void turnLeft(){
   analogWrite(enR, 100);
   analogWrite(enL, 0);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  
}
void turnRight() {
  analogWrite(enR, 0);
  analogWrite(enL, 80);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}
// Encoder ISR for left wheel
void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

// Encoder ISR for right wheel
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

// Stop the motors
void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void calculateCoordinates(){

 dr = float(rightEnCount/float(encoderCountsPerRevolution+300)* wheelCircumference);
 dl = float(leftEnCount/float(encoderCountsPerRevolution)* wheelCircumference);
//   Serial.print("rightcount:");
//    Serial.println(rightEnCount);
//    Serial.print("leftcount:");
//    Serial.println(leftEnCount);
// 
    theta += (dr - dl) / d;
 
  x += (dl + dr) / 2.0 * cos(theta);
  y += (dl + dr) / 2.0 * sin(theta);

   
  
  rightEnCount = 0; 
  leftEnCount = 0; 

  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(theta);
//    Serial.print("dr:");
//    Serial.println(dr);
//    Serial.print("dl:");
//    Serial.println(dl);
   

}
