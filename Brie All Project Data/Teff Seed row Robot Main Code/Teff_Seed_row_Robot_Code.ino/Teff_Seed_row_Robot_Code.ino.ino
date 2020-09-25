//MSC PROJECT:DESIGN AND IMPLIMENTATION OF SOLAR POWERED TEFF SEED ROW PLANTING ROBOT//
//ELECTRONICS COMMUNICATION MANAGEMENT   
//*BY BIRHANU YAYEH 2013 E.C*
//************************************************************************************//
   
#include <SoftwareSerial.h>
#include <Servo.h>
//L293 Connection
const int motorA1      = 3;
const int motorA2      = 4;
const int motorAspeed  = 5;
const int motorB1      = 7;
const int motorB2      = 8;
const int motorBspeed  = 6;

//Useful Variables
int state;
int vSpeed = 200; // Default speed, from 0 to 255
int greenled = A4;// Connect greenled to analog A4 of arduino board
int redled = A3; //Connect redled to analog A3 of arduino board
int blueled = A5; //Connect Blueled to analog A5 of arduino board
int Yellowled = 13;//Connect Yellowled to analog pin 13 of arduino board
int buzzer = A2; //Connect Yellowled to analog pin 13 of arduino board
int angle = 0; // defining servo intial agle to 0
int trig = 11; // connect trigger pin of Ultrasonic to pin number 11 
int echo = 12; // connect echo pin of Ultrasonic to pin number 12 
Servo myServo;

void setup() {
  // Set pins as outputs:
  pinMode(motorA1, OUTPUT);// Definning motorA1 as an output
  pinMode(motorA2, OUTPUT);//Definning motorA2 as an output
  pinMode(motorB1, OUTPUT);//Definning motorB1 as an output
  pinMode(motorB2, OUTPUT);//Definning motorB2 as an output
  pinMode(buzzer, OUTPUT);//Definning buzzer as an output
  myServo.attach(9);//Definning servo to pin 9 
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(greenled, OUTPUT);//Definning greenled as an output
  pinMode(redled, OUTPUT); //Definning redled as an output
  pinMode(blueled, OUTPUT);//Definning blueled as an output
  pinMode(Yellowled, OUTPUT);//Definning yellowled as an output
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {

  if (Serial.available() > 0) {
    state = Serial.read();

  }

  Serial.println(state);
  //Change speed if state is equal from 0 to 4. Values must be from 0 to 255 (PWM)
  if (state == '0') {
    vSpeed = 0;
  }
  else if (state == '1') {
    vSpeed = 100;
  }
  else if (state == '2') {
    vSpeed = 180;
  }
  else if (state == '3') {
    vSpeed = 200;
  }
  else if (state == '4') {
    vSpeed = 255;
  }

  /***********************Forward****************************/
  //If state is equal with letter 'F', car will go forward!
  if (state == 'F') {
    digitalWrite (motorA1, LOW);
    delay(1);
    digitalWrite(motorA2, HIGH);
    delay(1);
    digitalWrite (motorB1, LOW);
    delay(1);
    digitalWrite(motorB2, HIGH);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    digitalWrite(Yellowled, HIGH);
    delay(1);
    digitalWrite(Yellowled, LOW);
    delay(1);
    myServo.write(0);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
  }
  /**********************Forward Left************************/
  //If state is equal with letter 'I', car will go forward left
  else if (state == 'I') {
    digitalWrite (motorA1, LOW);
    delay(1);
    digitalWrite(motorA2, HIGH);
    delay(1);
    digitalWrite (motorB1, LOW);
    delay(1);
    digitalWrite(motorB2, HIGH);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, vSpeed);
  }
  /**********************Forward Right************************/
  //If state is equal with letter 'G', car will go forward right
  else if (state == 'G') {
    digitalWrite (motorA1, LOW);
    delay(1);
    digitalWrite(motorA2, HIGH);
    delay(1);
    digitalWrite (motorB1, LOW);
    delay(1);
    digitalWrite(motorB2, HIGH);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, 0);
  }
  /***********************Backward****************************/
  //If state is equal with letter 'B', car will go backward
  else if (state == 'B') {
    digitalWrite (motorA1, HIGH);
    delay(1);
    digitalWrite(motorA2, LOW);
    delay(1);
    digitalWrite (motorB1, HIGH);
    delay(1);
    digitalWrite(motorB2, LOW);
    digitalWrite(redled , HIGH);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, HIGH);
    myServo.write(90);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
  }
  /**********************Backward Left************************/
  //If state is equal with letter 'J', car will go backward left
  else if (state == 'J') {
    digitalWrite (motorA1, HIGH);
    delay(1);
    digitalWrite(motorA2, LOW);
    delay(1);
    digitalWrite (motorB1, HIGH);
    delay(1);
    digitalWrite(motorB2, LOW);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, 0);
    analogWrite (motorBspeed, vSpeed);
  }
  /**********************Backward Right************************/
  //If state is equal with letter 'H', car will go backward right
  else if (state == 'H') {
    digitalWrite (motorA1, HIGH);
    delay(1);
    digitalWrite(motorA2, LOW);
    delay(1);
    digitalWrite (motorB1, HIGH);
    delay(1);
    digitalWrite(motorB2, LOW);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, 0);
  }
  /***************************Left*****************************/
  //If state is equal with letter 'L', wheels will turn left
  else if (state == 'L') {
    digitalWrite (motorA2, HIGH);
    delay(1);
    digitalWrite(motorA1, LOW);
    delay(1);
    digitalWrite (motorB2, LOW);
    delay(1);
    digitalWrite(motorB1, HIGH);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , HIGH);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
  }
  /***************************Right*****************************/
  //If state is equal with letter 'R', wheels will turn right
  else if (state == 'R') {
    digitalWrite (motorA2, LOW);
    delay(1);
    digitalWrite(motorA1, HIGH);
    delay(1);
    digitalWrite (motorB2, HIGH);
    delay(1);
    digitalWrite(motorB1, LOW);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, HIGH);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);
  }

  /************************Stop*****************************/
  //If state is equal with letter 'S', stop the car
  else if (state == 'S') {
    analogWrite(motorA1, 0);
    analogWrite(motorA2, 0);
    analogWrite(motorB1, 0);
    analogWrite(motorB2, 0);
    /************************Stop*****************************/
    //If state is equal with letter 'O', make the car to auto mode
  }
  else if (state == 'O')
  {
    long du = 0, ds = 0; // Transmitting pulse
    digitalWrite(trig, LOW);//set trig pin to high
    delayMicroseconds(10);//wait 2MS
    digitalWrite(trig, HIGH);//set trig pin to high to send pulse
    delayMicroseconds(10);//wait 10MS
    digitalWrite(trig, LOW);// Waiting for pulse
    du = pulseIn(echo, HIGH);// check for the pulse
    ds = du * 0.034 / 2 ;// Calculating distance
    
    //move the car forward
    
    digitalWrite (motorA1, LOW);
    delay(1);
    digitalWrite(motorA2, HIGH);
    delay(1);
    digitalWrite (motorB1, LOW);
    delay(1);
    digitalWrite(motorB2, HIGH);
    digitalWrite(redled , LOW);
    digitalWrite(greenled , LOW);
    digitalWrite(blueled, LOW);
    digitalWrite(buzzer, LOW);
    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);

    if (ds <= 10) //if the space betwen the robot and the object less than 10 or 10 then turn right until the robot gets a space more than 10
    {
      analogWrite (motorAspeed, vSpeed);
      analogWrite (motorBspeed, vSpeed);
      digitalWrite (motorA2, LOW);
      delay(1);
      digitalWrite(motorA1, HIGH);
      delay(1);
      digitalWrite (motorB2, HIGH);
      delay(1);
      digitalWrite(motorB1, LOW);
      digitalWrite(redled , LOW);
      digitalWrite(greenled , LOW);
      digitalWrite(blueled, HIGH);
      digitalWrite(buzzer, LOW);
    }
    state == 'O';//here we will let the robot do the same job untill the we press stop bottom from mobile or press any bottom so we control it manualy
  }
}
