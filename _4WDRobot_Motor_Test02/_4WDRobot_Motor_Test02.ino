/* 
Sketch for  to test out the operation of the Robot.
4WD Robot by Glenn Mossy
DC Robotics Group, Arduino Motors Workshop 
Nov 29, 2014, version 1.1
      
      
       *THE SUBROUTINES WORK AS FOLLOWS*

motorA(mode, speed)
% replace A with B to control motor B %

mode is a number 0 -> 3 that determines what the motor 
will do.
0 = coast/disable the H bridge
1 = turn motor clockwise
2 = turn motor counter clockwise
3 = brake motor

speed is a number 0 -> 100 that represents percentage of
motor speed.
0 = off
50 = 50% of full motor speed
100 = 100% of full motor speed

EXAMPLE
Say you need to have motor A turn clockwise at 33% of its
full speed.  The subroutine call would be the following...

motorA(1, 33);
*/

#include <Servo.h> //servo library
#include "pitches.h"// used for the speaker output
#define SPEAKER 10// Speaker Pin

Servo headservo;

//Hardware set pins
// L298D Motor Controller
#define ENA 5  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 3  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 7  //IN1 on pin  controls one side of bridge A
#define IN2 6  //IN2 on pin controls other side of A
#define IN3 4  //IN3 on pin conrtols one side of bridge B 
#define IN4 2  //IN4 on pin  controls other side of B

const int EchoPin = 11; // HC-SR04 Ultrasonic signal input
const int TrigPin = 12; // HC-SR04 Ultrasonic signal output

const int rightmotorpin1 = IN1;
const int rightmotorpin2 = IN2;
const int leftmotorpin1  = IN3; //signal output of Dc motor driven plate
const int leftmotorpin2  = IN4;

int motorSpeed = 0;

const int HeadServopin = 9; // signal input of headservo
//const int maxStart = 800; //run dec time
unsigned long time; // (time used instead of loops)
unsigned long time1; // (time used instead of loops)
int add= 0; //used for nodanger loop count
int add1= 0;  //used for nodanger loop count
int roam = 1;
int currDist = 5000; // distance
boolean running = false;// 

void setup() {
  /*
  //initialize beeps
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  //End Initialize Beeps
  */
  
Serial.begin(9600); // Enables Serial monitor for debugging purposes
Serial.println("Ready to receive Serial val Commands!"); // Tell us I"m ready
//pinMode(EchoPin, INPUT);//signal input port

//signal output port
  //set all of the outputs
  pinMode(IN1, OUTPUT);       // Motor Driver
  pinMode(IN2, OUTPUT);       // Motor Driver
  pinMode(IN3, OUTPUT);       // Motor Driver
  pinMode(IN4, OUTPUT);       // Motor Driver
  int motorSpeed = 15;        // Set initial motor speed
  
/*
// HC-SR04 interface
   pinMode(TrigPin, OUTPUT);  // Set the HC-SR04 pin
   pinMode(EchoPin, INPUT);  // Set the HC-SR04 pin
   
headservo.attach(HeadServopin);
//start movable head
headservo.write(160);
delay(1000);
headservo.write(20);
delay(1000);
headservo.write(90);
delay(1000);
return;
*/

}
void loop() 

  {
    if (Serial.available() > 0)
    {
    int val = Serial.read();	//read serial input commands
    switch(val)
    {
    case 'f' : 
      Serial.println("Rolling Forward!");
      moveForward(motorSpeed); 
      delay(5000);
      brake();
      delay(5000);
      break;
      
    case 'l' :
    
      Serial.println("Turning Left!");
      body_lturn(motorSpeed);
      delay(5000);
      brake();
      delay(5000);
      break;
      
    case 'r' : 
      
      Serial.println("Turning Right!");
      body_rturn(motorSpeed);
      delay(5000);
      brake();
      delay(5000);
            break;
          
   case 'b' :
          
      Serial.println("Turning Backward!");
      moveBackward(motorSpeed);
      delay(5000);
      brake();
      delay(5000);
            break;
    }      
    delay(20);  
  }
             
  if(roam == 0){ //just listen for serial commands and wait
      // Do something else if you like
      }
      else if(roam == 1){  //If roam active- drive autonomously
      }
  
  }
  
void moveForward(int motorSpeed)
{
   // int motorSpeed);  // change the 15 to the Speed variable, and put Speed int the function call command arguments.
    //also add delayTime for example like this:   moveForward(int delayTime, int motorSpeed)
    
    motorA(1, motorSpeed);  //have motor A turn clockwise at 15% speed
    motorB(1, motorSpeed);  //have motor A turn clockwise at 15% speed
    delay(10);
}
void body_lturn(int motorSpeed)
{
   motorA(1, motorSpeed);  //have motor B turn clockwise at 15% speed
   motorB(2, motorSpeed);  //have motor B turn counterclockwise at 15% speed
       delay(10);
}
void body_rturn(int motorSpeed)
{
   motorA(2, motorSpeed);  //have motor B turn counterclockwise at 15% speed
   motorB(1, motorSpeed);  //have motor B turn clockwise at 15% speed
   delay(10);
}
void moveBackward(int motorSpeed)
{
    int Speed;
    motorA(2, motorSpeed);  //have motor A turn counterclockwise at 15% speed
    motorB(2, motorSpeed);  //have motor A turn counterclockwisee at 15% speed
    delay(10);
}

void brake()
{
    motorA(3, 100);  //brake motor A with 100% braking power
    motorB(3, 100);  //brake motor A with 100% braking power
    delay(1);
}


//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      
      break;
  }
}
//**********************************************************


//******************   Motor B control   *******************
  void motorB(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENB, LOW);  //set enable low to disable B
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      
      break;
  }
}
//**********************************************************

void toggleRoam(){
if(roam == 0){
   roam = 1;
   Serial.println("Activated Roam Mode");
  }
  else{
    roam = 0;
    brake();
    Serial.println("De-activated Roam Mode");
  }
}
void buzz(){
tone(SPEAKER, NOTE_C7, 100);
delay(50);
tone(SPEAKER, NOTE_C6, 100);
}


//measure distance, unit “cm”
long MeasuringDistance() {
      // Calculates the Distance in cm
    float cm;                          //define variables for distance sensor 
    // ((time)*(Speed of sound))/ toward and backward of object) = Width of Echo pulse, in uS (micro second)
    // How to call the function:    long Distance_cm = Distance(Duration);   // Use function to calculate the distance  
long duration;
long adjust = 1.15;                        // Calibration adjustment based on actual measurement test
//pinMode(TrigPin, OUTPUT);
digitalWrite(TrigPin, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin, HIGH);
delayMicroseconds(5);
digitalWrite(TrigPin, LOW);
//pinMode(EchoPin, INPUT);
duration = pulseIn(EchoPin, HIGH);
return duration / 29 / 2 + adjust;        // Actual calculation in centimeters
}

