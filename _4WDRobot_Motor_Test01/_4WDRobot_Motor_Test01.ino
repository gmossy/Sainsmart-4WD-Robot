/* 
4WDRobot_Motor_Test
Sketch to test out the operation of the Sainsmart 4WD Robot.
4WD Robot by Glenn Mossy
https://github.com/gmossy/Sainsmart-4WD-Robot
gmossy@gmail.com

DC Robotics Group, Arduino Motors Workshop, Dec 6, 2014
Dec 1, 2014, version 1.0      
      
       *THE SUBROUTINES WORK AS FOLLOWS*

motorA(mode, speed)  // % replace A with B to control motor B %

mode is a number 0 -> 3 that determines what the motor will do.
0 = coast/disable the H bridge
1 = turn motor clockwise
2 = turn motor counter clockwise
3 = brake motor

speed is a number 0 -> 100 that represents percentage of motor speed.
0 = off
50 = 50% of full motor speed
100 = 100% of full motor speed

EXAMPLE
Say you need to have motor A turn clockwise at 33% of its full speed.  The subroutine call would be the following...
motorA(1, 33);
*/

#include <Servo.h>    //servo library
#include "pitches.h"  // used for the speaker output
#define SPEAKER 10    // define a Speaker Pin

Servo headservo;      // initialize a servo object

//Hardware set the pins
// L298D Motor Controller
#define ENA 5  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 3  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 7  //IN1 on pin controls Motor A  Motor A is the left side Motor
#define IN2 6  //IN2 on pin controls Motor A
#define IN3 4  //IN3 on pin conrtols Motor B  Motor B is the Right side Motor
#define IN4 2  //IN4 on pin controls Motor B 

const int rightmotorpin1 = IN1;  //signal output 1 of Dc motor 
const int rightmotorpin2 = IN2;  //signal output 2 of Dc motor 
const int leftmotorpin1  = IN3;  //signal output 3 of Dc motor 
const int leftmotorpin2  = IN4;  //signal output 4 of Dc motor 

int motorSpeed;          //define motor speed parameter which will be mapped as a percentage value

// pins for the HC-SR04 Ultrasonic Distance Sensor
const int EchoPin = 11; // HC-SR04 Ultrasonic signal input
const int TrigPin = 12; // HC-SR04 Ultrasonic signal output
const int HeadServopin = 9; // pin for signal input of headservo

/*
//const int maxStart = 800; //run dec time
unsigned long time;         // (time used instead of loops)
unsigned long time1;        // (time used instead of loops)
int add= 0;                 // used for nodanger loop count in roam mode
int add1= 0;                // used for nodanger loop count in roam mode
int currDist = 5000; // distance
boolean running = false;// 
*/
  int roam = 0;             //just listen for serial commands and wait


void setup() { 
Serial.begin(9600); // Enables Serial monitor for debugging purposes
Serial.println("Ready to receive Serial Commands![f, b, r, l, s]"); // Tell us I"m ready

  motorSpeed= 30;            //set an initial motor speed as a percentage value

 //signal output port
 //set all of the outputs for the motor driver
  pinMode(IN1, OUTPUT);       // Motor Driver
  pinMode(IN2, OUTPUT);       // Motor Driver
  pinMode(IN3, OUTPUT);       // Motor Driver
  pinMode(IN4, OUTPUT);       // Motor Driver
  int motorSpeed = 15;        // Set motorSpeed variable with an initial motor speed % (percentage)
  
  intialize_beeps();          // call the intialize_beeps method and play some tones 

/* uncomment the HC-SR04 code when you are ready to use it.
// HC-SR04 interface
   pinMode(TrigPin, OUTPUT);  // Set the HC-SR04 Trig as Output
   pinMode(EchoPin, INPUT);   // Set the HC-SR04 Echo as Input
   
headservo.attach(HeadServopin);
//start my movable servo head, to verify I work.
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
     buzz();
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
   case 's' :      
      Serial.println("Stop!");
      brake();
      delay(5000);
            break;
    }      
    delay(1);  
    Serial.println("Ready to receive Serial Commands![f, b, r, l, s]"); // Tell us I"m ready
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
    
    motorA(1, motorSpeed);  //have motor A turn clockwise at % speed, , call motor control method
    motorB(1, motorSpeed);  //have motor B turn clockwise at % speed
    Serial.println("Forward");
}
void body_rturn(int motorSpeed)
{
   motorA(1, motorSpeed);  //have motor A turn counterclockwise 
   motorB(2, motorSpeed);  //have motor B turn clockwise 
   Serial.println("Right");
}
   void body_lturn(int motorSpeed)
{
   motorA(2, motorSpeed);  //have motor A turn clockwise
   motorB(1, motorSpeed);  //have motor B turn counterclockwise 
   Serial.println("Left");
}
void moveBackward(int motorSpeed)
{
    motorA(2, motorSpeed);  //have motor A turn counterclockwise 
    motorB(2, motorSpeed);  //have motor B turn counterclockwise
    Serial.println("Backward");
}

void brake()
{
    motorA(3, 100);  //brake motor A with 100% braking power, call motor control method
    motorB(3, 100);  //brake motor B with 100% braking power, call motor control method
    Serial.println("Brake");
}


//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
 // Method that will accept mode and speed in percentage.  
 // mode: The 3 modes are 0) coast  1) Clockwise motor  2) Counter-clockwise 3) brake
 // percent: The speed of the motor in percentage value for the PWM.
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      Serial.println("Disable/coast A");
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      Serial.println("turn motor A clockwise");
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      Serial.println("turn motor A counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      Serial.println("brake motor A");
      
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
      Serial.println("Disable/coast B");
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty); 
           Serial.println("turn motor B clockwise"); 
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
       Serial.println("turn motor B counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      Serial.println("brake motor B");      
      break;
  }
}
//**********************************************************

void toggleRoam(){
  // This method chooses to make the robot roam or else use the serial command input.
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


//measure distance, unit “cm”
long MeasuringDistance() {
  // This method reads the HC-SR04 Ultrasonic Sensor and returns the distance measured of an object in front the sensor
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

void buzz(){
 // method to create a buzz sound
tone(SPEAKER, NOTE_C7, 100);
delay(50);
tone(SPEAKER, NOTE_C6, 100);
delay(50);
}
void intialize_beeps()
{
   //initializing the robot beeps sound
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  tone(SPEAKER, NOTE_C7, 100);
  delay(500);
  tone(SPEAKER, NOTE_C6, 100);
  delay(500);
  //End Initialize Beeps
}
