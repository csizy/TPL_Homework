// Created on: 2019.11.10.
// Author: Ádám Csizy
//
// Sketch for identifying the cannon (motor) controll mechanisms and the motor encoder.

#include "TimerOne.h"
#include <PinChangeInterruptBoards.h>
#include <YetAnotherPcInt.h>

// Signal - pin definitions
#define MOTOR2_3A 28
#define MOTOR2_4A 29
#define MOTOR2_EN 9
#define MOTOR2_ENCODER_A A10
#define MOTOR2_ENCODER_B A11

// Utility definitions
#define OPERATING_TIME 5300 // Operating time in milliseconds
#define TIMER_PERIOD 1000 // Timer period time in usec
#define MAX_SPEED 255 // PWM duty cycle
#define HALF_SPEED 127 // PWM duty cycle
#define NO_SPEED 0 // PWM duty cycle
#define ENCODER_STEP 0.003551 // Degree equivalent of one encoder trigger

////////////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ///////////////////////////////////////////////////////////////////////////////////////////

volatile long int encoderCounter = 0;
volatile boolean stopFlag = false;

///////////////////////////////////////////////////////////////////////////// UTILITY FUNCTION DEFINITIONS ////////////////////////////////////////////////////////////////////////////////////

// Set cannon rotation upwards
void setDirUp(void){

  digitalWrite(MOTOR2_3A, LOW);
  digitalWrite(MOTOR2_4A, HIGH);
}

// Set cannon rotation downwards
void setDirDown(void){

  digitalWrite(MOTOR2_3A, HIGH);
  digitalWrite(MOTOR2_4A, LOW);
}

// Rotate cannon with given PWM (PWM duty: 0 - 255)
void setRotationSpeed(byte rotationSpeed){

  analogWrite(MOTOR2_EN, rotationSpeed);
}

// Stop cannon rotation
void stopRotation(void){
  
  analogWrite(MOTOR2_EN,NO_SPEED);
}

// ENCODER 2A ISR
void ENCODER_2A_IRQHandler(void){

  if(digitalRead(MOTOR2_ENCODER_A) != digitalRead(MOTOR2_ENCODER_B))
    encoderCounter--;
  else
    encoderCounter++;
}

// ENCODER 2B ISR
void ENCODER_2B_IRQHandler(void){

  if(digitalRead(MOTOR2_ENCODER_A) == digitalRead(MOTOR2_ENCODER_B))
    encoderCounter--;
  else
    encoderCounter++;
}

//////////////////////////////////////////////////////////////////////////////////// SETUP FUNCTION ///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  // Setup serial communication
  Serial.begin(9600);

  // Initialize timer
  Timer1.initialize(TIMER_PERIOD);

  // Set timer ISR
  // Timer1.attachInterrupt(TIMER_IRQHandler, OPERATING_TIME);

  // Set pin modes
  pinMode(MOTOR2_3A, OUTPUT);
  pinMode(MOTOR2_4A, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_ENCODER_A, INPUT);
  pinMode(MOTOR2_ENCODER_B, INPUT);

  // Set hardware ISR
  PcInt::attachInterrupt(MOTOR2_ENCODER_A, ENCODER_2A_IRQHandler, CHANGE);
  PcInt::attachInterrupt(MOTOR2_ENCODER_B,  ENCODER_2B_IRQHandler, CHANGE);

}

////////////////////////////////////////////////////////////////////////////////////// LOOP FUNCTION //////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  /* TEST PROGRAM #2 (analogWrite()) */

  if(stopFlag){

     stopRotation();
     Serial.print("Encoder counter on stop:\t");
     Serial.print(encoderCounter);
     Serial.print("\tVertical angle on stop:\t");
     Serial.println(encoderCounter*ENCODER_STEP);
  }
  else{

    /*
     setDirUp();
     setRotationSpeed(63);
     delay(2000);
     stopRotation();
     delay(1000);
     setDirDown();
     setRotationSpeed(63);
     delay(2000);

    */
     setDirUp();
     setRotationSpeed(63);
     delay(1000);
     stopFlag = true;
  }
  
}
