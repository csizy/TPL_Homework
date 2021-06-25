// Created on: 2019.11.26.
// Author: Ádám Csizy
//
// Sketch for testing and implementing serial communication and control mechanism.

#include "TimerOne.h"
#include <PinChangeInterruptBoards.h>
#include <YetAnotherPcInt.h>

// Signal - pin definitions
#define MOTOR2_3A 28
#define MOTOR2_4A 29
#define MOTOR2_EN 9
#define MOTOR2_ENCODER_A A10
#define MOTOR2_ENCODER_B A11
#define IR_SENSOR_VALUE A1
#define LED_RED 6
#define LED_GREEN 7
#define LED_BLUE 8

// Utility definitions
#define MODE_AUTO 0 // Automatic fire control mode
#define MODE_MANUAL 1 // Manual fire control mode
#define MODE_LOCK 2 // Locked fire control mode

#define MAX_SPEED 255 // PWM duty cycle
#define NO_SPEED 0 // PWM duty cycle

#define SAMPLE_PERIOD 20 // Sample period in milliseconds
#define Kp 4.4 // PI controller coefficient
#define Ki 4.5 // PI controller coefficient
#define U_OFFSET 0.0 // PI controller offset

#define ENCODER_STEP 0.003551 // Degree equivalent of one encoder trigger
#define ADC_STEP 0.003225806452 // Voltage equivalent of the ADC value (Vcc/ADC range = 3.3V/1023)

#define INVALID_DOMAIN_INDEX 255 // Invalid characteristic array index

#define FIRING_RANGE 90.2 // Cannon firing range (depends on bullet type)

// Characteristics steepness [V/cm]
#define STEEP0 -0.132
#define STEEP1 -0.068
#define STEEP2 -0.044
#define STEEP3 -0.028
#define STEEP4 -0.02
#define STEEP5 -0.016
#define STEEP6 -0.016
#define STEEP7 -0.012
#define STEEP8 -0.01
#define STEEP9 -0.006
#define STEEP10 -0.01
#define STEEP11 -0.006
#define STEEP12 -0.004
#define STEEP13 -0.008
#define STEEP14 -0.004
#define STEEP15 -0.006
#define STEEP16 -0.002
#define STEEP17 -0.004
#define STEEP18 -0.004
#define STEEP19 -0.002
#define STEEP20 -0.002
#define STEEP21 -0.002
#define STEEP22 -0.002
#define STEEP23 -0.002
#define STEEP24 0.0 

// Characteristics offset [V]
#define OFFSET0 3.52
#define OFFSET1 2.56
#define OFFSET2 2.08
#define OFFSET3 1.68
#define OFFSET4 1.44
#define OFFSET5 1.3
#define OFFSET6 1.3
#define OFFSET7 1.12
#define OFFSET8 1.02
#define OFFSET9 0.8
#define OFFSET10 1.04
#define OFFSET11 0.78
#define OFFSET12 0.64
#define OFFSET13 0.94
#define OFFSET14 0.62
#define OFFSET15 0.79
#define OFFSET16 0.43
#define OFFSET17 0.62
#define OFFSET18 0.62
#define OFFSET19 0.41
#define OFFSET20 0.41
#define OFFSET21 0.41
#define OFFSET22 0.41
#define OFFSET23 0.41
#define OFFSET24 0.15

// Characteristics limit [V]
#define LIMIT0 2.2
#define LIMIT1 1.54
#define LIMIT2 1.2
#define LIMIT3 0.98
#define LIMIT4 0.84
#define LIMIT5 0.74
#define LIMIT6 0.66
#define LIMIT7 0.58
#define LIMIT8 0.52
#define LIMIT9 0.47
#define LIMIT10 0.44
#define LIMIT11 0.39
#define LIMIT12 0.36
#define LIMIT13 0.34
#define LIMIT14 0.3
#define LIMIT15 0.28
#define LIMIT16 0.25
#define LIMIT17 0.24
#define LIMIT18 0.22
#define LIMIT19 0.2
#define LIMIT20 0.19
#define LIMIT21 0.18
#define LIMIT22 0.17
#define LIMIT23 0.16
#define LIMIT24 0.15
#define LIMIT25 0.15

////////////////////////////////////////////////////////////////////////////////////TYPE DEFINITIONS///////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
  
  double steepness;
  double offset;
} linearChar;

/////////////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES //////////////////////////////////////////////////////////////////////////////////////////

byte mode;
double actualAngle = 0.0, referenceAngle = 0.0;
int inputData;
double integrator = 0.0;
volatile int encoderCounter = 0;
linearChar charArray[25] = {{STEEP0,OFFSET0},{STEEP1,OFFSET1},{STEEP2,OFFSET2},{STEEP3,OFFSET3},{STEEP4,OFFSET4},{STEEP5,OFFSET5},{STEEP6,OFFSET6},{STEEP7,OFFSET7},{STEEP8,OFFSET8},
  {STEEP9,OFFSET9},{STEEP10,OFFSET10},{STEEP11,OFFSET11},{STEEP12,OFFSET12},{STEEP13,OFFSET13},{STEEP14,OFFSET14},{STEEP15,OFFSET15},{STEEP16,OFFSET16},{STEEP17,OFFSET17},
  {STEEP18,OFFSET18},{STEEP19,OFFSET19},{STEEP20,OFFSET20},{STEEP21,OFFSET21},{STEEP22,OFFSET22},{STEEP23,OFFSET23},{STEEP24,OFFSET24}};

///////////////////////////////////////////////////////////////////////////// UTILITY FUNCTION DEFINITIONS ////////////////////////////////////////////////////////////////////////////////////

// Update actual cannon angle and reset encoder counter
void updateActualAngle(void){

  actualAngle += (double)encoderCounter*ENCODER_STEP;
  encoderCounter = 0;
}

// Update reference cannon angle
void updateReferenceAngle(double distance){

  double g = 9.8; // Gravitation acceleration
  double v0 = 2.9732; // Initial velocity
 
  referenceAngle = ( ((180.0/PI) * asin( ((distance/100.0)*g)/(v0*v0))) / 2.0 );
}

// PI controller
int PIController(double error){

  int u;

  u = (int)(Kp*error + Ki*(integrator + (SAMPLE_PERIOD/1000.0)*error));

  if(u < 0)
    u = u - U_OFFSET;
  else
    if(u > 0)
      u = u + U_OFFSET;
  
  if(MAX_SPEED >= abs(u)){

    integrator += (SAMPLE_PERIOD/1000.0)*error;
  }
  else{ // Control sign saturation
   
      if(u < 0)
        u = -MAX_SPEED;
      else
        u = MAX_SPEED;
  }

  return u;
}

// Get target distance
double getDistance(void){

  byte domainIndex;
  double distance;
  double IRSensorVoltage = analogRead(IR_SENSOR_VALUE)*ADC_STEP;

  // Select appropriate characteristic by IR sensor voltage
  if(IRSensorVoltage > LIMIT0)
    domainIndex = INVALID_DOMAIN_INDEX;
  else if(IRSensorVoltage <= LIMIT0 && IRSensorVoltage > LIMIT1)
    domainIndex = 0;
  else if(IRSensorVoltage <= LIMIT1 && IRSensorVoltage > LIMIT2)
    domainIndex = 1;
  else if(IRSensorVoltage <= LIMIT2 && IRSensorVoltage > LIMIT3)
    domainIndex = 2;
  else if(IRSensorVoltage <= LIMIT3 && IRSensorVoltage > LIMIT4)
    domainIndex = 3;
  else if(IRSensorVoltage <= LIMIT4 && IRSensorVoltage > LIMIT5)
    domainIndex = 4;
  else if(IRSensorVoltage <= LIMIT5 && IRSensorVoltage > LIMIT6)
    domainIndex = 5;
  else if(IRSensorVoltage <= LIMIT6 && IRSensorVoltage > LIMIT7)
    domainIndex = 6;
  else if(IRSensorVoltage <= LIMIT7 && IRSensorVoltage > LIMIT8)
    domainIndex = 7;
  else if(IRSensorVoltage <= LIMIT8 && IRSensorVoltage > LIMIT9)
    domainIndex = 8;
  else if(IRSensorVoltage <= LIMIT9 && IRSensorVoltage > LIMIT10)
    domainIndex = 9;
  else if(IRSensorVoltage <= LIMIT10 && IRSensorVoltage > LIMIT11)
    domainIndex = 10;
  else if(IRSensorVoltage <= LIMIT11 && IRSensorVoltage > LIMIT12)
    domainIndex = 11;
  else if(IRSensorVoltage <= LIMIT12 && IRSensorVoltage > LIMIT13)
    domainIndex = 12;
  else if(IRSensorVoltage <= LIMIT13 && IRSensorVoltage > LIMIT14)
    domainIndex = 13;
  else if(IRSensorVoltage <= LIMIT14 && IRSensorVoltage > LIMIT15)
    domainIndex = 14;
  else if(IRSensorVoltage <= LIMIT15 && IRSensorVoltage > LIMIT16)
    domainIndex = 15;
  else if(IRSensorVoltage <= LIMIT16 && IRSensorVoltage > LIMIT17)
    domainIndex = 16;
  else if(IRSensorVoltage <= LIMIT17 && IRSensorVoltage > LIMIT18)
    domainIndex = 17;
  else if(IRSensorVoltage <= LIMIT18 && IRSensorVoltage > LIMIT19)
    domainIndex = 18;
  else if(IRSensorVoltage <= LIMIT19 && IRSensorVoltage > LIMIT20)
    domainIndex = 19;
  else if(IRSensorVoltage <= LIMIT20 && IRSensorVoltage > LIMIT21)
    domainIndex = 20;
  else if(IRSensorVoltage <= LIMIT21 && IRSensorVoltage > LIMIT22)
    domainIndex = 21;
  else if(IRSensorVoltage <= LIMIT22 && IRSensorVoltage > LIMIT23)
    domainIndex = 22;
  else if(IRSensorVoltage <= LIMIT23 && IRSensorVoltage > LIMIT24)
    domainIndex = 23;
  else if(IRSensorVoltage <= LIMIT24 && IRSensorVoltage > LIMIT25)
    domainIndex = 24;
  else
    domainIndex = INVALID_DOMAIN_INDEX;

// Check if target is inside characteristic range
 if(INVALID_DOMAIN_INDEX != domainIndex){

  distance = (IRSensorVoltage-charArray[domainIndex].offset)/charArray[domainIndex].steepness;
 }
 else{

  distance = -1.0;
 }

 return distance /*+5.0*/; // Optional distance correction
}

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

void log(int u, double distance){

  Serial.print("Distance:  "); Serial.print(distance);
  Serial.print(" cm    Ref. Angle:  "); Serial.print(referenceAngle);
  Serial.print("   Actual Angle:  "); Serial.print(actualAngle);
  Serial.print("   U:  "); Serial.print(u);
  Serial.print("   Mode:  ");

  switch(mode){
    case MODE_MANUAL: Serial.println("MANUAL"); break;
    case MODE_AUTO: Serial.println("AUTO"); break;
    case MODE_LOCK: Serial.println("LOCK"); break;
  }
}
//////////////////////////////////////////////////////////////////////////////////// SETUP FUNCTION ///////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  // Setup serial communication
  Serial.begin(9600);

  // Set pin modes
  pinMode(MOTOR2_3A, OUTPUT);
  pinMode(MOTOR2_4A, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);
  pinMode(MOTOR2_ENCODER_A, INPUT);
  pinMode(MOTOR2_ENCODER_B, INPUT);
  pinMode(IR_SENSOR_VALUE, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Set analog reference
  analogReference(DEFAULT); // 3.3 V

  // Attach hardware ISRs
  PcInt::attachInterrupt(MOTOR2_ENCODER_A, ENCODER_2A_IRQHandler, CHANGE);
  PcInt::attachInterrupt(MOTOR2_ENCODER_B,  ENCODER_2B_IRQHandler, CHANGE);

  // Wait for input data
  while(1 > Serial.available()){}

  inputData = Serial.read();

  switch(inputData){

    case 'a':
      mode = MODE_AUTO;
      break;
      
    case 'm':
      stopRotation();
      while(1 > Serial.available()){}
      referenceAngle = 0;
      referenceAngle = (Serial.read() - '0')*10;
      while(1 > Serial.available()){} 
      referenceAngle += (Serial.read()-'0');
      mode = MODE_MANUAL;
      break;
      
    case 'l':
      mode = MODE_LOCK;
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////// LOOP FUNCTION ////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  int u = 0; // Control sign
  double distance; // Target distance

  if(0 < Serial.available()){

    inputData = Serial.read();
    
    switch(inputData){

    case 'a':
      mode = MODE_AUTO;
      break;
      
    case 'm':
      stopRotation();
      while(1 > Serial.available()){}
      referenceAngle = 0;
      referenceAngle = (Serial.read() - '0')*10;
      while(1 > Serial.available()){} 
      referenceAngle += (Serial.read()-'0');
      mode = MODE_MANUAL;
      break;
      
    case 'l':
      mode = MODE_LOCK;
      break;
  }
 }
  
  switch(mode){

    case MODE_AUTO:
    
      distance = getDistance(); // Get target distance
      
      if(-1.0 != distance && FIRING_RANGE >= distance){ // Check if target is inside firing range

        digitalWrite(LED_RED,LOW); // Turn off all LED
        digitalWrite(LED_GREEN,LOW);
        digitalWrite(LED_BLUE,LOW);
        
        updateReferenceAngle(distance); // Update reference angle
        updateActualAngle(); // Update actual cannon angle
        u = PIController(referenceAngle - actualAngle); // Calculate control sign
        
        if(0 > u){

          setDirDown(); // Set rotation direction down if control sign is negative
          setRotationSpeed(abs(u));
        }
        else{

          setDirUp(); // Set rotation direction up if control sign is positive
          setRotationSpeed(u);        
        }

        // TODO: Send control sign value to Labview HMI via serial communication.
      }
      else{

        digitalWrite(LED_RED,HIGH); // Turn on red LED for this mode
        digitalWrite(LED_GREEN,LOW);
        digitalWrite(LED_BLUE,LOW);
        stopRotation(); // Stop cannon rotation
        updateActualAngle(); // Update actual cannon angle
        
        // TODO: Send control sign value (u = 0) to Labview HMI via serial communication.
      }
      
      break;

    case MODE_MANUAL:

      digitalWrite(LED_RED,LOW); // Turn off all LED
      digitalWrite(LED_GREEN,LOW);
      digitalWrite(LED_BLUE,LOW);
      updateActualAngle(); // Update actual cannon angle
      u = PIController(referenceAngle - actualAngle); // Calculate control sign

      if(0 > u){

        setDirDown(); // Set rotation direction down if control sign is negative
        setRotationSpeed(abs(u));
      }
      else{

        setDirUp(); // Set rotation direction up if control sign is positive
        setRotationSpeed(u);        
      }
      
      break;

    case MODE_LOCK:

      digitalWrite(LED_RED,LOW); // Turn on green LED for this mode
      digitalWrite(LED_GREEN,HIGH);
      digitalWrite(LED_BLUE,LOW);
      stopRotation(); // Stop cannon rotation
      updateActualAngle(); // Update actual cannon angle
      distance = getDistance();
      // TODO: Send control sign value (u = 0) to Labview HMI via serial communication.
      break;
    
  }

  log(u, distance);
  delay(SAMPLE_PERIOD); // Alternative: use Timer1 interrupt for whole loop body. Encoder interrupts must be protected( interrupts(), noInterrupts()).

}
