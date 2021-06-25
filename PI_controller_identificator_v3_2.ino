// Created on: 2019.12.10.
// Author: Ádám Csizy
//
// Sketch implementing serial communication and control mechanism.

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
#define OFFSET_SPEED 50 // PWM duty offset

#define SAMPLE_PERIOD 20 // Sample period in milliseconds
#define Kp 3.5 // PI controller coefficient
#define Ki 4.5 // PI controller coefficient

#define ENCODER_STEP 0.003551 // Degree equivalent of one encoder trigger
#define ADC_STEP 0.003225806452 // Voltage equivalent of the ADC value (Vcc/ADC range = 3.3V/1023)

#define INVALID_DOMAIN_INDEX 255 // Invalid characteristic array index
#define AVG_SAMPLE_NUMBER 5 // Averaging sample number

#define FIRING_RANGE 90.2 // Cannon firing range (depends on bullet type)

#define MAX_REF_ANGLE 90.0 // Upper limit of reference angle
#define MIN_REF_ANGLE 0.0 // Lower limit of reference angle
#define SAFETY_ANGLE 40.0 // Safety reference angle (in case of failure)

// Characteristics steepness [V/cm]
#define STEEP0 -0.122
#define STEEP1 -0.066
#define STEEP2 -0.034
#define STEEP3 -0.028
#define STEEP4 -0.018
#define STEEP5 -0.016
#define STEEP6 -0.014
#define STEEP7 -0.012
#define STEEP8 -0.014
#define STEEP9 -0.008
#define STEEP10 -0.012
#define STEEP11 -0.006
#define STEEP12 -0.004
#define STEEP13 -0.004
#define STEEP14 -0.004
#define STEEP15 -0.002
#define STEEP16 0.0
#define STEEP17 -0.002
#define STEEP18 0.0
#define STEEP19 -0.002
#define STEEP20 -0.002
#define STEEP21 -0.002
#define STEEP22 -0.002
#define STEEP23 -0.002
#define STEEP24 -0.002 

// Characteristics offset [V]
#define OFFSET0 3.29
#define OFFSET1 2.45
#define OFFSET2 1.81
#define OFFSET3 1.66
#define OFFSET4 1.36
#define OFFSET5 1.29
#define OFFSET6 1.21
#define OFFSET7 1.12
#define OFFSET8 1.22
#define OFFSET9 0.89
#define OFFSET10 1.13
#define OFFSET11 0.74
#define OFFSET12 0.6
#define OFFSET13 0.6
#define OFFSET14 0.6
#define OFFSET15 0.43
#define OFFSET16 0.25
#define OFFSET17 0.44
#define OFFSET18 0.24
#define OFFSET19 0.45
#define OFFSET20 0.45
#define OFFSET21 0.45
#define OFFSET22 0.45
#define OFFSET23 0.45
#define OFFSET24 0.45

// Characteristics limit [V]
#define LIMIT0 2.07
#define LIMIT1 1.46
#define LIMIT2 1.13
#define LIMIT3 0.96
#define LIMIT4 0.82
#define LIMIT5 0.73
#define LIMIT6 0.65
#define LIMIT7 0.58
#define LIMIT8 0.52
#define LIMIT9 0.45
#define LIMIT10 0.41
#define LIMIT11 0.35
#define LIMIT12 0.32
#define LIMIT13 0.3
#define LIMIT14 0.28
#define LIMIT15 0.26
#define LIMIT16 0.25
#define LIMIT17 0.25
#define LIMIT18 0.24
#define LIMIT19 0.24
#define LIMIT20 0.23
#define LIMIT21 0.22
#define LIMIT22 0.21
#define LIMIT23 0.2
#define LIMIT24 0.19
#define LIMIT25 0.18

////////////////////////////////////////////////////////////////////////////////////TYPE DEFINITIONS///////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
  
  double steepness;
  double offset;
} linearChar;

/////////////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES //////////////////////////////////////////////////////////////////////////////////////////

byte mode = MODE_LOCK, msb;
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

  // Using inclined throw formula
  referenceAngle = ( ((180.0/PI) * asin( ((distance/100.0)*g)/(v0*v0))) / 2.0 );
}

// PI controller
int PIController(double error){

  int u, u_resc;

  // Calculate control sign
  // Formula: u[k] = Kp*error[k] + Ki*integrator[k] = Kp*error[k] + Ki*(integrator[k-1] + Ts*error[k]) where Ts is sample period and Ki,Kp are constants.
  u = (int)(Kp*error + Ki*(integrator + (SAMPLE_PERIOD/1000.0)*error));

  // Control sign saturation
  if(MAX_SPEED >= abs(u)){

    // Update actual integrator value
    integrator += (SAMPLE_PERIOD/1000.0) * error;

    // Rescaling speed with offset value
    // Formula: U_new = U_new_min_value + ((U_old - U_old_min_value)/(U_old_max_value - U_old_min_value)) * (U_new_max_value - U_new_min_value)
    u_resc = OFFSET_SPEED + (abs(u)/MAX_SPEED) * (MAX_SPEED - OFFSET_SPEED);

    if(u < 0)
      u = -u_resc;
    else
      u = u_resc;
  }
  else{
   
      if(u < 0)
        u = -MAX_SPEED;
      else
        u = MAX_SPEED;

      /* Rescaling is not necessary for maximal speed values (implicitly done). */
  }

  

  return u;
}

// Get target distance
double getDistance(void){

  byte domainIndex;
  unsigned int iter;
  double distance, IRSensorVoltage = 0.0;
  
  // Averaging for noise reduction
  for(iter = 0;iter < AVG_SAMPLE_NUMBER;++iter){
    
    IRSensorVoltage += analogRead(IR_SENSOR_VALUE)*ADC_STEP;
  }

  IRSensorVoltage = IRSensorVoltage/(double)AVG_SAMPLE_NUMBER;

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

// Program Logger
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
 
  // Remove trash from input buffer
  while(0 < Serial.available())
    Serial.read();

  // Wait for input data
  while(1 > Serial.available()){}

  inputData = Serial.read();

  switch(inputData){

    // Automate mode
    case 'A':
      mode = MODE_AUTO;
      break;

    // Manual mode  
    case 'M':
      referenceAngle = 0.0;
      msb = Serial.read();
      if(' ' == msb) // Handle LabVIEW specific 1 digit data (_<LSB> where _ is a space char)
        referenceAngle = (Serial.read() - '0');
      else{ // Handle LabVIEW specific 2 digit data (<char_MSB><char_LSB>)
      referenceAngle = (msb - '0') * 10;
      referenceAngle += (Serial.read() - '0');
      }

      // Saturate reference angle in case of data failure
      if((referenceAngle > MAX_REF_ANGLE) || (referenceAngle < MIN_REF_ANGLE))
        referenceAngle = SAFETY_ANGLE;
      mode = MODE_MANUAL;
      break;

    // Lock mode  
    case 'L':
      mode = MODE_LOCK;
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////// LOOP FUNCTION ////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  int u = 0; // Control sign
  double distance; // Target distance


  // Handle program mode switch on request
  if(0 < Serial.available()){

    inputData = Serial.read();
    
    switch(inputData){

    // Automate mode
    case 'A':
      mode = MODE_AUTO;
      break;

    // Manual mode  
    case 'M':
      referenceAngle = 0.0;
      msb = Serial.read();
      if(' ' == msb) // Handle LabVIEW specific 1 digit data (_<LSB> where _ is a space char)
        referenceAngle = (Serial.read() - '0');
      else{ // Handle LabVIEW specific 2 digit data (<char_MSB><char_LSB>)
      referenceAngle = (msb - '0') * 10;
      referenceAngle += (Serial.read() - '0');
      }

      // Saturate reference angle in case of failure
      if((referenceAngle > MAX_REF_ANGLE) || (referenceAngle < MIN_REF_ANGLE))
        referenceAngle = SAFETY_ANGLE;
      mode = MODE_MANUAL;
      break;

    // Lock mode  
    case 'L':
      mode = MODE_LOCK;
      break;
  }
 }

  // Program mode selector
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

        Serial.println(u); // Visualize u via LabVIEW chart
      }
      else{

        digitalWrite(LED_RED,HIGH); // Turn on red LED for this mode
        digitalWrite(LED_GREEN,LOW);
        digitalWrite(LED_BLUE,LOW);
        stopRotation(); // Stop cannon rotation
        updateActualAngle(); // Update actual cannon angle
        
        Serial.println(0); // Visualize u via LabVIEW chart
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
      Serial.println(u); // Visualize reference angle via LabVIEW chart (for debug)
      break;

    case MODE_LOCK:

      digitalWrite(LED_RED,LOW); // Turn on green LED for this mode
      digitalWrite(LED_GREEN,HIGH);
      digitalWrite(LED_BLUE,LOW);
      stopRotation(); // Stop cannon rotation
      updateActualAngle(); // Update actual cannon angle
      distance = getDistance();
      Serial.println(0); // Visualize u via LabVIEW chart
      break;
    
  }

  // log(u, distance);
  delay(SAMPLE_PERIOD);

}
