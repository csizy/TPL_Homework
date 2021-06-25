// Created on: 2019.11.18.
// Author: Ádám Csizy
//
// Sketch for identifying the IR distance sensor mechanisms and characteristic.
// Characteristic source: own measurement.

// Signal - pin definitions
#define IR_SENSOR_VALUE A1

// Utility definitions
#define ADC_Range 1023.0
#define Vcc 3.3
#define INVALID_DOMAIN_INDEX 255

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

typedef struct{
  
  double steepness;
  double offset;
} linearChar;

// Setup function
void setup() {

  // Set pin modes
  pinMode(IR_SENSOR_VALUE, INPUT);

  // Set analog reference
  analogReference(DEFAULT); // 3.3 V

  // Setup serial communication
  Serial.begin(9600);

}

void loop() {

  // Variables
  byte domainIndex;
  double ADCStep = Vcc/ADC_Range;
  double IRSensorVoltage = analogRead(IR_SENSOR_VALUE)*ADCStep;

  // IR distance sensor characteristics with linear interpolation
  linearChar charArray[25] = {{STEEP0,OFFSET0},{STEEP1,OFFSET1},{STEEP2,OFFSET2},{STEEP3,OFFSET3},{STEEP4,OFFSET4},{STEEP5,OFFSET5},{STEEP6,OFFSET6},{STEEP7,OFFSET7},{STEEP8,OFFSET8},
  {STEEP9,OFFSET9},{STEEP10,OFFSET10},{STEEP11,OFFSET11},{STEEP12,OFFSET12},{STEEP13,OFFSET13},{STEEP14,OFFSET14},{STEEP15,OFFSET15},{STEEP16,OFFSET16},{STEEP17,OFFSET17},
  {STEEP18,OFFSET18},{STEEP19,OFFSET19},{STEEP20,OFFSET20},{STEEP21,OFFSET21},{STEEP22,OFFSET22},{STEEP23,OFFSET23},{STEEP24,OFFSET24}};

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

 if(INVALID_DOMAIN_INDEX != domainIndex){

  // Send raw IR sensor value and converted distance to PC on serial port
  Serial.print("Voltage: \t");
  Serial.print(IRSensorVoltage);
  Serial.print("\tDistance:\t");
  Serial.println((IRSensorVoltage-charArray[domainIndex].offset)/charArray[domainIndex].steepness);
 }
 else{

  // Send ERROR message to PC on serial port
  Serial.print("Voltage:\t");
  Serial.print(IRSensorVoltage);
  Serial.print("\tDistance:\t");
  Serial.println("WARNING: OBJECT IS OUT OF THE CHARACTERISTICS RANGE !");
 }
}
