

#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include "math.h"

/*written by Kevin Le Bras

This program serves to collect data from Kevin Jones motor test setup.
It will collect current, rpm, temperature, and force data from the test bench and 
write that data into a text file on a SD card.
*/


//////////////////// TESTING PARAMETERS////////////////////////////////

#define FILE_NAME          "Vcalib.txt" // FILE NAME 

#define DATARATE           100 //in millisec
#define STEP_PERIOD        30000 //in milliseconds
#define MOTOR_INTERVALS    25    // range from 0-1000 

#define MOTOR_PULSE_HIGH   1900    //speed controller high and low pulse times in micro seconds
#define MOTOR_PULSE_LOW    1150

#define TEMP_TOO_HIGH      65   //in celcius
#define CURRENT_TOO_HIGH   16000   //in Milliamps 
#define POWER_TOO_HIGH     200000      //in milliwatts
#define VOLTAGE_TOO_HIGH   22000   //in millivolts

#define lb10_or_lb25    1     // 1 for 10, 0 for 25

//////////////////Define Pins//////////////////////////////////////
#define lb_PIN10 0
#define lb_PIN25 1
#define CURRENT_PIN 2
#define VOLTAGE_PIN 3
#define MOTOR_TEMP_PIN 4
#define SPEEDCTRL_TEMP_PIN 5
#define RPM_CNTR_PIN 0
#define MOTOR_PIN 8
#define DRIVER_PIN 8


#define TEMP_RESISTOR 2200


int SHUTDOWN = 0;


Servo Motor;

int  currentSpeed = 0;
int  speedVAR = 0;
//define recorded variables
long RPMcounter = 0;

//presure sensor variables
double PressureVoltage = 0;
int Pressure = 0;

int PRESSURE_PIN = lb_PIN10;
int PRESSURE_HIGH = 4535;
int PRESSURE_LOW  = 116;

//power consumption varaibles
int Current = 0;
int Voltage = 0;
int Power = 0;

// temperature variables
double tempMotor = 0;
double tempSpeedCtrl = 0;
double motorRes = 0;
double speedctrlRes = 0;


//rpm variables
long RPM = 0;
long RPMlast = 0;

//timing variables
long time_mill = 0;
long timelastRecord = 0;
long timelastSpeed = 0;
long timelastRPMcheck= 0;
long timelastChange = 0;

int motorTempVoltage = 0;
int speedctrlTempVoltage = 0;

const int chipSelect = 4;


//////////////Temperature sensor variables//////////////////

double A = 0;
double B = 0;
double C = 0;
// change these six variables to change the calibration

double R1 = 12263 ;   //resistance in ohms
double R2 = 1255;
double R3 = 237;

double T1 = 20;       // temperture in celcius
double T2 = 85;
double T3 = 150;



//////////////Initialization///////////////////////////////
void setup(){
    Serial.begin(9600);      // open the serial port at 9600 bps:   
    Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(53, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);   // sets the pin as output
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
    
    // set up the force sensor 
  }
  Serial.println("card initialized.");
  
  analogReadResolution(12); // changes resolution for arduino DUE
  
  Motor.attach(8);
  Motor.writeMicroseconds(0);  // reset motor 
  delay(1000);
 
// set up pins
// set up interupts
  attachInterrupt(2,RPMpulse,RISING);

// set up temperture sensor model (steinhart model)

double L1 = log(R1);
double L2 = log(R2);
double L3 = log(R3);

double Y1 = 1/T1;
double Y2 = 1/T2;
double Y3 = 1/T3;

double d2 = (Y2-Y1)/(L2-L1);
double d3 = (Y3-Y1)/(L3-L1);
//these are the 
C = ((d3-d2)/(L3-L2))/(L1+L2+L3);
B = d2 - C*(L1*L1 + L1*L2 + L2*L2);
A = Y1-((B+L1*L1*C)*L1);


if (lb10_or_lb25<1){
      PRESSURE_PIN = lb_PIN25;
}



// quick ramp up and shut down
Speed(200);
delay(3000);
Speed(0);
delay(3000);
WriteHeader();
GetData();
WriteData();
}

void loop(){
 
  
  time_mill  =  millis(); //get current time
  
  if (SHUTDOWN == 0){ // if not SHUT DOWN
    Speed(speedVAR); // write speed variable
  
    if(time_mill - timelastRecord > DATARATE){ //check to see if enough time has passed to get a new recording 
      timelastRecord  = time_mill ;//set new time to last time
  
      // shutdown conditions
      if ((tempMotor > TEMP_TOO_HIGH*10)||(tempSpeedCtrl > TEMP_TOO_HIGH*10)||(Current > CURRENT_TOO_HIGH)||(Voltage > VOLTAGE_TOO_HIGH)||(Power > POWER_TOO_HIGH)){
         SHUTDOWN = 1; // stop the test
      }
  
      GetData();   // gets data from ADC's and updates variables
      WriteData(); // writes the data to text file data
    }
  }
  
  else {
  Motor.writeMicroseconds(0); // shut down the motor if anything is wrong
  }
  
  
////////////////// change motor speed//////////////////////// 
  if(time_mill - timelastChange > STEP_PERIOD){
    
    timelastChange = time_mill;
    speedVAR = speedVAR + MOTOR_INTERVALS;
    
   if (speedVAR>1000){ 
   SHUTDOWN = 1;
     }
   
  }
/////////////////updates the RPM variable//////////////////
  
  
  if(time_mill - timelastRPMcheck > 1000){  // checks every 1 second
    timelastRPMcheck = time_mill;  // update last check tiem
    RPM = 30*(RPMcounter - RPMlast);  // convert from 2*hz to RPM
    RPMlast = RPMcounter;    
  }
   
}  


///////////////////RPM Interupt///////////////////////////
void RPMpulse(){
RPMcounter++;    // counts the number of ticks
}

//////////////////Speed function///////////////////////////

void Speed(int motorSpeed){ // maps a 0-1000 value to the appropraite pulse width to send to the motor
  
  if (motorSpeed > 1000) motorSpeed = 1000; // keeps the input value within constraints
  if (motorSpeed < 0) motorSpeed = 0;
  
  int pulseWidth = map(motorSpeed,0,1000,MOTOR_PULSE_LOW,MOTOR_PULSE_HIGH); //maps the value
  
  if (motorSpeed == 0) pulseWidth = 0; // if the motor 
  
  Motor.writeMicroseconds(pulseWidth);
  
  currentSpeed = motorSpeed;
}


//////////////////////////write data string to text file///////////////////////////////////
void WriteData(){
  String dataString = ""; // data string for log
  // create the data string
  dataString += String(time_mill);
  dataString += "\t";
  dataString += String(speedVAR);
  dataString += "\t";
  dataString += String((int)PressureVoltage);
  dataString += "\t";
  dataString += String(Voltage);
  dataString += "\t";
  dataString += String(Current);
  dataString += "\t";
  dataString += String(RPMcounter);
  dataString += "\t";
  dataString += String(RPM);
  dataString += "\t";
  dataString += String((int)(tempMotor*10));
  dataString += "\t";
  dataString += String((int)(tempSpeedCtrl*10));
  dataString += "\t";
  dataString += String(SHUTDOWN);
// uncoment for data recording
 File dataFile = SD.open(FILE_NAME, FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    
      } 
}

void GetData() {
  
  int cnt = 0;
  for (cnt = 0;cnt<10;cnt++){
  PressureVoltage = PressureVoltage + (double)analogRead(PRESSURE_PIN); //read pressure voltage
  }
  PressureVoltage = PressureVoltage/cnt;
  
  Voltage = analogRead(VOLTAGE_PIN); //read pressure voltage
  Current = analogRead(CURRENT_PIN); //read pressure voltage
  
  // calculate the temperatures
 // get raw voltage 
 motorTempVoltage = analogRead(MOTOR_TEMP_PIN);
 speedctrlTempVoltage = analogRead(SPEEDCTRL_TEMP_PIN);
 
 
 // calculate resistance
 motorRes = TEMP_RESISTOR*((4096/(double)motorTempVoltage)-1);
 speedctrlRes = TEMP_RESISTOR*((4096/(double)speedctrlTempVoltage)-1);
 
 
 // calculate the temperature using the steinhart model
 tempMotor = 1/(A+B*log(motorRes)+C*pow(log(motorRes),3));
 tempSpeedCtrl = 1/(A+B*log(speedctrlRes)+C*pow(log(speedctrlRes),3)); 
  
Pressure = (PressureVoltage*5.524)-629.84; // calculates the force of the system (not used)
// Voltage = (int)((double)Voltage*3.37);
Current = (int)((double)Current*11.26);
Power = (Voltage*Current)/1000;
}

////////////////// Write Header Function//////////////////////////


void WriteHeader(){ // writes data header to text file
  String dataString = "";
// create the header string
  dataString += "Time";
  dataString += "\t";
  dataString += "THR";
  dataString += "\t";
  dataString += "Force g";
  dataString += "\t";
  dataString += "Voltage";
  dataString += "\t";
  dataString += "Current";
  dataString += "\t";
  dataString += "Revs";
  dataString += "\t";
  dataString += "RPM";
  dataString += "\t";
  dataString += "Temp1";
  dataString += "\t";
  dataString += "Temp2";
  dataString += "\t";
  dataString += "SHUTDOWN";
// uncoment for data recording
 File dataFile = SD.open(FILE_NAME, FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
