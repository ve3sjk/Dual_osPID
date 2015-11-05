#include <EEPROM.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <UTFT.h>
#include <UTFT_DLB.h>
#include <UTouch.h>
#include <UTFT_Buttons.h>
#include <UTFT_DLB_Buttons.h>


#include "MAX31855_local.h"
#include "PID_v1_local.h"
#include "EEPROMAnything.h"
#include "PID_AutoTune_v0_local.h"
#include "LCDI2Cw_local.h"

// ***** PIN ASSIGNMENTS *****

const byte buzzerPin = 11;
const byte systemLEDPin = 13;
//const byte thermocoupleCS = 5;
//const byte thermocoupleCS2 = 6;
//const byte thermocoupleSO = 12;
//const byte thermocoupleCLK = 4;
const byte pid1_SSRPin = 10;
const byte pid2_SSRPin = 9;
const byte oneWireBus =8;

//const unsigned char i2cAddress = 0x4C;  // LCD module I2C address
char buffer2 [8];
const byte EEPROM_ID = 2; //used to automatically trigger and eeprom reset after firmware update (if necessary)

//added digital probe init code here

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

DeviceAddress tempSensor1 = {0x28, 0xFF, 0xBF, 0x86, 0x71, 0x15, 0x01, 0xB1 }; //Sensor 1
DeviceAddress tempSensor2 = {0x28, 0xFF, 0xF9, 0x1E, 0x71, 0x15, 0x02, 0xDB }; //Sensor 2
DeviceAddress tempSensor3 = {0x28, 0xFF, 0x6F, 0x28, 0x71, 0x15, 0x02, 0xB1 }; //Sensor 3
DeviceAddress tempSensor4 = {0x28, 0x95, 0x54, 0xEB, 0x03, 0x00, 0x00, 0xF7 }; //Sensor 4
DeviceAddress tempSensor5 = {0x28, 0x23, 0x50, 0xEB, 0x03, 0x00, 0x00, 0x5D }; //Sensor 5
DeviceAddress tempSensor6 = {0x28, 0xAB, 0x79, 0x51, 0x03, 0x00, 0x00, 0x97 }; //Sensor 6

float tempcheckSensor1;
float tempcheckSensor2;
float tempcheckSensor3;
float tempcheckSensor4;
float tempcheckSensor5;
float tempcheckSensor6;


//MAX31855 thermocouple(thermocoupleSO, thermocoupleCS, thermocoupleCLK);
//MAX31855 thermocouple2(thermocoupleSO, thermocoupleCS2, thermocoupleCLK);
//LCDI2Cw lcd(16, 2, i2cAddress);

//**********************added touch screen init

extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t SevenSegNumFont[];
extern uint8_t DejaVuSans18[];
extern uint8_t DejaVuSans24[];



UTFT_DLB      myGLCD(SSD1963_800ALT,38,39,40,41);
//UTFT_SPIflash myFiles(&myGLCD, &myFlash);
UTouch        myTouch(6,5,4,3,2);
UTFT_DLB_Buttons myButtons(&myGLCD, &myTouch);



unsigned long now, tft_Time, buttonTime, ioTime, serialTime;
boolean sendInfo=true, sendDash=true, sendTune=true, sendAtune=true;

bool editing=false;

bool pid1_tuning = false;
bool pid2_tuning= false;

double pid1_error, pid2_error;
int cmd_b0, cmd_b1;

double pid1_setpoint=255, pid1_input=255, pid1_output=255, pid1_pidInput=255;
double pid2_setpoint=100, pid2_input=100, pid2_output=100, pid2_pidInput=100;

double pid1_kp=255, pid1_ki=255, pid1_kd=255;
double pid2_kp=100, pid2_ki=100, pid2_kd=100;

byte pid1_ctrlDirection = 1;
byte pid2_ctrlDirection = 1;
byte pid1_modeIndex = 1;
byte pid2_modeIndex = 1;
byte databyte;

PID myPID1(&pid1_pidInput, &pid1_output, &pid1_setpoint,pid1_kp,pid1_ki,pid1_kd, DIRECT);
PID myPID2(&pid2_pidInput, &pid2_output, &pid2_setpoint,pid2_kp,pid2_ki,pid2_kd, DIRECT);

double pid1_aTuneStep = 25.0, pid1_aTuneNoise = 25.0;
double pid2_aTuneStep = 10.0, pid2_aTuneNoise = 10.0;

unsigned int pid1_aTuneLookBack = 25.0;
unsigned int pid2_aTuneLookBack = 10.0;

byte pid1_ATuneModeRember = 0;
byte pid2_ATuneModeRember = 0;

PID_ATune pid1_aTune(&pid1_pidInput, &pid1_output);
PID_ATune pid2_aTune(&pid2_pidInput, &pid2_output);


byte pid1_curProfStep=0;
byte pid2_curProfStep=0;
byte pid1_curType=0;
byte pid2_curType=0;
float pid1_curVal=0;
float pid2_curVal=0;
float pid1_helperVal=0;
float pid2_helperVal=0;
unsigned long pid1_helperTime=0;
unsigned long pid2_helperTime=0;
boolean pid1_helperFlag=false;
boolean pid2_helperFlag=false;
unsigned long pid1_curTime=0;
unsigned long pid2_curTime=0;

/*Profile declarations*/
const unsigned long pid1_profReceiveTimeout = 10000;
const unsigned long pid2_profReceiveTimeout = 10000;
unsigned long pid1_profReceiveStart=0;
unsigned long pid2_profReceiveStart=0;
boolean pid1_receivingProfile=false;
boolean pid2_receivingProfile=false;
const int pid1_nProfSteps = 15;
const int pid2_nProfSteps = 15;
byte pid1_profTypes[pid1_nProfSteps];
byte pid2_profTypes[pid2_nProfSteps];
unsigned long pid1_profTimes[pid1_nProfSteps];
unsigned long pid2_profTimes[pid2_nProfSteps];
float pid1_profVals[pid1_nProfSteps];
float pid2_profVals[pid2_nProfSteps];
boolean pid1_runningProfile = false;
boolean pid2_runningProfile = false;


void setup()
{
  TCCR1B = (TCCR1B & 0b11111000) | 0x05;
  Serial.begin(115200);
//  buttonTime=1;
  ioTime=5;
  serialTime=6;
  tft_Time=10;
  
  //windowStartTime=2;

  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setColor(255, 120, 120);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setFont(DejaVuSans18);
  Serial.println(" Dual osPID   ");
  Serial.println(" v1.00   ");
  myButtons.setTextFont(DejaVuSans18);
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);

//  delay(1000);

  initializeEEPROM();

  InitializeOutputCard();
  
  myPID1.SetSampleTime(1000);
  myPID1.SetOutputLimits(0, 255);
  myPID1.SetTunings(pid1_kp, pid1_ki, pid1_kd);
  myPID1.SetControllerDirection(pid1_ctrlDirection);
  myPID1.SetMode(pid1_modeIndex);
  
  myPID2.SetSampleTime(1000);
  myPID2.SetOutputLimits(0, 255);
  myPID2.SetTunings(pid2_kp, pid2_ki, pid2_kd);
  myPID2.SetControllerDirection(pid2_ctrlDirection);
  myPID2.SetMode(pid2_modeIndex);

}
//***************************loop start*****************
byte editDepth=0;
void loop()
{
  now = millis();
 // Serial.println(pid1_kp);
 // Serial.println(now);
  bool doIO = now >= ioTime;
  //read in the input
  if(doIO)
  { 
    ioTime+=250;
    //ReadInput();
    //ReadInput2();
    ReadDallas();
    //if(!isnan(pid1_input))pid1_pidInput = pid1_input;
    //if(!isnan(pid2_input))pid2_pidInput = pid2_input;
    pid1_pidInput = pid1_input;
    pid2_pidInput = pid2_input;
  }
  
  //Serial.println("done input/output");

  if(pid1_tuning)
  {
    byte cmd_b0 = (pid1_aTune.Runtime());

    if(cmd_b0 != 0)
    {
      pid1_tuning = false;
    }

    if(!pid1_tuning)
    { 
      // We're done, set the pid1_tuning parameters
      pid1_kp = pid1_aTune.GetKp();
      pid1_ki = pid1_aTune.GetKi();
      pid1_kd = pid1_aTune.GetKd();
      myPID1.SetTunings(pid1_kp, pid1_ki, pid1_kd);
      pid1_AutoTuneHelper(false);
      EEPROMBackupTunings();
    }
  }
  else
  {
    if(pid1_runningProfile) pid1_ProfileRunTime();
    //allow the pid to compute if necessary
    myPID1.Compute();
  }

 // Serial.println("done pid1 tuning check");
  
  if(pid2_tuning)
  {
    byte cmd_b1 = (pid2_aTune.Runtime());

    if(cmd_b1 != 0)
    {
      pid2_tuning = false;
    }

    if(!pid2_tuning)
    { 
      // We're done, set the tuning parameters
      pid2_kp = pid2_aTune.GetKp();
      pid2_ki = pid2_aTune.GetKi();
      pid2_kd = pid2_aTune.GetKd();
      myPID2.SetTunings(pid2_kp, pid2_ki, pid2_kd);
      pid2_AutoTuneHelper(false);
      EEPROMBackupTunings();
    }
  }
  else
  {
    if(pid2_runningProfile) pid2_ProfileRunTime();
    //allow the pid to compute if necessary
    myPID2.Compute();
  }
 // Serial.println("done pid2 tuning check");
  if(doIO)
  {
    //send to output card
    pid1_WriteToOutput();
    pid2_WriteToOutput();
  }
  
  //Serial.println("done send output");
  
  if(now>tft_Time)
  {
  drawTFT();
  tft_Time+=250;
  }
//  Serial.println("done tft");
//  Serial.println(serialTime);
  if(millis() > serialTime)
  {
//    if(pid1_receivingProfile && (now-pid1_profReceiveStart)>pid1_profReceiveTimeout) pid1_receivingProfile = false;
//    Serial.println("pid1prof");
//    if(pid2_receivingProfile && (now-pid2_profReceiveStart)>pid2_profReceiveTimeout) pid2_receivingProfile = false;
//    Serial.println("pid2prof");
    SerialReceive();
  //  Serial.println("done serial recv");
    SerialSend();
   // Serial.println("done serial send");
    serialTime += 500;
  }
}
//**********************************************end loop
//******************************************************

void drawTFT()
{
  myGLCD.clrScr();
  
  myGLCD.print("Input",90,20);
  dtostrf (pid1_input,5,2,buffer2);
  myGLCD.print(buffer2,85,50);
  myGLCD.print("PID",40,20);
  dtostrf (pid2_input,5,2,buffer2);
  myGLCD.print(buffer2,80,80);
  myGLCD.print("Setpoint",150,20);
  dtostrf (pid1_setpoint,5,2,buffer2);
  myGLCD.print(buffer2,150,50);
  myGLCD.print("001",40,50);
  myGLCD.print("002",40,80);
  dtostrf (pid2_setpoint,5,2,buffer2);
  myGLCD.print(buffer2,150,80);
  
}

void ReadDallas()
{
    sensors.requestTemperatures();
    tempcheckSensor1= sensors.getTempF(tempSensor1);
    tempcheckSensor2= sensors.getTempF(tempSensor2);
    tempcheckSensor3= sensors.getTempF(tempSensor3);
    tempcheckSensor4= sensors.getTempF(tempSensor4);
    tempcheckSensor5= sensors.getTempF(tempSensor5);
    tempcheckSensor6= sensors.getTempF(tempSensor6);

      pid1_input = tempcheckSensor4;
      pid2_input = tempcheckSensor5;
}   

//void ReadInput()
//{
//   double pid1_input = thermocouple.readThermocouple(CELSIUS);
//   if (pid1_input==FAULT_OPEN) || (pid1_input==FAULT_SHORT_GND) || (pid1_input==FAULT_SHORT_VCC))
//   {
//     pid1_error = pid1_input;
//     pid1_input = NAN;
//   }
//}

//void ReadInput2()
//{
//   double pid2_input = thermocouple2.readThermocouple(CELSIUS);
//   if (pid2_input==FAULT_OPEN || pid2_input==FAULT_SHORT_GND || pid2_input==FAULT_SHORT_VCC)
//   {
//     pid2_error = pid2_input;
//     pid2_input = NAN;
//   }
//}

void InitializeOutputCard()
{
  analogWrite(pid1_SSRPin, 0);
  analogWrite(pid2_SSRPin, 0);
}

void pid1_WriteToOutput()
{
  analogWrite(pid1_SSRPin, pid1_output);
}

void pid2_WriteToOutput()
{
  analogWrite(pid2_SSRPin, pid2_output);
}

void pid1_changeAutoTune()
{
  if(!pid1_tuning)
  {
    //initiate autotune
    pid1_AutoTuneHelper(true);
    pid1_aTune.SetNoiseBand(pid1_aTuneNoise);
    pid1_aTune.SetOutputStep(pid1_aTuneStep);
    pid1_aTune.SetLookbackSec((int)pid1_aTuneLookBack);
    pid1_tuning = true;
  }
  else
  { //cancel autotune
    pid1_aTune.Cancel();
    pid1_tuning = false;
    pid1_AutoTuneHelper(false);
  }
}

void pid2_changeAutoTune()
{
  if(!pid2_tuning)
  {
    //initiate autotune
    pid2_AutoTuneHelper(true);
    pid2_aTune.SetNoiseBand(pid2_aTuneNoise);
    pid2_aTune.SetOutputStep(pid2_aTuneStep);
    pid2_aTune.SetLookbackSec((int)pid2_aTuneLookBack);
    pid2_tuning = true;
  }
  else
  { //cancel autotune
    pid2_aTune.Cancel();
    pid2_tuning = false;
    pid2_AutoTuneHelper(false);
  }
}

void pid1_AutoTuneHelper(boolean start)
{

  if(start)
  {
    pid1_ATuneModeRember = myPID1.GetMode();
    myPID1.SetMode(MANUAL);
  }
  else
  {
    pid1_modeIndex = pid1_ATuneModeRember;
    myPID1.SetMode(pid1_modeIndex);
  } 
}

void pid2_AutoTuneHelper(boolean start)
{

  if(start)
  {
    pid2_ATuneModeRember = myPID2.GetMode();
    myPID2.SetMode(MANUAL);
  }
  else
  {
    pid2_modeIndex = pid2_ATuneModeRember;
    myPID2.SetMode(pid2_modeIndex);
  } 
}

void pid1_StartProfile()
{
  if(!pid1_runningProfile)
  {
    //initialize profle
    pid1_curProfStep=0;
    pid1_runningProfile = true;
    pid1_calcNextProf();
  }
}

void pid2_StartProfile()
{
  if(!pid2_runningProfile)
  {
    //initialize profle
    pid2_curProfStep=0;
    pid2_runningProfile = true;
    pid2_calcNextProf();
  }
}

void pid1_StopProfile()
{
  if(pid1_runningProfile)
  {
    pid1_curProfStep=pid1_nProfSteps;
    pid1_calcNextProf(); //pid1_runningProfile will be set to false in here
  } 
}

void pid2_StopProfile()
{
  if(pid2_runningProfile)
  {
    pid2_curProfStep=pid2_nProfSteps;
    pid2_calcNextProf(); //runningProfile will be set to false in here
  } 
}

void pid1_ProfileRunTime()
{
  if(pid1_tuning || !pid1_runningProfile) return;
 
  boolean pid1_gotonext = false;

  //what are we doing?
  if(pid1_curType==1) //ramp
  {
    //determine the value of the setpoint
    if(now>pid1_helperTime)
    {
      pid1_setpoint = pid1_curVal;
      pid1_gotonext=true;
    }
    else
    {
      pid1_setpoint = (pid1_curVal-pid1_helperVal)*(1-(float)(pid1_helperTime-now)/(float)(pid1_curTime))+pid1_helperVal; 
    }
  }
  else if (pid1_curType==2) //wait
  {
    float pid1_err = pid1_input-pid1_setpoint;
    if(pid1_helperFlag) //we're just looking for a cross
    {

      if(pid1_err==0 || (pid1_err>0 && pid1_helperVal<0) || (pid1_err<0 && pid1_helperVal>0)) pid1_gotonext=true;
      else pid1_helperVal = pid1_err;
    }
    else //value needs to be within the band for the prescribed time
    {
      if (abs(pid1_err)>pid1_curVal) pid1_helperTime=now; //reset the clock
      else if( (now-pid1_helperTime)>=pid1_curTime) pid1_gotonext=true; //we held for long enough
    }

  }
  else if(pid1_curType==3) //step
  {

    if((now-pid1_helperTime)>pid1_curTime)pid1_gotonext=true;
  }
  else if(pid1_curType==127) //buzz
  {
    if(now<pid1_helperTime)digitalWrite(buzzerPin,HIGH);
    else 
    {
       digitalWrite(buzzerPin,LOW);
       pid1_gotonext=true;
    }
  }
  else
  { //unrecognized type, kill the profile
    pid1_curProfStep=pid1_nProfSteps;
    pid1_gotonext=true;
  }
  if(pid1_gotonext)
  {
    pid1_curProfStep++;
    pid1_calcNextProf();
  }
}

void pid2_ProfileRunTime()
{
  if(pid2_tuning || !pid2_runningProfile) return;
 
  boolean pid2_gotonext = false;

  //what are we doing?
  if(pid2_curType==1) //ramp
  {
    //determine the value of the setpoint
    if(now>pid2_helperTime)
    {
      pid2_setpoint = pid2_curVal;
      pid2_gotonext=true;
    }
    else
    {
      pid2_setpoint = (pid2_curVal-pid2_helperVal)*(1-(float)(pid2_helperTime-now)/(float)(pid2_curTime))+pid2_helperVal; 
    }
  }
  else if (pid2_curType==2) //wait
  {
    float pid2_err = pid2_input-pid2_setpoint;
    if(pid2_helperFlag) //we're just looking for a cross
    {

      if(pid2_err==0 || (pid2_err>0 && pid2_helperVal<0) || (pid2_err<0 && pid2_helperVal>0)) pid2_gotonext=true;
      else pid2_helperVal = pid2_err;
    }
    else //value needs to be within the band for the perscribed time
    {
      if (abs(pid2_err)>pid2_curVal) pid2_helperTime=now; //reset the clock
      else if( (now-pid2_helperTime)>=pid2_curTime) pid2_gotonext=true; //we held for long enough
    }

  }
  else if(pid2_curType==3) //step
  {

    if((now-pid2_helperTime)>pid2_curTime)pid2_gotonext=true;
  }
  else if(pid2_curType==127) //buzz
  {
    if(now<pid2_helperTime)digitalWrite(buzzerPin,HIGH);
    else 
    {
       digitalWrite(buzzerPin,LOW);
       pid2_gotonext=true;
    }
  }
  else
  { //unrecognized type, kill the profile
    pid2_curProfStep=pid2_nProfSteps;
    pid2_gotonext=true;
  }
  if(pid2_gotonext)
  {
    pid2_curProfStep++;
    pid2_calcNextProf();
  }
}

void pid1_calcNextProf()
{
  if(pid1_curProfStep>=pid1_nProfSteps) 
  {
    pid1_curType=0;
    pid1_helperTime=0;
  }
  else
  { 
    pid1_curType = pid1_profTypes[pid1_curProfStep];
    pid1_curVal = pid1_profVals[pid1_curProfStep];
    pid1_curTime = pid1_profTimes[pid1_curProfStep];
  }
  if(pid1_curType==1) //ramp
  {
    pid1_helperTime = pid1_curTime + now; //at what time the ramp will end
    pid1_helperVal = pid1_setpoint;
  }   
  else if(pid1_curType==2) //wait
  {
    pid1_helperFlag = (pid1_curVal==0);
    if(pid1_helperFlag) pid1_helperVal= pid1_input-pid1_setpoint;
    else pid1_helperTime=now; 
  }
  else if(pid1_curType==3) //step
  {
    pid1_setpoint = pid1_curVal;
    pid1_helperTime = now;
  }
  else if(pid1_curType==127) //buzzer
  {
    pid1_helperTime = now + pid1_curTime;    
  }
  else
  {
    pid1_curType=0;
  }

  if(pid1_curType==0) //end
  { //we're done 
    pid1_runningProfile=false;
    pid1_curProfStep=0;
    Serial.println("P_DN");
    digitalWrite(buzzerPin,LOW);
  } 
  else
  {
    Serial.print("P_STP ");
    Serial.print(int(pid1_curProfStep));
    Serial.print(" ");
    Serial.print(int(pid1_curType));
    Serial.print(" ");
    Serial.print((pid1_curVal));
    Serial.print(" ");
    Serial.println((pid1_curTime));
  }
}

void pid2_calcNextProf()
{
  if(pid2_curProfStep>=pid2_nProfSteps) 
  {
    pid2_curType=0;
    pid2_helperTime =0;
  }
  else
  { 
    pid2_curType = pid2_profTypes[pid2_curProfStep];
    pid2_curVal = pid2_profVals[pid2_curProfStep];
    pid2_curTime = pid2_profTimes[pid2_curProfStep];
  }
  if(pid2_curType==1) //ramp
  {
    pid2_helperTime = pid2_curTime + now; //at what time the ramp will end
    pid2_helperVal = pid2_setpoint;
  }   
  else if(pid2_curType==2) //wait
  {
    pid2_helperFlag = (pid2_curVal==0);
    if(pid2_helperFlag) pid2_helperVal= pid2_input-pid2_setpoint;
    else pid2_helperTime=now; 
  }
  else if(pid2_curType==3) //step
  {
    pid2_setpoint = pid2_curVal;
    pid2_helperTime = now;
  }
  else if(pid2_curType==127) //buzzer
  {
    pid2_helperTime = now + pid2_curTime;    
  }
  else
  {
    pid2_curType=0;
  }

  if(pid2_curType==0) //end
  { //we're done 
    pid2_runningProfile=false;
    pid2_curProfStep=0;
    Serial.println("P2_DN");
    digitalWrite(buzzerPin,LOW);
  } 
  else
  {
    Serial.print("P2_STP ");
    Serial.print(int(pid2_curProfStep));
    Serial.print(" ");
    Serial.print(int(pid2_curType));
    Serial.print(" ");
    Serial.print((pid2_curVal));
    Serial.print(" ");
    Serial.println((pid2_curTime));
  }
}



const int pid1_eepromTuningOffset = 1; //13 bytes
const int pid1_eepromATuneOffset = 14; //12 bytes
const int pid2_eepromTuningOffset = 26; //13 bytes
const int pid2_eepromATuneOffset = 39; //12 bytes
const int pid1_eepromProfileOffset = 51; //128 bytes
const int pid2_eepromProfileOffset = 179; //128 bytes


void initializeEEPROM()
{
  //read in eeprom values
  byte firstTime = EEPROM.read(0);
  if(firstTime!=EEPROM_ID)
  {//the only time this won't be 1 is the first time the program is run after a reset or firmware update
    //clear the EEPROM and initialize with default values
    for(int i=1;i<1024;i++) EEPROM.write(i,0);
    EEPROMBackupTunings();
    EEPROMBackupATune();
    EEPROMBackupProfile();
    EEPROM.write(0,EEPROM_ID); //so that first time will never be true again (future firmware updates notwithstanding)
  }
  else
  {
    EEPROMRestoreTunings();
    EEPROMRestoreATune();
    EEPROMRestoreProfile();    
  }
}  

void EEPROMreset()
{
  EEPROM.write(0,0);
}

void EEPROMBackupTunings()
{
  EEPROM.write(pid1_eepromTuningOffset,pid1_ctrlDirection);
  EEPROM_writeAnything(pid1_eepromTuningOffset+1,pid1_kp);
  EEPROM_writeAnything(pid1_eepromTuningOffset+5,pid1_ki);
  EEPROM_writeAnything(pid1_eepromTuningOffset+9,pid1_kd);
 // Serial.println(pid1_eepromTuningOffset+1);
 // Serial.println(pid1_kp);
 // Serial.println(pid1_eepromTuningOffset+5);
 // Serial.println(pid1_ki);
  //Serial.println(pid1_eepromTuningOffset+9);
//  Serial.println(pid1_kd);
  EEPROM.write(pid2_eepromTuningOffset,pid2_ctrlDirection);
  EEPROM_writeAnything(pid2_eepromTuningOffset+1,pid2_kp);
  EEPROM_writeAnything(pid2_eepromTuningOffset+5,pid2_ki);
  EEPROM_writeAnything(pid2_eepromTuningOffset+9,pid2_kd);
//  Serial.println(pid2_eepromTuningOffset+1);
//  Serial.println(pid2_kp);
//  Serial.println(pid2_eepromTuningOffset+5);
//  Serial.println(pid2_ki);
//  Serial.println(pid2_eepromTuningOffset+9);
//  Serial.println(pid2_kd);

}

void EEPROMRestoreTunings()
{
  pid1_ctrlDirection = EEPROM.read(pid1_eepromTuningOffset);
  EEPROM_readAnything(pid1_eepromTuningOffset+1,pid1_kp);
  EEPROM_readAnything(pid1_eepromTuningOffset+5,pid1_ki);
  EEPROM_readAnything(pid1_eepromTuningOffset+9,pid1_kd);
 // Serial.println(pid1_eepromTuningOffset+1);
//  Serial.println(pid1_kp);
 // Serial.println(pid1_eepromTuningOffset+5);
 // Serial.println(pid1_ki);
 // Serial.println(pid1_eepromTuningOffset+9);
//  Serial.println(pid1_kd);
  
  pid2_ctrlDirection = EEPROM.read(pid2_eepromTuningOffset);
  EEPROM_readAnything(pid2_eepromTuningOffset+1,pid2_kp);
  EEPROM_readAnything(pid2_eepromTuningOffset+5,pid2_ki);
  EEPROM_readAnything(pid2_eepromTuningOffset+9,pid2_kd);
 // Serial.println(pid2_eepromTuningOffset+1);
 // Serial.println(pid2_kp);
 // Serial.println(pid2_eepromTuningOffset+5);
 // Serial.println(pid2_ki);
 // Serial.println(pid2_eepromTuningOffset+9);
 // Serial.println(pid2_kd);

  
}

void EEPROMBackupATune()
{
  EEPROM_writeAnything(pid1_eepromATuneOffset,pid1_aTuneStep);
  EEPROM_writeAnything(pid1_eepromATuneOffset+4,pid1_aTuneNoise);
  EEPROM_writeAnything(pid1_eepromATuneOffset+8,pid1_aTuneLookBack);
  EEPROM_writeAnything(pid2_eepromATuneOffset,pid2_aTuneStep);
  EEPROM_writeAnything(pid2_eepromATuneOffset+4,pid2_aTuneNoise);
  EEPROM_writeAnything(pid2_eepromATuneOffset+8,pid2_aTuneLookBack);
}

void EEPROMRestoreATune()
{
  EEPROM_readAnything(pid1_eepromATuneOffset,pid1_aTuneStep);
  EEPROM_readAnything(pid1_eepromATuneOffset+4,pid1_aTuneNoise);
  EEPROM_readAnything(pid1_eepromATuneOffset+8,pid1_aTuneLookBack);
  EEPROM_readAnything(pid2_eepromATuneOffset,pid2_aTuneStep);
  EEPROM_readAnything(pid2_eepromATuneOffset+4,pid2_aTuneNoise);
  EEPROM_readAnything(pid2_eepromATuneOffset+8,pid2_aTuneLookBack);
}

void EEPROMBackupProfile()
{
  EEPROM_writeAnything(pid1_eepromProfileOffset, pid1_profTypes);
  EEPROM_writeAnything(pid1_eepromProfileOffset + 16, pid1_profVals);
  EEPROM_writeAnything(pid1_eepromProfileOffset + 77, pid1_profTimes); //there might be a slight issue here (/1000?)
  EEPROM_writeAnything(pid2_eepromProfileOffset, pid2_profTypes);
  EEPROM_writeAnything(pid2_eepromProfileOffset + 16, pid2_profVals);
  EEPROM_writeAnything(pid2_eepromProfileOffset + 77, pid2_profTimes); //there might be a slight issue here (/1000?)
}

void EEPROMRestoreProfile()
{
  EEPROM_readAnything(pid1_eepromProfileOffset, pid1_profTypes);
  EEPROM_readAnything(pid1_eepromProfileOffset + 16, pid1_profVals);
  EEPROM_readAnything(pid1_eepromProfileOffset + 77, pid1_profTimes); //there might be a slight issue here (/1000?)
  EEPROM_readAnything(pid2_eepromProfileOffset, pid2_profTypes);
  EEPROM_readAnything(pid2_eepromProfileOffset + 16, pid2_profVals);
  EEPROM_readAnything(pid2_eepromProfileOffset + 77, pid2_profTimes); //there might be a slight issue here (/1000?)
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/

boolean ackDash = false, ackTune = false;
union {                // This Data structure lets
  byte asBytes[32];    // us take the byte array
  float asFloat[8];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats


void SerialReceive()
{

  // read the bytes sent from Processing
  
  byte identifier=0;
  byte index=0;
  byte cmd_b0=0,cmd_b1=0,cmd_b2=0;

  if(Serial.available())
  {
    byte identifier = Serial.read();
    byte cmd_b0 = Serial.read();
    byte cmd_b1 = Serial.read();
    while (Serial.available())
    {
      byte cmd_b2 = Serial.read();
      foo.asBytes[index] = cmd_b2;
      index++;
    }
    Serial.println("ready");
    switch(identifier)
    {
      case 0: //Settings Received 
        ReceiveSettings();
        break;
      case 1: //Tunings Received 
        ReceiveTunings();
        break;
      case 2: //auto tune Received
        ReceiveAtune();
        break;
      case 3: //EEPROM reset
        if((cmd_b0==9) && (cmd_b1==8))
        {
          EEPROMreset(); 
        }
        break;
      case 4:  //receiving profile
        pid1_ReceiveProfile();
        break;
      case 5:  //receiving profile2
        pid2_ReceiveProfile();
        break;
      case 6: //profile command
        ProfileCommand();
        break;
      default:
        break;
    }
  }
}

void ReceiveSettings()
{
  pid1_setpoint=double(foo.asFloat[0]);
  pid2_setpoint=double(foo.asFloat[2]);
  
  if(cmd_b0==0)                              // * change PID1 mode to manual and set output 
  {
    myPID1.SetMode(MANUAL);
    pid1_modeIndex=0;
    pid1_output=double(foo.asFloat[1]);
  }
  else 
  {
    myPID1.SetMode(AUTOMATIC);
    pid1_modeIndex=1;
  }

  if(cmd_b1==0)                              // * change PID2 mode to manual and set output
  {
    myPID2.SetMode(MANUAL);
    pid2_modeIndex=0;
    pid2_output=double(foo.asFloat[3]);
  }
  else 
  {
    myPID2.SetMode(AUTOMATIC);
    pid2_modeIndex=1;
  }
  sendDash = true;
}
  
void ReceiveTunings()
{
  pid1_kp = double(foo.asFloat[0]);
  pid1_ki = double(foo.asFloat[1]);
  pid1_kd = double(foo.asFloat[2]);
    
  pid2_kp = double(foo.asFloat[3]);
  pid2_ki = double(foo.asFloat[4]);
  pid2_kd = double(foo.asFloat[5]);
  pid1_ctrlDirection = cmd_b0;
  pid2_ctrlDirection = cmd_b1;
  
  myPID1.SetTunings(pid1_kp, pid1_ki, pid1_kd);
  myPID2.SetTunings(pid2_kp, pid2_ki, pid2_kd);
  
  if(cmd_b0==0) myPID1.SetControllerDirection(DIRECT);
  else myPID1.SetControllerDirection(REVERSE);
  
  if(cmd_b1==0) myPID2.SetControllerDirection(DIRECT);
  else myPID2.SetControllerDirection(REVERSE);
  EEPROMBackupTunings();
  sendTune = true;
}

void ReceiveAtune()
{
	
  // PID 001					
  pid1_aTuneStep = foo.asFloat[0];
  pid1_aTuneNoise = foo.asFloat[1];
  pid1_aTuneLookBack = (unsigned int)foo.asFloat[2];
  
  // PID 002
  pid2_aTuneStep = foo.asFloat[3];
  pid2_aTuneNoise = foo.asFloat[4];  
  pid2_aTuneLookBack = (unsigned int)foo.asFloat[5];
  
  if((cmd_b0==0 && pid1_tuning) || (cmd_b0==1 && !pid1_tuning))
  { //toggle autotune state
    pid1_changeAutoTune();
  }
  if((cmd_b1==0 && pid2_tuning) || (cmd_b1==1 && !pid2_tuning))
  { //toggle autotune state
    pid2_changeAutoTune();
  }
  EEPROMBackupATune();
  sendAtune = true;   
}

void pid1_ReceiveProfile()
{
  cmd_b0=pid1_nProfSteps;
  pid1_receivingProfile=true;
  if(pid1_runningProfile)                          //stop the current profile execution
  {
    pid1_StopProfile();
  }
  if(!cmd_b0)                               // store profile receive start time
  {
    pid1_profReceiveStart = millis();
  }
  while(pid1_receivingProfile)
  {
    if((millis() - pid1_profReceiveStart) >= 1000) //there was a timeout issue.  reset this transfer
    {
      pid1_receivingProfile=false;
      Serial.println("ProfError");
      EEPROMRestoreProfile();
    }
    if(cmd_b0>=pid1_nProfSteps)                      // profile receive complete
    {
      pid1_receivingProfile=false;
      Serial.print("ProfDone ");              // acknowledge profile recieve completed
      EEPROMBackupProfile();
      Serial.println("Archived");             // acknowledge profile stored
    }
    else                                      // read in profile step values
    {
      pid1_profVals[cmd_b0] = foo.asFloat[0];
      pid1_profTimes[cmd_b0] = (unsigned long)(foo.asFloat[1] * 1000);
      Serial.print("ProfAck ");              // request next profile step values
    }
    
    byte index=0;                                 // read in next profile step bytes
    byte cmd_b0 = Serial.read();
    while (Serial.available())
    {
      byte cmd_b2 = Serial.read();
      foo.asBytes[index] = cmd_b2;
      index++;
    }
  }
}

void pid2_ReceiveProfile()
{
  cmd_b1=pid2_nProfSteps;
  pid2_receivingProfile=true;
  if(pid2_runningProfile)                          //stop the current profile execution
  {
    pid2_StopProfile();
  }
  while(pid2_receivingProfile)
  {
    if((millis() - pid2_profReceiveStart) >= 1000) //there was a timeout issue.  reset this transfer
    {
      pid2_receivingProfile=false;
      Serial.println("ProfError");
      EEPROMRestoreProfile();
    }
    if(cmd_b1==0)                               // store profile receive start time
    {
      pid2_profReceiveStart = millis();
    }
    if(cmd_b1>=pid2_nProfSteps)                      // profile receive complete
    {
      pid2_receivingProfile=false;
      Serial.print("ProfDone ");              // acknowledge profile receive completed
      EEPROMBackupProfile();
      Serial.println("Archived");             // acknowledge profile stored
    }
    else                                      // read in profile step values
    {
      pid2_profVals[cmd_b1] = foo.asFloat[0];
      pid2_profTimes[cmd_b1] = (unsigned long)(foo.asFloat[1] * 1000);
      Serial.print("ProfAck ");              // request next profile step values
    }
    
    byte index=0;                                 // read in next profile step bytes
    byte cmd_b1 = Serial.read();
    while (Serial.available())
    {
      byte cmd_b2 = Serial.read();
      foo.asBytes[index] = cmd_b2;
      index++;
    }
  }
}

void ProfileCommand()
{
  if(!cmd_b0 && !pid1_runningProfile) pid1_StartProfile();
  if(cmd_b0 && pid1_runningProfile) pid1_StopProfile();
  if(!cmd_b1 && !pid2_runningProfile) pid2_StartProfile();
  if(cmd_b1 && pid2_runningProfile) pid2_StopProfile();
}


// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?

void SerialSend()
{
  if(sendInfo)
  {//just send out the stock identifier
    Serial.print("\nDual osPID v1.0");
    Serial.println("");
    sendInfo = false; //only need to send this info once per request
  }
  if(sendDash)
  {
    // Header
    Serial.print("DASH"); // 0
    Serial.print(" ");
    // PID 001
    //*************************************
    Serial.print(pid1_setpoint);  //1                
    Serial.print(" "); 
    if(isnan(pid1_input)) Serial.print(pid1_error);  
    else Serial.print(pid1_input); // 2
    Serial.print(" ");                
    Serial.print(pid1_output); // 3                 
    Serial.print(" ");
    Serial.print(myPID1.GetMode()); // 4
    Serial.print(" ");
    
    // PID 002
    //**************************************
    Serial.print(pid2_setpoint); // 5
    Serial.print(" ");
    if(isnan(pid2_input)) Serial.print(pid2_error);
    else Serial.print(pid2_input); // 6
    Serial.print(" ");
    Serial.print(pid2_output); // 7
    Serial.print(" ");    
    Serial.print(myPID2.GetMode()); // 8
    Serial.print(" ");
    
    //Raw Sensor Readings
    //*************************************
    Serial.print(tempcheckSensor1); // 9
    Serial.print(" ");
    Serial.print(tempcheckSensor2); // 10
    Serial.print(" ");
    Serial.print(tempcheckSensor3); // 11
    Serial.print(" ");
    Serial.print(tempcheckSensor4); // 12
    Serial.print(" ");
    Serial.print(tempcheckSensor5); // 13
    Serial.print(" ");
    Serial.print(tempcheckSensor6); // 14
    Serial.print(" ");
    
    // Ack
    //*************************************
    Serial.println(ackDash?1:0);    // 15
   // Serial.print(" ");
    if(sendDash)sendDash=false;
  }
  if(sendTune)
  {
    // header
    //*************************************
    Serial.print("TUNE "); // 0
    
    // PID 001
    //*************************************
    Serial.print(myPID1.GetKp()); // 1
    Serial.print(" ");
    Serial.print(myPID1.GetKi()); // 2
    Serial.print(" ");
    Serial.print(myPID1.GetKd()); // 3
    Serial.print(" ");
    Serial.print(myPID1.GetDirection()); // 4
    Serial.print(" ");
    Serial.print(pid1_tuning?1:0); // 5
    Serial.print(" ");
    
    // PID 002
    //*************************************
    Serial.print(myPID2.GetKp()); // 6
    Serial.print(" ");
    Serial.print(myPID2.GetKi()); // 7
    Serial.print(" ");
    Serial.print(myPID2.GetKd()); // 8
    Serial.print(" ");
    Serial.print(myPID2.GetDirection()); // 9
    Serial.print(" ");
    Serial.print(pid2_tuning?1:0); // 10
    Serial.print(" ");
           
    if(sendTune)sendTune=false;
  }
  if(sendAtune)
  {
    // PID 001
    //*************************************
    Serial.print(pid1_aTuneStep); // 11
    Serial.print(" ");
    Serial.print(pid1_aTuneNoise); // 12
    Serial.print(" ");
    Serial.print(pid1_aTuneLookBack); // 13
    Serial.print(" ");
    
    // PID 002
    //*************************************
    Serial.print(pid2_aTuneStep); // 14
    Serial.print(" ");
    Serial.print(pid2_aTuneNoise); // 15
    Serial.print(" ");
    Serial.print(pid2_aTuneLookBack); // 16
    Serial.print(" ");
    
    // ACK
    //*************************************
    Serial.println(ackTune?1:0); // 17
    if(sendAtune)sendAtune=false;
  }
  if(pid1_runningProfile)
  {
    Serial.print("PID1PROF ");
    Serial.print(int(pid1_curProfStep));
    Serial.print(" ");
    Serial.print(int(pid1_curType));
    Serial.print(" ");
  
  switch(pid1_curType)
  {
    case 1: //ramp
      Serial.println((pid1_helperTime-now)); //time remaining
      break;
    case 2: //wait
      Serial.print(abs(pid1_input-pid1_setpoint));
      Serial.print(" ");
      Serial.println(pid1_curVal==0? -1 : float(now-pid1_helperTime));
      break;  
    case 3: //step
      Serial.println(pid1_curTime-(now-pid1_helperTime));
      break;
    default: 
      break;
  }
  }  
  if(pid2_runningProfile)
  {
    Serial.print("PID2PROF ");
    Serial.print(int(pid2_curProfStep));
    Serial.print(" ");
    Serial.print(int(pid2_curType));
    Serial.print(" ");
  
  switch(pid2_curType)
  {
    case 1: //ramp
      Serial.println((pid2_helperTime-now)); //time remaining
      break;
    case 2: //wait
      Serial.print(abs(pid2_input-pid2_setpoint));
      Serial.print(" ");
      Serial.println(pid2_curVal==0? -1 : float(now-pid2_helperTime));
      break;  
    case 3: //step
      Serial.println(pid2_curTime-(now-pid2_helperTime));
      break;
    default: 
      break;    
      
  }
}
}







