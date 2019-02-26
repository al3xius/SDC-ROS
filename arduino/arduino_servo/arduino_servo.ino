#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sdc_msgs/state.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

#include "BasicStepperDriver.h"
#include "SPI.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 1

//pin def
const int DIR = 4;
const int STEP = 2;
const int ENABLE = 5;

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

unsigned long previousMillis = 0;


//encoder
int crc = 0;

int zeroPoint = 0;
int posValid = 0;
int posSync = 0;
int staleData = 0;
int actualCrc = 0;

long result = 0;
long lastResult = 0;
int turns = 0;

long crcCheck = 0;
long curPos = 0;


//dimentions in steps
int goalPos = 0;
int moveSteps = 0;
const int minPos = -360; 
const int maxPos = 360; 
const int zeroPos = 0; 
 
sdc_msgs::state state_msg;
std_msgs::Int32 pos;




//Subscriber
void messageCb( const sdc_msgs::state& data){
  //steering
  //getPos();
  if (data.mode == "remote" || data.mode == "cruise") {
    stepper.enable();
    goalPos = map(data.steeringAngle, -100, 100, minPos, maxPos);
    moveSteps = goalPos - curPos;
    curPos += moveSteps;
    stepper.rotate(moveSteps);
  }
  else{
    stepper.disable();
  }
}

void getPos(){
  //TODO: Multi turn
  int zero1 = SPI.transfer(0x00); //zero
  int zero2 = SPI.transfer(0x08); //zero + default zero point
  zeroPoint = zero2 & 0B00000001; //1 = Factory Default
  
  int flags = SPI.transfer(0x10); //Position Valid + Positon Syncronized
  posSync = flags >> 6 & 0B00000001; //1 = trigggerd by SPI
  posValid = flags >> 7; //1 = Valid

  int pos1 = SPI.transfer(0x18);
  int pos2 = SPI.transfer(0x20);
  
  crc = SPI.transfer(0x28);
  actualCrc = crc & 0B01111111;
  staleData = crc >> 7;

  crcCheck = zeroPoint;
  crcCheck = crcCheck * 256 + flags;
  crcCheck = crcCheck * 256 + pos1;
  crcCheck = crcCheck * 256 + pos2;
  crcCheck = crcCheck * 256 + crc;
  crcCheck = crcCheck >> 7;
  
  result = pos1;
  result = result * 256 + pos2;

  if(abs(result - lastResult) > 1000){
    turns --;
  }
  else if(abs(lastResult-result) > 1000)
  {
    turns ++;
  }
  

  //TODO: crc check
  if(posValid){
    curPos = result + turns * 1023 - zeroPos;
  }
  else{
    curPos = curPos;
  }
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb);
//ros::Publisher p("/arduino/servo", &pos);

void setup()
{ 
  
  nh.getHardware()->setBaud(57600);
  Serial.begin(57600);
  nh.initNode();

  nh.subscribe(sub);
 
  stepper.begin(RPM, MICROSTEPS);
  stepper.disable();
  digitalWrite(5,HIGH);

  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
}



void loop()
{
  /*unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > 20){
    getPos();
    pos.data = curPos;
    previousMillis = millis();
    p.publish(&pos);
  }*/
  
  nh.spinOnce();
}
