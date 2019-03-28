#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "BasicStepperDriver.h"
#include "SPI.h"

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sdc_msgs/state.h>
#include <std_msgs/String.h>
#include <sdc_msgs/arduinoIn.h>

ros::NodeHandle nh;

sdc_msgs::state state_msg;
sdc_msgs::arduinoIn arduinoIn_msg;

ros::Publisher p("/servo/in", &arduinoIn_msg);

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 1
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 1000

//pin def
const int DIR = 5;
const int STEP = 3;
const int ENABLE = 6;

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

unsigned long previousMillis = 0;
unsigned long indicateMillis = 0;

//encoder vars
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
long encoderPos = 0;

//rotary dimentions in steps
int goalPos = 0;
int prevGoalPos = 0;
int remainingSteps = 0;
int moveSteps = 0;
const int minPos = -700; 
const int maxPos = 700; 


const int zeroPos = 248; 
boolean enableSteering = false;
boolean prevEnableSteering = false;

//Subscriber callback
void messageCb( const sdc_msgs::state& data){
  //steering
  enableSteering = data.enableSteering;
  goalPos = map(data.steeringAngle, -100, 100, minPos, maxPos);
}

void getPos(){
  //get curent positon  of encoder
  int zero1 = SPI.transfer(0x00); //zero
  int zero2 = SPI.transfer(0x08); //zero + default zero point
  zeroPoint = zero2 & 0B00000001; //1 = Factory Default
  
  int flags = SPI.transfer(0x10);     //Position Valid + Positon Syncronized
  posSync = flags >> 6 & 0B00000001;  //1 = trigggerd by SPI
  posValid = flags >> 7;              //1 = Valid

  int pos1 = SPI.transfer(0x18);
  int pos2 = SPI.transfer(0x20);
  
  crc = SPI.transfer(0x28);
  //crc = SPI.transfer(0x30);
  actualCrc = crc & 0B01111111;
  staleData = crc >> 7;

  /*
  crcCheck = zeroPoint;
  crcCheck = crcCheck * 256 + flags;
  crcCheck = crcCheck * 256 + pos1;
  crcCheck = crcCheck * 256 + pos2;
  crcCheck = crcCheck * 256 + crc;
  crcCheck = crcCheck >> 7;*/
  
  result = pos1 * 256 + pos2;

  /*
  if(abs(result - lastResult) > 1000){
    turns --;
  }
  else if(abs(lastResult - result) > 1000)
  {
    turns ++;
  }*/
  
  arduinoIn_msg.analog[0] = zero1;
  arduinoIn_msg.analog[1] = zero2;
  arduinoIn_msg.analog[2] = flags;
  arduinoIn_msg.analog[3] = pos1;
  arduinoIn_msg.analog[4] = pos2;
  arduinoIn_msg.analog[5] = crc;
  arduinoIn_msg.analog[6] = result;

  //TODO: crc check
  if(posValid){
    encoderPos = result + turns * 1023 - zeroPos;
  }
  else{
    encoderPos = curPos;
  }
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb);

void setup()
{ 
  
  nh.getHardware()->setBaud(57600);
  Serial.begin(57600);
  nh.initNode();

  nh.subscribe(sub);
 
  //setup Stepper
  stepper.begin(RPM, MICROSTEPS);
  stepper.disable();
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  //pinMode(8, INPUT);

  //SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  //getPos();
  //zeroPos = curPos;
  nh.advertise(p);
}



void loop()
{
  if (nh.connected()){
  unsigned long currentMillis = millis();

  if(enableSteering && prevEnableSteering != enableSteering){
    //curPos = map(encoderPos - zeroPos, 0, 1023, 0, 700);  
  }

  if(enableSteering) {
    stepper.enable();
    if (abs(goalPos - prevGoalPos) > 50){
      //stop move
      remainingSteps = stepper.stop();
      curPos -= remainingSteps;
    }
    moveSteps = goalPos - curPos;
    stepper.move(moveSteps);

    //stepper.getStepsCompleted();
    curPos += moveSteps;
    prevGoalPos = goalPos;
  }
  else{
    stepper.disable();
  }

  
  getPos();
  //arduinoIn_msg.analog[0] = curPos;
  //arduinoIn_msg.analog[1] = encoderPos;

  if(currentMillis - previousMillis > 20){
    previousMillis = millis();
    p.publish(&arduinoIn_msg);
  }
  }else{
    stepper.disable();
  }
  nh.spinOnce();
}
