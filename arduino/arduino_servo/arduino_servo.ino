#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <sdc_msgs/arduinoIn.h>
#include <sdc_msgs/state.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 1

//pin def
const int DIR = 8;
const int STEP = 9;
const int ENABLE = 13;

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

//dimentions in steps
int curPos = 0;
int goalPos = 0;
int moveSteps = 0;
const int minPos = -360; 
const int maxPos = 360;  
 
sdc_msgs::state starte_msg;
String mode = "";

//Subscriber
void messageCb( const sdc_msgs::state& data){
  //steering
  mode = data.mode;
  if (mode == "remote" || data.mode == "cruise") {
    stepper.enable();
    goalPos = map(data.steeringAngle, -100, 100, minPos, maxPos);
    moveSteps = goalPos - curPos;
    stepper.rotate(moveSteps);
    curPos += moveSteps;
  }
  else{
    stepper.disable();
  }
    
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb );

void setup()
{ 
  
  nh.getHardware()->setBaud(57600);
  Serial.begin(57600);
  nh.initNode();

  nh.subscribe(sub);
 

  stepper.begin(RPM, MICROSTEPS);
  stepper.disable();
}



void loop()
{
  nh.spinOnce();
}
