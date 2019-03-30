#include <ros.h>
#include <sdc_msgs/state.h>
//#include <std_msgs/Int32.h>
#include "BasicStepperDriver.h"

//to Test
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 105> nh;

#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 1
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 1000

#define DIR 5
#define STEP 3
#define ENABLE 6

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);

boolean enableSteering = false;

int goalPos = 0;
int curPos = 0;
#define minPos -700
#define maxPos 700

void messageCb( const sdc_msgs::state& data){
  //steering
  enableSteering = data.enableSteering;
  goalPos = map(data.steeringAngle, -100, 100, minPos, maxPos);
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb);

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.subscribe(sub);

  //setup Stepper
  stepper.begin(RPM, MICROSTEPS);
  stepper.disable();
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
}

void loop() {
  if (nh.connected()){
    if(enableSteering){
      stepper.enable();
      stepper.move(goalPos - curPos);
      curPos += goalPos - curPos;
    }
    else{
      stepper.disable();
    }
  }
  else{
    if(curPos != 0){
      goalPos = 0;
      stepper.move(goalPos - curPos);
      curPos += goalPos - curPos;
    }
    stepper.disable();
  }
  nh.spinOnce();
}
