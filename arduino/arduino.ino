//#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <ros.h>

/* 
 * rosserial ADC Example
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <sdc_msgs/arduinoIn.h>
#include <sdc_msgs/state.h>

ros::NodeHandle nh;

//Publisher
sdc_msgs::arduinoIn arduinoIn_msg;
ros::Publisher p("/arduino/in", &arduinoIn_msg);

//Subscriber
void messageCb( const sdc_msgs::state& data){
  
  //throttle
  int throttle = 0;
  if (data.enableMotor){
    if(data.direction < 0){
      throttle = int(data.throttle/2);
    }
    else{
      throttle = data.throttle;
    }
  }
  else{
    throttle = 0;
  }
  
  int duty = map(throttle, 0, 100, 0, 255);
  analogWrite(10, duty);
  analogWrite(11, duty);
  
  //direction
  if (data.direction > 0 && data.enableMotor){
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
  }
  if(data.direction < 0 && data.enableMotor){
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);
  }
  /*else{
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
  }*/
  
  //TODO: Steering
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb );


void setup()
{ 
  //PWM
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  //Pullup
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  
  //Digital OUT
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  
  nh.initNode();

  nh.advertise(p);
  nh.subscribe(sub);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;

void loop()
{
  arduinoIn_msg.adc0 = averageAnalog(0);
  arduinoIn_msg.adc1 = averageAnalog(1);
  arduinoIn_msg.adc2 = averageAnalog(2);
  arduinoIn_msg.adc3 = averageAnalog(3);
  arduinoIn_msg.adc4 = averageAnalog(4);
  arduinoIn_msg.adc5 = averageAnalog(5);
  
  //Digital IN
  arduinoIn_msg.D2 = digitalRead(2);
  arduinoIn_msg.D3 = digitalRead(3);
  arduinoIn_msg.D4 = digitalRead(4);
  arduinoIn_msg.D7 = digitalRead(7);
  arduinoIn_msg.D8 = digitalRead(8);
    
  p.publish(&arduinoIn_msg);

  nh.spinOnce();
}

