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
  int duty = data.throttle;
  duty = map(duty, 0, 100, 0, 255);
  analogWrite(11, duty);
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb );


void setup()
{ 
  pinMode(11, OUTPUT);
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
    
  p.publish(&arduinoIn_msg);

  nh.spinOnce();
}

