/* 
 * rosserial ADC Example
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

//Publisher
rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);

//Subscriber
void messageCb( const geometry_msgs::Twist& data){
  int duty = data.linear.x;
  duty = map(duty, 0, 100, 0, 255);
  analogWrite(11, duty);
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", &messageCb );


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
  adc_msg.adc0 = averageAnalog(0);
  adc_msg.adc1 = averageAnalog(1);
  adc_msg.adc2 = averageAnalog(2);
  adc_msg.adc3 = averageAnalog(3);
  adc_msg.adc4 = averageAnalog(4);
  adc_msg.adc5 = averageAnalog(5);
    
  p.publish(&adc_msg);

  nh.spinOnce();
}

