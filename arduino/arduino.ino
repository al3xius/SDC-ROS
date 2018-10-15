
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
#include <std_msgs/String.h>

ros::NodeHandle nh;
//Wait for connection
//while(!nh.connected()) {nh.spinOnce();}


//Publisher
sdc_msgs::arduinoIn arduinoIn_msg;
sdc_msgs::state starte_msg;
String indicate = "";

ros::Publisher p("/arduino/in", &arduinoIn_msg);

unsigned long previousMillis = 0;
unsigned long indicateMillis = 0;

int pullUpPins[] = {2, 3, 4, 7, 8, 14, 15, 18};
int outputPins[] = {10, 11, 13, 16, 17, 19};

const int is_mega = LOW;

int indicatorState = LOW;


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
  else if(data.direction < 0 && data.enableMotor){
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);
  }
  else{
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
  }

  indicate = data.indicate;

  if (is_mega){
    //light
    if (data.light){
      digitalWrite(19, HIGH);
    }
    else{
      digitalWrite(19, LOW);
    }

  }
  //TODO: Steering
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb );

void setup()
{ 
  //Pullup IN
  for (int i = 0; i < sizeof(pullUpPins); i++){
    if(pullUpPins[i] <= 13 || is_mega == 'mega'){
      pinMode(pullUpPins[i], INPUT_PULLUP);
    }
  }

  //Output
  for (int i = 0; i < sizeof(outputPins); i++){
    if(outputPins[i] <= 13 || is_mega == 'mega'){
      pinMode(outputPins[i], OUTPUT);
    }
  }
  

  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(p);
  int last_milli = millis();
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
  unsigned long currentMillis = millis();
  
  //Analog IN
  for(int i = 0; i < 5; i++){
    arduinoIn_msg.analog[i] = analogRead(i);
  }

  //Digital IN
  for (int i = 0; i < sizeof(pullUpPins); i++){
    if((pullUpPins[i] <= 13 || is_mega)){
      arduinoIn_msg.digital[pullUpPins[i]] = digitalRead(pullUpPins[i]);
    }
  }
  
  
  if (is_mega){
    if (indicate == "Left"){
      digitalWrite(16, indicatorState);
    }
    else if (indicate == "Right"){
      digitalWrite(17, indicatorState);
  
    }
    else if (indicate == "Both"){
      digitalWrite(16, indicatorState);
      digitalWrite(17, indicatorState);
    }
    else{
      digitalWrite(16, LOW);
      digitalWrite(17, LOW);
    }
    arduinoIn_msg.digital[50] = indicatorState;
  }
  
  //timer so serial doesn't get overloaded
  if(currentMillis - previousMillis > 20){
    previousMillis = millis();
    p.publish(&arduinoIn_msg);
  }
 
  //indicator
  if(currentMillis - indicateMillis > 500){
    indicateMillis = millis();
    if (indicatorState == LOW)
      indicatorState = HIGH;
    else
      indicatorState = LOW;
  }
  
  nh.spinOnce();
}

