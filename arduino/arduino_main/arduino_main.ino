
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

int pullUpPins[] = {18, 22, 23, 24, 25, 26, 27};
int outputPins[] = {3, 4, 9, 10, 12, 16, 17, 19};

const int is_mega = LOW;

int indicatorState = LOW;

//Speedometer
const int speedometerPin = A2;
const int circumference = 1000; //in mm
int maxSpeedCounter = 100;//min time (in ms) of one rotation (for debouncing)
int speedCounter;
int speedVal;
long timer = 0;// time between one full rotation (in ms)
float speedKMH = 0.00;


//Subscriber
void messageCb( const sdc_msgs::state& data){

  //-----------motor controller-----------


  digitalWrite(12, data.enableMotor);
  
  //direction
  if (data.direction > 0 && data.enableMotor){
    //forward
    digitalWrite(10, LOW); //forward pin
    digitalWrite(9, HIGH); //backward pin
  }
  else if(data.direction < 0 && data.enableMotor){
    //backward
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
  }
  else{
    digitalWrite(10, HIGH);
    digitalWrite(9, HIGH);
  }

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
  analogWrite(3, duty);
  analogWrite(4, duty);
  

  //----------------END-------------------

  //---------------LIGHTS-----------------

  indicate = data.indicate;
  digitalWrite(19, data.light);

  //----------------END-------------------
}

ros::Subscriber<sdc_msgs::state> sub("state", &messageCb );

void setup()
{ 
  //Pullup IN
  for (int i = 0; i < sizeof(pullUpPins); i++){
    pinMode(pullUpPins[i], INPUT_PULLUP);
  }
  //Pinput 
  pinMode(speedometerPin, INPUT);

  //Output
  for (int i = 0; i < sizeof(outputPins); i++){
    pinMode(outputPins[i], OUTPUT);
  }
  
  //speedometer
  speedCounter = maxSpeedCounter;
  
  //Source: https://www.instructables.com/id/Arduino-Bike-Speedometer/
  // TIMER SETUP- the timer interrupt allows preceise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1
  cli();//stop interrupts

  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;
  // set timer count for 1khz increments
  OCR1A = 1999;// = (1/1000) / ((1/(16*10^6))*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
  //END TIMER SETUP

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
//speedomerter 
ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 1kHz to measure reed switch
  speedVal = digitalRead(speedometerPin);//get val of A3
  if (speedVal){//if reed switch is closed
    if (speedCounter == 0){//min time between pulses has passed
      speedKMH = (3600*float(circumference))/float(timer);//calculate km per hour
      timer = 0;//reset timer
      speedCounter = maxSpeedCounter;//reset speedCounter
    }
    else{
      if (speedCounter > 0){//don't let speedCounter go negative
        speedCounter -= 1;//decrement speedCounter
      }
    }
  }
  else{//if reed switch is open
    if (speedCounter > 0){//don't let speedCounter go negative
      speedCounter -= 1;//decrement speedCounter
    }
  }
  if (timer > 2000){
    speedKMH = 0;//if no new pulses from reed switch- tire is still, set mph to 0
  }
  else{
    timer += 1;//increment timer
  } 
}

void loop()
{
  unsigned long currentMillis = millis();
  
  //Analog IN
  for(int i = 0; i < 5; i++){
    arduinoIn_msg.analog[i] = analogRead(i);
  }

  //Digital IN
  for (int i = 0; i < sizeof(pullUpPins); i++){
    arduinoIn_msg.digital[pullUpPins[i]] = digitalRead(pullUpPins[i]);
  }
  
  
  if (indicate == "Left"){
    digitalWrite(16, indicatorState);
    digitalWrite(17, LOW);
  }
  else if (indicate == "Right"){
    digitalWrite(16, LOW);
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

 
  //indicator
  if(currentMillis - indicateMillis > 500){
    indicateMillis = millis();
    if (indicatorState == LOW)
      indicatorState = HIGH;
    else
      indicatorState = LOW;
  }

  arduinoIn_msg.analog[6] = speedKMH;

  //timer so serial doesn't get overloaded
  if(currentMillis - previousMillis > 20){
    previousMillis = millis();
    p.publish(&arduinoIn_msg);
  }
  
  nh.spinOnce();
}