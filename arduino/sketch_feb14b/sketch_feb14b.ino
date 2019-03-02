#include "SPI.h"
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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
}

void loop() {
  // put your main code here, to run repeatedly:
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

  if(result - lastResult < -100){
    turns ++;
  }
  else if(result > 1000 && lastResult < 23)
  {
    turns --;
  }
  lastResult = result;

  //TODO: crc check
  if(posValid){
    curPos = result + turns*1023;
  }
  else{
    curPos = curPos;
  }
  //Serial.println(curPos);
  //Serial.println(result);
  Serial.println(turns);
  delay(10);
}
