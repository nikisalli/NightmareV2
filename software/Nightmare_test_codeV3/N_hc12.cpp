#include "N_hc12.h"
#include "N_defines.h"
#include "N_sensors.h"

using namespace hc12;

int angle1_;
int speed1_;
int angle2_;
int speed2_;
int active_;
int state_;
int state1_;

const int hc12::angle1() { return angle1_; }
const int hc12::speed1() { return speed1_; }
const int hc12::angle2() { return angle2_; }
const int hc12::speed2() { return speed2_; }
const int hc12::active() { return active_; }
const int hc12::state()  { return state_;  }
const int hc12::state1() { return state1_; }

void hc12::init(){
  Serial1.begin(HC12_BAUD_RATE, SERIAL_8N1, HC12_PIN_RX, HC12_PIN_TX);
  Serial1.setRxBufferSize(20);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
}

bool hc12::receive(){
  if(Serial1.available()>10){
    byte bytes[8];
    unsigned long val = micros();
    while(Serial1.read()!=0x55){
      if((micros()-val)>2000){
        return 0;
      }
    }
    Serial1.read();
    for(byte& i : bytes){
      i = Serial1.read();
    }
    if(((byte)(~(bytes[0]+bytes[1]+bytes[2]+bytes[3]+bytes[4]+bytes[5]+bytes[6]))) == bytes[7]){
      angle1_ = bytes[0]*1.41176470588;
      angle2_ = bytes[2]*1.41176470588;
      speed1_ = bytes[1];
      speed2_ = bytes[3];
      active_ = bytes[4];
      state_  = bytes[5];
      state1_ = bytes[6];
      return 1;
    }
  }
  else{
    return 0;
  }
}

void hc12::write(){
  Serial1.println(
      '#' + String(((int)(read_battery_voltage()*100))/100.0)
    + '+' + String(((int)(read_battery_current()*100))/100.0) 
    + "+*");
}
