#include "N_Servos.h"
#include "N_defines.h"
#include "N_Octapod.h"
#include "N_math.h"
#include "N_structs.h"

bool active_servo[26] = {};
int variable_angles[26] = {};
bool written = false;
bool writing = false;

TaskHandle_t Task2;

void Task2code( void * parameter) {
  for (;;) {
    if(written){
      if (!writing){
        for (int i = 0; i < 24; i++) {
          if (active_servo[i]){
            servoMove((variable_angles[i]+SERVO_OFFSETS[i]),i+1);
          }
        }
      }
    }
    else{
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void servoInit(){
  Serial2.begin(115200);
  Serial2.setRxBufferSize(1024);
  
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);

  for(int i=0;i<26;i++){
    active_servo[i] = true;
  }
  
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
}

void bodyToServos(float pos[][8]){
  float lol[24];
  struct Dleg * dlegs = Nightmare::dlegs;
  for(int i=0;i<4;i++){
     trigz(lol[3*i],lol[3*i+1],lol[3*i+2],-(pos[0][i]-LEG_START_OFFSET[i*2]),-(pos[1][i]-LEG_START_OFFSET[i*2+1]),pos[2][i],dlegs[i]);
     dlegs[i].CX_ANGLE=lol[3*i]+SERVO_OFFSETS[3*i];
     dlegs[i].FM_ANGLE=lol[3*i+1]+SERVO_OFFSETS[3*i+1];
     dlegs[i].TB_ANGLE=lol[3*i+2]+SERVO_OFFSETS[3*i+2];
  }
  for(int i=4;i<8;i++){
     trigz(lol[3*i],lol[3*i+1],lol[3*i+2],-(pos[0][i]-LEG_START_OFFSET[i*2]),pos[1][i]-LEG_START_OFFSET[i*2+1],pos[2][i],dlegs[i]);
     dlegs[i].CX_ANGLE=lol[3*i]+SERVO_OFFSETS[3*i];
     dlegs[i].FM_ANGLE=lol[3*i+1]+SERVO_OFFSETS[3*i+1];
     dlegs[i].TB_ANGLE=lol[3*i+2]+SERVO_OFFSETS[3*i+2];
  }
  servoWrite(lol);
}

void servoWrite(float angles[]){
  for (int i = 0; i < 24; i++) {
    written = true;
    variable_angles[i] = angles[i];
  }
}

void bodyDetach(){
  writing = true;
  for(int i=0;i<26;i++){
    if(active_servo[i]){
      servoDetach(i);
      active_servo[i] = false;
    }
  }
  writing = false;
}

void bodyAttach(){
  writing = true;
  for(int i=0;i<26;i++){
    if(!active_servo[i]){
      servoAttach(i);
      active_servo[i] = true;
    }
  }
  writing = false;
}

#define SERVO_HEADER            0x55,  0x55
#define SERVO_MOVE_TIME_WRITE       7,  1
#define SERVO_MOVE_TIME_READ        3,  2
#define SERVO_MOVE_TIME_WAIT_WRITE  7,  7
#define SERVO_MOVE_TIME_WAIT_READ   3,  8
#define SERVO_MOVE_START            3, 11
#define SERVO_MOVE_STOP             3, 12
#define SERVO_ID_WRITE              4, 13
#define SERVO_ID_READ               3, 14
#define SERVO_ANGLE_OFFSET_ADJUST   4, 17
#define SERVO_ANGLE_OFFSET_WRITE    3, 18
#define SERVO_ANGLE_OFFSET_READ     3, 19
#define SERVO_ANGLE_LIMIT_WRITE     7, 20
#define SERVO_ANGLE_LIMIT_READ      3, 21
#define SERVO_VIN_LIMIT_WRITE       7, 22
#define SERVO_VIN_LIMIT_READ        3, 23
#define SERVO_TEMP_MAX_LIMIT_WRITE  4, 24
#define SERVO_TEMP_MAX_LIMIT_READ   3, 25
#define SERVO_TEMP_READ             3, 26
#define SERVO_VIN_READ              3, 27
#define SERVO_POS_READ              3, 28
#define SERVO_OR_MOTOR_MODE_WRITE   7, 29
#define SERVO_OR_MOTOR_MODE_READ    3, 30
#define SERVO_LOAD_OR_UNLOAD_WRITE  4, 31
#define SERVO_LOAD_OR_UNLOAD_READ   3, 32
#define SERVO_LED_CTRL_WRITE        4, 33
#define SERVO_LED_CTRL_READ         3, 34
#define SERVO_LED_ERROR_WRITE       4, 35
#define SERVO_LED_ERROR_READ        3, 36

template<typename T>
void servo_write(T t) {
  Serial2.write(t);
}

template<typename T, typename... Args>
void servo_write(T t, Args... args) {
  servo_write(t);
  servo_write(args...);
}

void servo_tx_enb() {
  digitalWrite(SERVO_PIN_TX_ENB,HIGH);
  digitalWrite(SERVO_PIN_RX_ENB,LOW);
}

void servo_rx_enb() {
  digitalWrite(SERVO_PIN_TX_ENB,LOW);
  digitalWrite(SERVO_PIN_RX_ENB,HIGH);
}

void servoMove(int ang, byte id){
  servo_tx_enb();
  
  ang = limit(map(ang, -120, 120, 0, 1000),0,1000);
  
  servo_write(
    SERVO_HEADER, 
    id, 
    SERVO_MOVE_TIME_WRITE, 
    ang & 0xFF, 
    ang >> 8, 
    0x00, 0x00, 
    ~((byte)(id+0x08+(ang & 0xFF)+(ang >> 8)))
  );
  
  //delayMicroseconds(1100);
  //Serial2.flush();
  servo_rx_enb();
}

void servoAttach(byte id){
  active_servo[id] = true;
  servo_tx_enb();
  
  servo_write(
    SERVO_HEADER,
    id,
    SERVO_LOAD_OR_UNLOAD_WRITE,
    0x01,
    ~((byte)(id+0x24))
  );
  
  Serial2.flush();
  servo_rx_enb();
}

void servoDetach(byte id){
  active_servo[id] = false;
  servo_tx_enb();
  
  servo_write(
    SERVO_HEADER,
    id,
    SERVO_LOAD_OR_UNLOAD_WRITE,
    0x00,
    ~((byte)(id+0x23))
  );

  Serial2.flush();
  servo_rx_enb();
}

int servoReadPos(byte id){
  servo_tx_enb();

  servo_write(
    SERVO_HEADER,
    id,
    SERVO_POS_READ,
    ~((byte)(id+0x1F))
  );

  Serial2.flush();
  servo_rx_enb();

  unsigned long time = micros();
  while(!Serial2.available()){
    if((micros()-time)>3000){
      return 4000;
    }
  }
  byte tentatives = 0;
  while(Serial2.read() != 0x1C){
    tentatives++;
    if(tentatives>10){
      return 4000;
    }
  }
  byte low = Serial2.read();
  byte high = Serial2.read();
  byte checksum = Serial2.read();
  byte checksum_ = (~((byte)(id+0x21+low+high)));
  if(checksum == checksum_){
    return fmap((low|(high<<8)),0,1000,-120,120);
  }
}

