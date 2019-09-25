#include "N_Octapod.h"
#include "N_hc12.h"

Octapod Nightmare;
remote hc12;
TaskHandle_t Task3;
lp_filter f1(0.1);
lp_filter f2(0.1);
lp_filter f3(0.1);
lp_filter f4(0.1);
bool activated;
int done;

void setup() {
  Serial.begin(115200);
  hc12.init();
  xTaskCreatePinnedToCore(Task3code, "Task3", 10000, NULL, 0, &Task3, 0);
  Nightmare.init();
  disableCore0WDT();
  disableCore1WDT();
}

void loop() {
  if (hc12.active_) {
    if (!activated) {
      Nightmare.standUp();
      activated = true;
    }
    switch (hc12.state_) {
      case 0: {
          if (hc12.speed1_ > 10 or hc12.speed2_ > 10) {
            Nightmare.step(2, 6, 1.3);
          } else {
            Nightmare.stand();
          }
        } break; 
      case 1: {
          int dir[4];

          dir[0] = hc12.speed1_ * sin(toRad(hc12.angle1_));
          dir[1] = hc12.speed1_ * cos(toRad(hc12.angle1_));
          dir[2] = hc12.speed2_ * sin(toRad(hc12.angle2_));
          dir[3] = hc12.speed2_ * cos(toRad(hc12.angle2_));

          f1.set_input(dir[0]);
          f2.set_input(dir[1]);
          f3.set_input(dir[2]);
          f4.set_input(dir[3]);

          float alpha = limit(fmap(f1.get_val(), -100, 100, -10, 10), -10, 10);
          float beta = limit(fmap(f2.get_val(), -100, 100, -10, 10), -10, 10);
          float x_move = limit(fmap(f3.get_val(), -100, 100, 8, -8), -8, 8);
          float y_move = limit(fmap(f4.get_val(), -100, 100, 8, -8), -8, 8);

          Nightmare.move(alpha, beta, 0, x_move, y_move, 0);
          delay(5);
        } break;
    }
  }
  else {
    if (activated) {
      Nightmare.sit();
      activated = false;
    }
  }
}

void Task3code( void * parameter) {
  for (;;) {
    if(hc12.receive()){
      if (hc12.state_ == 0) {
        float x1 = (hc12.speed1_ * cos(toRad(hc12.angle1_)))/15;
        float y1 = (hc12.speed1_ * sin(toRad(hc12.angle1_)))/15;
        float x2 = (hc12.speed2_ * cos(toRad(hc12.angle2_)))/6;
        float y2 = hc12.speed2_ * sin(toRad(hc12.angle2_));
        Nightmare.y_step = x1;
        Nightmare.x_step = y1;
        if(x1+y1>10){
          if(Nightmare.angle > 0){
            Nightmare.angle = x2-(((x1+y1)-10)*4);
          }
          else{
            Nightmare.angle = x2+(((x1+y1)-10)*4);
          }
        }
        else{
          Nightmare.angle = x2;
        }
      }
      hc12.write();
    }
  }
}
