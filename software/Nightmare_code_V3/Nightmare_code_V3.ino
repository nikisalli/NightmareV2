#include "N_Octapod.h"
#include "N_hc12.h"

TaskHandle_t Task3;
lp_filter f1(0.1);
lp_filter f2(0.1);
lp_filter f3(0.1);
lp_filter f4(0.1);
bool activated;

void setup() {
  hc12::init();
  xTaskCreatePinnedToCore(Task3code, "Task3", 10000, NULL, 0, &Task3, 0);
  Nightmare::init();
  Nightmare::startPanda();
  disableCore0WDT();
  disableCore1WDT();
}

void loop() {
  Nightmare::readBody();
  delay(50);
  /*if (hc12::active()) {
    if (!activated) {
      Nightmare::standUp();
      activated = true;
    }
    switch (hc12::state()) {
      case 0: {
          if (hc12::speed1() > 10 or hc12::speed2() > 10) {
            Nightmare::step(hc12::state1(), 6, Nightmare::speed);
          } else {
            Nightmare::stand();
          }
        } break; 
      case 1: {
          int dir[4];

          dir[0] = hc12::speed1() * sin(toRad(hc12::angle1()));
          dir[1] = hc12::speed1() * cos(toRad(hc12::angle1()));
          dir[2] = hc12::speed2() * sin(toRad(hc12::angle2()));
          dir[3] = hc12::speed2() * cos(toRad(hc12::angle2()));

          f1.set_input(dir[0]);
          f2.set_input(dir[1]);
          f3.set_input(dir[2]);
          f4.set_input(dir[3]);

          float alpha = limit(fmap(f1.get_val(), -100, 100, -10, 10), -10, 10);
          float beta = limit(fmap(f2.get_val(), -100, 100, -10, 10), -10, 10);
          float x_move = limit(fmap(f3.get_val(), -100, 100, 8, -8), -8, 8);
          float y_move = limit(fmap(f4.get_val(), -100, 100, 8, -8), -8, 8);

          Nightmare::move(alpha, beta, 0, x_move, y_move, 0);
          delay(5);
        } break;
      case 2: {
          int dir[4];

          dir[0] = fmap(hc12.speed1_ * sin(toRad(hc12.angle1_)),-100,100,-500,500);
          dir[1] = fmap(hc12.speed1_ * cos(toRad(hc12.angle1_)),-100,100,-500,500);
          dir[2] = fmap(hc12.speed2_ * sin(toRad(hc12.angle2_)),-100,100,-500,500);
          dir[3] = fmap(hc12.speed2_ * cos(toRad(hc12.angle2_)),-100,100,-500,500);

          Nightmare.moveleg(dir[0], dir[1], dir[2], dir[3]);
          delay(10);
        } break;
    }
  }
  else {
    if (activated) {
      Nightmare::sit();
      activated = false;
    }
    Nightmare::readBody();
    delay(50);
  }*/
}

void Task3code( void * parameter) {
  for (;;) {
    if(hc12::receive()){
      if (hc12::state() == 0) {
        float x1 = (hc12::speed1() * cos(toRad(hc12::angle1())))/12;
        float y1 = (hc12::speed1() * sin(toRad(hc12::angle1())))/12;
        float x2 = (hc12::speed2() * cos(toRad(hc12::angle2())))/6;
        float y2 =  hc12::speed2() * sin(toRad(hc12::angle2()));
        Nightmare::y_step = x1;
        Nightmare::x_step = y1;
        if(x1+y1>10){
          if(x2 > 0){
            Nightmare::angle = x2-(((x1+y1)-5)*10);
            if(Nightmare::angle < 0){
              Nightmare::angle = 0;
            }
          }
          else{
            Nightmare::angle = x2+(((x1+y1)-5)*10);
            if(Nightmare::angle > 0){
              Nightmare::angle = 0;
            }
          }
        }
        else{
          Nightmare::angle = x2;
        }
        Nightmare::angle = -Nightmare::angle;
        float speed_ = (hc12::speed1() > hc12::speed2() ? hc12::speed1() : hc12::speed2()) / 10.0;
        if(speed_ < 1.0){
          speed_ = 5.0;
        }
        Nightmare::speed = ((1/(speed_ - 0.5)) * 2) + 0.22;
      }
      hc12::write();
    }
  }
}
