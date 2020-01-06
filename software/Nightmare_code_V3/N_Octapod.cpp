#include "N_Octapod.h"
#include "Arduino.h"
#include "N_Servos.h"
#include "N_defines.h"
#include "N_math.h"
#include "N_structs.h"
#include "N_panda.h"

using namespace Nightmare;

// private
float pos[3][8] = {};
float bpos[3][2] = {};
float pbpos[3][8] = {};
const int tarantula[8] = {4, 2, 7, 1, 3, 5, 0, 6};
const int slow_gait[8] = {4, 2, 6, 0, 3, 5, 1, 7};
unsigned long timing;
boolean first_step = true;
byte leg;

// public
float Nightmare::x_step;
float Nightmare::y_step;
float Nightmare::angle;
float Nightmare::speed;
Dleg  Nightmare::dlegs[8];

void Nightmare::init() {
  for (int i = 0; i < 8; i++) {
    dlegs[i] = {
      LEG_DIMENSIONS[i * 3], 
      LEG_DIMENSIONS[i * 3 + 1], 
      LEG_DIMENSIONS[i * 3 + 2], 
      LEG_START_OFFSET[i * 2], 
      LEG_START_OFFSET[i * 2 + 1], 
      0,
      0,
      0,
      LEG_SIDE[i]
    };
    bodyAttach(); //REMOVE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  }

  pinMode(FAN1_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);
  pinMode(DEBUG_LED_PIN_1, OUTPUT);
  pinMode(DEBUG_LED_PIN_2, OUTPUT);

  digitalWrite(FAN1_PIN, HIGH);
  digitalWrite(FAN2_PIN, LOW);

  servoInit();
  
  pandaInit();
}

bool Nightmare::pandaIsOn() {
  return pandaIsOnline();
}

void Nightmare::startPanda() {
  pandaPowerOn();
}

void Nightmare::standUp() {
  Nightmare::readBody();
  bodyAttach();

  /*int l = millis() % 2;
  for (int k = 0; k < 4; k++) {
    int j = RANDOM_SEQ[l][k];
    int q = RANDOM_SEQ[l][k + 4];
    float p[3][4] = {
        {STAND_POS[0][j], STAND_POS[0][j], pos[0][j], pos[0][j]},
        {STAND_POS[1][j], STAND_POS[1][j], pos[1][j], pos[1][j]},
        {START_Z, START_Z + 4, pos[2][j] + 4, pos[2][j]},
    };
    float t[3][4] = {
        {STAND_POS[0][q], STAND_POS[0][q], pos[0][q], pos[0][q]},
        {STAND_POS[1][q], STAND_POS[1][q], pos[1][q], pos[1][q]},
        {START_Z, START_Z + 4, pos[2][q] + 4, pos[2][q]},
    };
    for (float i = 0; i <= 1; i += 0.01) {
      Spline3D(p, i, pos[0][j], pos[1][j], pos[2][j]);
      Spline3D(t, i, pos[0][q], pos[1][q], pos[2][q]);
      bodyToServos(pos);
      delay(3);
    }
    pos[0][j] = STAND_POS[0][j];
    pos[1][j] = STAND_POS[1][j];
    pos[2][j] = START_Z;
    pos[0][q] = STAND_POS[0][q];
    pos[1][q] = STAND_POS[1][q];
    pos[2][q] = START_Z;
  }*/

  delay(5);
  for (float j = 0; j < 1; j += 0.01) {
    for (int i = 0; i < 8; i++) {
      pos[0][i] = STAND_POS[0][i];
      pos[1][i] = STAND_POS[1][i];
      pos[2][i] = timeVal(j, STAND_Z, START_Z);
    }
    bodyToServos(pos);
    delay(20);
  }
}

void Nightmare::readBody() {
  for(int i=0;i<8;i++){
    int coxa_angle = servoReadPos(3*i+1);
    int femur_angle = servoReadPos(3*i+2);
    int tibia_angle = servoReadPos(3*i+3);
    /*if(coxa_angle != 4000 && femur_angle != 4000 && tibia_angle != 4000){
      if(i<4){
        float temp[3];
        ftrigz(temp[0], temp[1], temp[2], coxa_angle-SERVO_OFFSETS[3*i], femur_angle-SERVO_OFFSETS[3*i+1], tibia_angle-SERVO_OFFSETS[3*i+2], dlegs[i]);
        pos[0][i] = temp[0]+LEG_START_OFFSET[i*2];
        pos[1][i] = temp[1]+LEG_START_OFFSET[i*2+1];
        pos[2][i] = temp[2];
      }
      else{
        float temp[3];
        ftrigz(temp[0], temp[1], temp[2], -(coxa_angle-SERVO_OFFSETS[3*i]), femur_angle-SERVO_OFFSETS[3*i+1], tibia_angle-SERVO_OFFSETS[3*i+2], dlegs[i]);
        pos[0][i] = temp[0]+LEG_START_OFFSET[i*2];
        pos[1][i] = temp[1]+LEG_START_OFFSET[i*2+1];
        pos[2][i] = temp[2];
      }
    }*/
    if(coxa_angle != 4000){
      dlegs[i].CX_ANGLE = coxa_angle;
    }
    if(femur_angle != 4000){
      dlegs[i].FM_ANGLE = femur_angle;
    }
    if(tibia_angle != 4000){
      dlegs[i].TB_ANGLE = tibia_angle;
    }
  }
  //bodyToServos(pos);
}

void Nightmare::sit() {
  for (float j = 0; j < 1; j += 0.01) {
    for (int i = 0; i < 8; i++) {
      pos[0][i] = STAND_POS[0][i];
      pos[1][i] = STAND_POS[1][i];
      pos[2][i] = timeVal(j, START_Z, STAND_Z);
    }
    bodyToServos(pos);
    delay(20);
  }
  bodyDetach();
}

void Nightmare::stand() {
  first_step = true;
  if (!(compareMatrix(pos, STAND_POS))) {
    int l = millis() % 2;
    for (int k = 0; k < 4; k++) {
      int j = RANDOM_SEQ[l][k];
      int q = RANDOM_SEQ[l][k + 4];
      float p[3][4] = {
          {STAND_POS[0][j], STAND_POS[0][j], pos[0][j], pos[0][j]},
          {STAND_POS[1][j], STAND_POS[1][j], pos[1][j], pos[1][j]},
          {STAND_POS[2][j], STAND_POS[2][j] + 4, pos[2][j] + 4, pos[2][j]},
      };
      float t[3][4] = {
          {STAND_POS[0][q], STAND_POS[0][q], pos[0][q], pos[0][q]},
          {STAND_POS[1][q], STAND_POS[1][q], pos[1][q], pos[1][q]},
          {STAND_POS[2][q], STAND_POS[2][q] + 4, pos[2][q] + 4, pos[2][q]},
      };
      for (float i = 0; i <= 1; i += 0.01) {
        Spline3D(p, i, pos[0][j], pos[1][j], pos[2][j]);
        Spline3D(t, i, pos[0][q], pos[1][q], pos[2][q]);
        bodyToServos(pos);
        delay(3);
      }
      pos[0][j] = STAND_POS[0][j];
      pos[1][j] = STAND_POS[1][j];
      pos[2][j] = STAND_POS[2][j];
      pos[0][q] = STAND_POS[0][q];
      pos[1][q] = STAND_POS[1][q];
      pos[2][q] = STAND_POS[2][q];
    }
  } else {
    bodyToServos(pos);
  }
}

enum Gait {
  Tarantula,
  Scorpio,
  Ant,
};

void tarantula_gait(const float height, const float time);
void scorpio_gait(const float height, const float time);
void ant_gait(const float height, const float time);

void Nightmare::step(const int walk_pattern, const float height, const float time) {
  using walk_patterns = void (*)(float, float);
  constexpr walk_patterns wps[] { 
    [Gait::Tarantula] = tarantula_gait, 
    [Gait::Scorpio]   = scorpio_gait, 
    [Gait::Ant]       = ant_gait };
  (wps[walk_pattern])(height, time);
}

void tarantula_gait(const float height, const float time) {
  for (int a = 0; a < 8; a++) {
    const int fz = tarantula[a];
    const int pz = a == 0 ? tarantula[7] : tarantula[a-1];
    bpos[0][1] = bpos[0][0]; // p
    bpos[1][1] = bpos[1][0]; // p
    bpos[2][1] = bpos[2][0]; // p
    bpos[0][0] = pos[0][fz]; // f
    bpos[1][0] = pos[1][fz]; // f
    bpos[2][0] = pos[2][fz]; // f
    for (float b = 0; b < 1; b += 0.05) {
      for (int c = 0; c < 8; c++) {
        if (!first_step) {
          if (c != fz && c != pz) {
            RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(angle / 120),
                      0, 0, 0);
            Tmatrix(pos[0][c], pos[1][c], pos[2][c], -(x_step / 120),
                    -(y_step / 120), 0);
          }
        } else {
          if (c != fz) {
            RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(angle / 120),
                      0, 0, 0);
            Tmatrix(pos[0][c], pos[1][c], pos[2][c], -(x_step / 120),
                    -(y_step / 120), 0);
          }
        }
      }
      if (!first_step) {
        float cr[3] = {STAND_POS[0][pz], STAND_POS[1][pz], STAND_POS[2][pz]};
        RmatrixZ(cr[0], cr[1], cr[2], angle / 2.0, 0, 0, 0); // prev
        Tmatrix(cr[0], cr[1], cr[2], x_step / 2.0,
                y_step / 2.0, 0);
        float bm[3][4] = {
            // prev
            {cr[0], cr[0], bpos[0][1], bpos[0][1]},
            {cr[1], cr[1], bpos[1][1], bpos[1][1]},
            {STAND_POS[2][pz], STAND_POS[2][pz] + height,
              STAND_POS[2][pz] + height, STAND_POS[2][pz]},
        };
        Spline3D(bm, (b / 2.0) + 0.5, pos[0][pz], pos[1][pz],
                  pos[2][pz]); // prev
      }
      float br[3] = {STAND_POS[0][fz], STAND_POS[1][fz], STAND_POS[2][fz]};
      RmatrixZ(br[0], br[1], br[2], angle / 2.0, 0, 0, 0); // forw
      Tmatrix(br[0], br[1], br[2], x_step / 2.0,
              y_step / 2.0, 0);
      float am[3][4] = {
          // forw
          {br[0], br[0], bpos[0][0], bpos[0][0]},
          {br[1], br[1], bpos[1][0], bpos[1][0]},
          {STAND_POS[2][fz], STAND_POS[2][fz] + height,
            STAND_POS[2][fz] + height, STAND_POS[2][fz]},
      };
      Spline3D(am, b / 2.0, pos[0][fz], pos[1][fz], pos[2][fz]); // forw

      bodyToServos(pos);
      delayMicroseconds(speed * 18750);
    }
    first_step = false;
  }
}

void scorpio_gait(const float height, const float time) {
  const int slow_gait[] = {4, 2, 6, 0, 3, 5, 1, 7};
  for (int a = 0; a < 8; a++) {
    const int fz = slow_gait[a];
    const int pz = a == 0 ? slow_gait[7] : slow_gait[a-1];
    bpos[0][1] = bpos[0][0]; // p
    bpos[1][1] = bpos[1][0]; // p
    bpos[2][1] = bpos[2][0]; // p
    bpos[0][0] = pos[0][fz]; // f
    bpos[1][0] = pos[1][fz]; // f
    bpos[2][0] = pos[2][fz]; // f
    for (float b = 0; b < 1; b += 0.05) {
      for (int c = 0; c < 8; c++) {
        if (!first_step) {
          if (c != fz && c != pz) {
            RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(angle / 120),
                      0, 0, 0);
            Tmatrix(pos[0][c], pos[1][c], pos[2][c], -(x_step / 120),
                    -(y_step / 120), 0);
          }
        } else {
          if (c != fz) {
            RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(angle / 120),
                      0, 0, 0);
            Tmatrix(pos[0][c], pos[1][c], pos[2][c], -(x_step / 120),
                    -(y_step / 120), 0);
          }
        }
      }
      if (!first_step) {
        float cr[3] = {STAND_POS[0][pz], STAND_POS[1][pz], STAND_POS[2][pz]};
        RmatrixZ(cr[0], cr[1], cr[2], angle / 2.0, 0, 0, 0); // prev
        Tmatrix(cr[0], cr[1], cr[2], x_step / 2.0,
                y_step / 2.0, 0);
        float bm[3][4] = {
            // prev
            {cr[0], cr[0], bpos[0][1], bpos[0][1]},
            {cr[1], cr[1], bpos[1][1], bpos[1][1]},
            {STAND_POS[2][pz], STAND_POS[2][pz] + height,
              STAND_POS[2][pz] + height, STAND_POS[2][pz]},
        };
        Spline3D(bm, (b / 2.0) + 0.5, pos[0][pz], pos[1][pz],
                  pos[2][pz]); // prev
      }
      float br[3] = {STAND_POS[0][fz], STAND_POS[1][fz], STAND_POS[2][fz]};
      RmatrixZ(br[0], br[1], br[2], angle / 2.0, 0, 0, 0); // forw
      Tmatrix(br[0], br[1], br[2], x_step / 2.0,
              y_step / 2.0, 0);
      float am[3][4] = {
          // forw
          {br[0], br[0], bpos[0][0], bpos[0][0]},
          {br[1], br[1], bpos[1][0], bpos[1][0]},
          {STAND_POS[2][fz], STAND_POS[2][fz] + height,
            STAND_POS[2][fz] + height, STAND_POS[2][fz]},
      };
      Spline3D(am, b / 2.0, pos[0][fz], pos[1][fz], pos[2][fz]); // forw

      bodyToServos(pos);
      delayMicroseconds(speed * 18750);
    }
    first_step = false;
  }
}

void ant_gait(const float height, const float time) {
  for (int a = 0; a < 2; a++) {
    for (int c = 0; c < 4; c++) {
      const int i = c * 2 + (1 - a);
      pbpos[0][i] = pos[0][i];
      pbpos[1][i] = pos[1][i];
      pbpos[2][i] = pos[2][i];
    }
    for (float b = 0; b < 1; b += 0.01) {
      for (int c = 0; c < 4; c++) {
        const int i = c * 2 + a;
        RmatrixZ(pos[0][i], pos[1][i], pos[2][i], -(angle / 100), 0,
                  0, 0);
        Tmatrix(pos[0][i], pos[1][i], pos[2][i], -(x_step / 100),
                -(y_step / 100), 0);
      }
      for (int c = 0; c < 4; c++) {
        const int i = c * 2 + (1 - a);
        float cr[3] = {STAND_POS[0][i], STAND_POS[1][i], STAND_POS[2][i]};
        RmatrixZ(cr[0], cr[1], cr[2], angle / 2.0, 0, 0, 0);
        Tmatrix(cr[0], cr[1], cr[2], x_step / 2.0,
                y_step / 2.0, 0);
        float bm[3][4] = {
            // prev
            {cr[0], cr[0], pbpos[0][i], pbpos[0][i]},
            {cr[1], cr[1], pbpos[1][i], pbpos[1][i]},
            {STAND_POS[2][i], STAND_POS[2][i] + height,
              STAND_POS[2][i] + height, STAND_POS[2][i]},
        };
        Spline3D(bm, b, pos[0][i], pos[1][i], pos[2][i]);
      }
      bodyToServos(pos);
      delayMicroseconds(speed * 5000);
    }
    first_step = false;
  }
}

void Nightmare::moveleg(float dir0, float dir1, float dir2, float dir3) {
  if (dir1 > 400 and (millis() - timing) > 300) {
    timing = millis();
    leg++;
    if (leg > 7) {
      leg = 0;
    }
  } else if (dir2 < -400 and (millis() - timing) > 300) {
    timing = millis();
    leg--;
    if (leg < 0) {
      leg = 7;
    }
  }
  float actualpos[3][8] = {};
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 3; j++) {
      actualpos[j][i] = STAND_POS[j][i];
      pos[j][i] = STAND_POS[j][i];
    }
  }
  actualpos[0][leg] += limit(map(dir3, -500, 500, -6, 6), -6, 6);
  actualpos[1][leg] += limit(map(dir2, -500, 500, -6, 6), -6, 6);
  actualpos[2][leg] += limit(map(dir0, -500, 500, 12, 0), -6, 6);
  pos[0][leg] = actualpos[0][leg];
  pos[1][leg] = actualpos[1][leg];
  pos[2][leg] = actualpos[2][leg];
  bodyToServos(pos);
}

void Nightmare::move(float alpha, float beta, float gamma, float x_move,
                   float y_move, float z_move) {
  for (int i = 0; i < 8; i++) {
    float actualpos[3][8] = {};
    for (int j = 0; j < 3; j++) {
      actualpos[j][i] = STAND_POS[j][i];
    }
    Rmatrix(actualpos[0][i], actualpos[1][i], actualpos[2][i], alpha, beta,
            gamma, 0, 0, 0);
    actualpos[0][i] += x_move;
    actualpos[1][i] += y_move;
    for (int j = 0; j < 3; j++) {
      pos[j][i] = actualpos[j][i];
    }
  }
  bodyToServos(pos);
}
