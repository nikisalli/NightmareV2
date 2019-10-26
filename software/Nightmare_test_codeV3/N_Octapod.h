#ifndef N_OCTAPOD_H
#define N_OCTAPOD_H

#include"N_defines.h"
#include"N_math.h"

class Octapod{
public:
  float x_step;
  float y_step;
  float angle;
  float speed;
  
  Octapod();
  void init();
  void stand();
  void force_stand();
  void step(const int walk_pattern, const float height, const float time);
  void standUp();
  void move(float alpha, float beta, float gamma, float x_move, float y_move, float z_move);
  void moveleg(float dir0, float dir1, float dir2, float dir3);
  void sit();
private:

  struct Dleg dlegs[8];
  float pos[3][8] = {};
  float bpos[3][2] = {};
  float pbpos[3][8] = {};
  const int tarantula[8] = {4, 2, 7, 1, 3, 5, 0, 6};
  const int slow_gait[8] = {4, 2, 6, 0, 3, 5, 1, 7};
  unsigned long timing;
  boolean first_step = true;
  byte leg;

  void tarantula_gait(const float height, const float time);
  void scorpio_gait(const float height, const float time);
  void ant_gait(const float height, const float time);
};

#endif
