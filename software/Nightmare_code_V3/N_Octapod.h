#ifndef N_OCTAPOD_H
#define N_OCTAPOD_H

#include"N_defines.h"
#include"N_math.h"

namespace Nightmare {

extern float x_step;
extern float y_step;
extern float angle;
extern float speed;
extern struct Dleg dlegs[8];

extern void init();
extern void stand();
extern void force_stand();
extern void step(const int walk_pattern, const float height, const float time);
extern void standUp();
extern void move(float alpha, float beta, float gamma, float x_move, float y_move, float z_move);
extern void moveleg(float dir0, float dir1, float dir2, float dir3);
extern void sit();
extern bool pandaIsOn();
extern void startPanda();

};

#endif
