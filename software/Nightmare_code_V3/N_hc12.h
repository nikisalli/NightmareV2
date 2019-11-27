#ifndef N_HC12_H
#define N_HC12_H

#include "Arduino.h"

namespace hc12 {

extern const int angle1();
extern const int speed1();
extern const int angle2();
extern const int speed2();
extern const int active();
extern const int state();
extern const int state1();

extern void init();
extern bool receive();
extern void write();

}

#endif
