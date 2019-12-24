#ifndef N_SERVOS_H
#define N_SERVOS_H

#include "Arduino.h"

void servoInit();
void servoMove(int pos, byte id);
void servoDetach(byte id);
void servoAttach(byte id);
int servoReadPos(byte id);

void bodyToServos(float pos[][8]);
void servoWrite(float angles[]);
void bodyDetach();
void bodyAttach();

#endif
