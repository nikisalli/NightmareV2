#ifndef N_DEFINES_H
#define N_DEFINES_H

#include "Arduino.h"
#include "N_structs.h"

//
//                        /
//                      /    ______
//                    /     |      |                    |-|-----|-|
//                  FM     /| +--+ | ^               O               O
//                 /     /  |______| |               | \           / |
//               /     /    / |  |   |         LEG 1 |  \  (:::)  /  | LEG 8
//             /     /    /   |  |   |               |   \ __X__ /   |
//      <--CX-->   /    /     |  |   |          O\        X     X        /O
//       ___ ____/_   /       |  |   |    LEG 2 |   \     )     (     /   |
//------|   |      |/         |  |  TB          |      \X         X/      | LEG 7
//      | + | +--+ |          |  |   |          |O__     )       (     __O|
//------|___|______|          |  |   |           |   ''\X         X/''   |
//                            |__|   |           |        )     (        | 
//                            |  |   |    LEG 3  | O______X_____X______O |  LEG 6
//                            |  |   |             |        ]|[        |
//                            |__|   v             |       || ||       |
//                                          LEG 4  |       || ||       |  LEG 5
//                                                         |___|

const bool LEFT  = true;  //macros for code cleaness.
const bool RIGHT = false;

//==========================================//
//===========BODY & LEGS DIMENSIONS=========//
//==========================================//

// Dimension of each segment of each leg in centimeters.
//                                  L1C  L1F   L1T   L2C  L2F   L2T   L3C  L3F   L3T   L4C  L4F   L4T    L5C  L5F   L5T   L6C  L6F   L6T   L7C  L7F   L7T   L8C  L8F   L8T
const float LEG_DIMENSIONS[24] = {  6.5, 13.0, 17.0, 6.5, 13.0, 17.0, 6.5, 13.0, 17.0, 6.5, 13.0, 17.0,  6.5, 13.0, 17.0, 6.5, 13.0, 17.0, 6.5, 13.0, 17.0, 6.5, 13.0, 17.0};

// Offset of each servo in degrees.
//                                 L1C   L1F L1T  L2C  L2F L2T L3C  L3F L3T L4C   L4F L4T L5C    L5F L5T L6C   L6F L6T L7C  L7F L7T L8C   L8F L8T
const float SERVO_OFFSETS[24]    = {-45.3, 0,  0,  -9.1, 0,  0,  9.1, 0,  0,  45.3, 0,  0,  -45.3, 0,  0,  -9.1, 0,  0,  9.1, 0,  0,  45.3, 0,  0};

// Side of each leg in  respect to the body.
//                        LEG1  LEG2  LEG3  LEG4  LEG5   LEG6   LEG7   LEG8
const bool LEG_SIDE[8] = {LEFT, LEFT, LEFT, LEFT, RIGHT, RIGHT, RIGHT, RIGHT};

const float BODY_LENGHT_MAX = 21.6f;
const float BODY_LENGHT_MIN = 8.00f;
const float BODY_WIDTH_MAX  = 23.0f;
const float BODY_WIDTH_MIN  = 14.0f;

//   /\      LEG1------LEG8         -  -           
//  /||\     /           \          |  |           
//   ||     /             \         |  |           
//   ||    /               \        |  |           
//        /       (+)       \       |  | LEG1 LEG8   
//      LEG2                LEG7    -  | X OFFSET   - LEG2 LEG7
//       |                    |     |  |            | X OFFSET
//       |    (-)  O  (+)     |     -  -            - 
//       |     X,Y CENTER     |     |  |            | LEG3 LEG6
//      LEG3      0,0       LEG6    -  |            - X OFFSET
//        \       (-)       /       |  | LEG4 LEG5 
//         \               /        |  | X OFFSET  
//          \             /         |  |          
//           \           /          |  |           
//  X        LEG4------LEG5         -  -           
//  ^    |-----|----|----|-----|
//  |     LEG2  LEG1 LEG8  LEG7   Y OFFSET
//  | Z   LEG3  LEG4 LEG5  LEG6
//  +------> Y

// Offset of each leg attach point to the body in respect to the center O of the body in cm.
const float LEG_START_OFFSET[16] = {
   (BODY_LENGHT_MAX/2), -(BODY_WIDTH_MIN/2),    //L1X  L1Y
   (BODY_LENGHT_MIN/2), -(BODY_WIDTH_MAX/2),    //L2X  L2Y
  -(BODY_LENGHT_MIN/2), -(BODY_WIDTH_MAX/2),    //L3X  L3Y
  -(BODY_LENGHT_MAX/2), -(BODY_WIDTH_MIN/2),    //L4X  L4Y
  -(BODY_LENGHT_MAX/2),  (BODY_WIDTH_MIN/2),    //L5X  L5Y
  -(BODY_LENGHT_MIN/2),  (BODY_WIDTH_MAX/2),    //L6X  L6Y
   (BODY_LENGHT_MIN/2),  (BODY_WIDTH_MAX/2),    //L7X  L7Y
   (BODY_LENGHT_MAX/2),  (BODY_WIDTH_MIN/2)     //L8X  L8Y
};

//==========================================//
//===========WALK & STAND SETTINGS==========//
//==========================================//

// Step settings.
const float MAX_STEP_LENGHT   = 7.0; //maximum arc lenght covered by two points of a natural spline while walking
const float MAX_STEP_HEIGHT   = 5.0; //natural spline max point height from ground
const float MINIMUM_STEP_TIME = 0.5; //minimum time for executing a step
const float WALK_FRAMES       = 30;  //maximum frames to compute while walking
const float START_Z           = -4;  //initial leg height
const float STAND_Z           = -12; //stand position leg height
const float BODY_X_SHIFT      = 0;   //bias added to 

const int RANDOM_SEQ[2][8] = { //sequences to follow while adjusting leg positions
  {0,2,1,3, 4,6,5,7},
  {2,1,3,0, 7,5,6,4},
};

// Natural position. Used while standing or doing nothing and to calculate everything with it.
const float STAND_POS[3][8] = {
// L1        L2        L3        L4        L5        L6        L7        L8
  { 27,      10,      -10,      -27,      -27,      -10,       10,       27},      //X
  {-17,     -27 ,     -27,      -17,       17,       27,       27,       17},      //Y
  {STAND_Z,  STAND_Z,  STAND_Z,  STAND_Z,  STAND_Z,  STAND_Z,  STAND_Z,  STAND_Z}, //Z
};

//==========================================//
//===========ELECTRONICS SETTINGS===========//
//==========================================//

const int   FAN1_PIN = 21;
const int   FAN2_PIN = 22;
const int   SERVO_PIN_TX_ENB = GPIO_NUM_18;
const int   SERVO_PIN_RX_ENB = GPIO_NUM_19;
const int   HC12_PIN_TX = 2;
const int   HC12_PIN_RX = 15;
const int   HC12_PIN_SET = 13;
const int   DEBUG_LED_PIN_1 = 4;
const int   DEBUG_LED_PIN_2 = 5;
const int   BATT_VOLTAGE_READ_PIN = 35;
const int   BATT_VOLTAGE_READ_FILTER_CONSTANT = 2;
const int   BATT_CURR_READ_PIN = 32;
const int   BATT_CURR_READ_FILTER_CONSTANT = 1;
const int   HC12_BAUD_RATE = 9600;
const int   DEF_FAN_SPEED = 100;
const int   MAX_FAN_SPEED = 255;
const int   PANDA_POWER_PIN = 23;
const int   PANDA_POWER_TIME = 3000;
const float BATT_MIN_VOLTAGE = 7.0f;
const float BATT_MAX_VOLTAGE = 8.5f;

#endif
