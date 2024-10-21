#ifndef _CONTROL_H
#define _CONTROL_H

#include "sys.h" 

int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int Target,int Encoder_left,int Encoder_right);
int Turn(int gyro_Z,int YK);


#endif
