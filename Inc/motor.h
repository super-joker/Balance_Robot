#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"



#define MAX_SPEED 1000



void SetMotorA(int speed);
int ReadEncoderA(void);





int CalVelocity(int encoder, int target_speed);





#endif /* __MOTOR_H */

