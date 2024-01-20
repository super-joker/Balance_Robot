#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"



#define MAX_SPEED 1000



void SetMotorA(int speed);
void SetMotorB(int speed);
int ReadEncoderA(void);
int ReadEncoderB(void);




int CalVelocityA(int encoder, int target_speed);
int CalVelocityB(int encoder, int target_speed);
int CalBalance(float angle,float mechanical_balance,float gyro);



#endif /* __MOTOR_H */

