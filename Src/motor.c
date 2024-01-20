#include "motor.h"
#include "main.h"
/**
 * 
 * dir: 0:+
 *      1:-
 * 
 * speed:0-1000
*/


float velocity_kp_a = -15;
float velocity_ki_a = -3;
float velocity_kp_b = -15;
float velocity_ki_b = -3;
float balance_kp = 100;
float balance_kd = 0.35;

void SetMotorA(int speed)
{
  if(speed >= MAX_SPEED)
  {
    speed = MAX_SPEED;
  }

  if(speed >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_A1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_A2_Pin,GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_A1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_A2_Pin,GPIO_PIN_SET);
  }
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(speed));
}


int ReadEncoderA(void)
{
  int diretion =  __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);     
  int count = __HAL_TIM_GET_COUNTER(&htim3);
	
	if(count > (0xFFFF / 2))
	{
		count = count - 0xFFFF;
	}
	
  __HAL_TIM_SET_COUNTER(&htim3,0);
  return count;
}



void SetMotorB(int speed)
{
  if(speed >= MAX_SPEED)
  {
    speed = MAX_SPEED;
  }

  if(speed >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_B1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_B2_Pin,GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_B1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_CONTROL_B2_Pin,GPIO_PIN_SET);
  }
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(speed));
}


int ReadEncoderB(void)
{
  int diretion =  __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);     
  int count = __HAL_TIM_GET_COUNTER(&htim2);
	
	if(count > (0xFFFF / 2))
	{
		count = count - 0xFFFF;
	}
	
  __HAL_TIM_SET_COUNTER(&htim2,0);
  return count;
}



int CalVelocityA(int encoder, int target_speed)
{
	static float velocity_a, current_encoder_a;
	static float encoder_integral_a;
	current_encoder_a = encoder - target_speed;
	encoder_integral_a += current_encoder_a;

	velocity_ki_a = -8.2 + (target_speed / 20);
	
	if(encoder_integral_a > 10000) encoder_integral_a = 10000;
	if(encoder_integral_a < -10000) encoder_integral_a = -10000;
	velocity_a = current_encoder_a * velocity_kp_a + encoder_integral_a * velocity_ki_a;
	return velocity_a;
}


int CalVelocityB(int encoder, int target_speed)
{
	static float velocity_b, current_encoder_b;
	static float encoder_integral_b;
	current_encoder_b = encoder - target_speed;
	encoder_integral_b += current_encoder_b;

	velocity_ki_a = -8.2 + (target_speed / 20);
	
	if(encoder_integral_b > 10000) encoder_integral_b = 10000;
	if(encoder_integral_b < -10000) encoder_integral_b = -10000;
	velocity_b = current_encoder_b * velocity_kp_b + encoder_integral_b * velocity_ki_b;
	return velocity_b;
}


int CalBalance(float angle,float mechanical_balance,float gyro)
{  
	 float bias = angle-mechanical_balance;    							 //===求出平衡的角度中值和机械相关
	 return balance_kp * bias + balance_kd * gyro;          //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
}

