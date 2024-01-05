#include "motor.h"
#include "main.h"
/**
 * 
 * dir: 0:+
 *      1:-
 * 
 * speed:0-1000
*/


//float velocity_kp = -40;
//float velocity_ki = -5;

//float velocity_kp = -10;
//float velocity_ki = -3.5;

float velocity_kp_a = -15;
float velocity_ki_a = -3;
float velocity_kp_b = -15;
float velocity_ki_b = -3;
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
	debug_printf("countA:%d.\n",(int)count);
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
	debug_printf("countB:%d.\n",(int)count);
  return count;
}





int CalVelocityA(int encoder, int target_speed)
{
	static float velocity_a, current_encoder_a;
	static float encoder_integral_a;
	// current_encoder_a = current_encoder_a*0.8 + ((encoder - target_speed) * 0.2);
	current_encoder_a = encoder - target_speed;
	encoder_integral_a += current_encoder_a;

	velocity_ki_a = -7 + (target_speed / 20);
	
	if(encoder_integral_a > 10000) encoder_integral_a = 10000;
	if(encoder_integral_a < -10000) encoder_integral_a = -10000;
	velocity_a = current_encoder_a * velocity_kp_a + encoder_integral_a * velocity_ki_a;
	debug_printf("A:encode:%d,encoder_integral_a:%d.velocity_a:%d.\n",(int)encoder,(int)encoder_integral_a,(int)velocity_a);
	return velocity_a;
}


int CalVelocityB(int encoder, int target_speed)
{
	static float velocity_b, current_encoder_b;
	static float encoder_integral_b;
	// current_encoder_b = current_encoder_b*0.8 + ((encoder - target_speed) * 0.2);
	current_encoder_b = encoder - target_speed;
	encoder_integral_b += current_encoder_b;

	velocity_ki_b = -7 + (target_speed / 20);
	
	if(encoder_integral_b > 10000) encoder_integral_b = 10000;
	if(encoder_integral_b < -10000) encoder_integral_b = -10000;
	velocity_b = current_encoder_b * velocity_kp_b + encoder_integral_b * velocity_ki_b;
	return velocity_b;
}

//int CalVelocity(int encoder, int target_speed)
//{
//	static float velocity, current_encoder;
//	static float encoder_integral;
//	current_encoder = current_encoder*0.8 + (encoder * 0.2);

//	encoder_integral += current_encoder;
//	encoder_integral -= target_speed;
//	
//	if(encoder_integral > 10000) encoder_integral = 10000;
//	if(encoder_integral < -10000) encoder_integral = -10000;
//	velocity = current_encoder * velocity_kp + encoder_integral * velocity_ki;
//	//debug_printf("encode:%d,encoder_integral:%d.velocity:%d.\n",(int)encoder,(int)encoder_integral,(int)velocity);
//	return velocity;
//}





