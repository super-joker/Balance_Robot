#include "motor.h"
#include "main.h"
/**
 * 
 * dir: 0:+
 *      1:-
 * 
 * speed:0-1000
*/


float velocity_kp = -40;
float velocity_ki = -5;
	
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
	printf("count:%d.\n",(int)count);
  return count;
}




int CalVelocity(int encoder, int target_speed)
{
	static float velocity, current_encoder;
	static float encoder_integral;
	current_encoder = current_encoder*0.8 + ((encoder - target_speed) * 0.2);

	encoder_integral += current_encoder;

	if(encoder_integral > 10000) encoder_integral = 10000;
	if(encoder_integral < -10000) encoder_integral = -10000;
	velocity = current_encoder * velocity_kp + encoder_integral * velocity_ki;
	printf("encode:%d,encoder_integral:%d.velocity:%d.\n",(int)encoder,(int)encoder_integral,(int)velocity);
	return velocity;
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
//	//printf("encode:%d,encoder_integral:%d.velocity:%d.\n",(int)encoder,(int)encoder_integral,(int)velocity);
//	return velocity;
//}





