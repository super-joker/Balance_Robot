#include "motor.h"
#include "main.h"
/**
 * 
 * dir: 0:+
 *      1:-
 * 
 * speed:0-1000
*/


float velocity_kp = 15;
float velocity_ki = 3;
float balance_kp = -140; // -140
float balance_kd = 0.1; // 0.1
float balance_ki = -2;   // -1
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



int CalVelocity(int encoder_a,int encoder_b, int target_speed)
{
	static int velocity, current_encoder;
	static int encoder_integral;
	current_encoder = target_speed - ((encoder_a + encoder_b) / 2);
	encoder_integral += current_encoder;

	velocity_ki = 8.2 - (target_speed / 20);
	
	if(encoder_integral > 10000) encoder_integral = 10000;
	if(encoder_integral < -10000) encoder_integral = -10000;
	velocity = current_encoder * velocity_kp + encoder_integral * velocity_ki;

  //debug_printf("encoder_integral:%d, velocity:%d\n",encoder_integral,velocity);
	return velocity;
}



int CalBalance(float angle,float mechanical_balance,short gyro)
{  
	static float angle_integral;

	
	if(angle_integral > 1000) angle_integral = 1000;
	if(angle_integral < -1000) angle_integral = -1000;
	
	float bias = 0 - angle + mechanical_balance;//-mechanical_balance;    							 //===求出平衡的角度中值和机械相关
  angle_integral += bias;
	return balance_kp * bias + balance_kd * gyro + balance_ki * angle_integral;          //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
}


void SendUserWave(uint16_t data[],uint16_t len);
int encoder_a, encoder_b;
static float pitch,roll,yaw; 		//欧拉角
static short aacx,aacy,aacz;		//加速度传感器原始数据
static  short gyrox,gyroy,gyroz;	//陀螺仪原始数据
uint16_t wave_data[10] = {0};
int target_speed = 0;
int pid_speed_pwm;
int pid_balance_pwm;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if((GPIO_Pin == MPU_INT_Pin) && gStartAllControl == 1)
  {
    mpu_dmp_get_data(&pitch,&roll,&yaw);
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//陀螺仪

    encoder_a = ReadEncoderA();
    encoder_b = ReadEncoderB();

		
		
    pid_balance_pwm = CalBalance(pitch,0,gyroy);
    pid_speed_pwm = CalVelocity(encoder_a,encoder_b,target_speed);

    int speed_balance_pwm = (pid_balance_pwm + pid_speed_pwm) / 2;
    SetMotorA(speed_balance_pwm);
    SetMotorB(-speed_balance_pwm);
		//debug_printf("pid_speed_pwm:%d,pid_balance_pwm:%d,speed_balance_pwm:%d\n",pid_speed_pwm,pid_balance_pwm,speed_balance_pwm);
		
  //  wave_data[0] = encoder_a;
  //  wave_data[1] = encoder_b;
  //  wave_data[2] = target_speed;
  //  wave_data[3] = pid_speed_pwm;
  //  wave_data[5] = (uint16_t)pitch;
  //  wave_data[6] = (uint16_t)gyroy;
  //  wave_data[7] = pid_balance_pwm;
  //  SendUserWave(wave_data,8);	


     debug_printf("encoder_a:%d,encoder_b:%d\n",encoder_a,encoder_b);
    //debug_printf("pitch:%f,gyroy:%d,pid_balance_pwm:%d\n",pitch,gyroy,pid_balance_pwm);
  }
}






void SendUserWave(uint16_t data[],uint16_t len)
{
	if(data == NULL || len == 0 )
		return;
	
	
  uint8_t cnt = 0;
  uint8_t databuffer[7];
	uint8_t sum = 0, index = 0;
	
	
  databuffer[cnt++] = 0xAA;
  databuffer[cnt++] = 0xAA;
  databuffer[cnt++] = 0xF1;
  databuffer[cnt++] = len*2;
	
	for(index = 0; index < len; index++)
	{
		databuffer[cnt++] = (uint8_t)((data[index]>>8) & 0xFF);
		databuffer[cnt++] = (uint8_t)(data[index] & 0xFF);
		
	}

  for(index = 0; index < cnt; index++)
  {
    sum += databuffer[index];
  }

  databuffer[cnt++] = sum;
  //HAL_UART_Transmit(&huart1,databuffer,cnt,1000);
  HAL_UART_Transmit_DMA(&huart1,databuffer,cnt);

}
