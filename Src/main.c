/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度	    

int debug_printf(const char *format,...);

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	//for(i=0;i<len+4;i++)HAL_UART_Transmit(&huart1,send_buf,32,1000);	//发送数据到串口1 
	for(i=0;i<len+4;i++)HAL_UART_Transmit_DMA(&huart1,send_buf,32); 	//发送数据到串口1 
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}  
/* USER CODE END 0 */


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
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  //debug_printf("Hello, Balance robot!\n");
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	uint16_t index = 0;
	uint16_t wave_data[4] = {0};
	int target_speed_a = 40;
	int target_speed_b = 40;
	int pid_speed_pwm_a = 0;
  int pid_speed_pwm_b = 0;
	int encoder_a = 0, encoder_b = 0;

	


//  while(MPU_Init())
//  {
//    debug_printf("MPU Init fail.");
//    delay_ms(200);
//  }  
//  while(mpu_dmp_init())
//  {
//    debug_printf("mpu_dmp_init fail:%d\n",mpu_dmp_init());
//    delay_ms(200);
//  }  

	
	debug_printf("Hello, world1!\n");
	
	debug_printf("Hello, world2!\n");
//	char buffer1[] = "123456789\n";
//	char buffer2[] = "1234567891111\n";
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer1, strlen(buffer1));
//	delay_us(500);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer2, strlen(buffer2));
//	
//	uint8_t aTxBuffer[] = " **111*** ";

//	HAL_UART_Transmit_DMA(&huart1, aTxBuffer, 10);
//		
	while(1)
	{
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer1, strlen(buffer1));
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer2, strlen(buffer2));
	
	
//		mpu_dmp_get_data(&pitch,&roll,&yaw);
//		temp=MPU_Get_Temperature();	//?????
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//??????????
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//???????
//    mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//??????????????????
//    usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));

//		encoder_a = ReadEncoderA();
//    encoder_b = ReadEncoderB();
//		pid_speed_pwm_a = CalVelocityA(encoder_a, target_speed_a);
//    //pid_speed_pwm_b = CalVelocityB(encoder_b, target_speed_b);
//		SetMotorA(pid_speed_pwm_a);
//    //SetMotorB(pid_speed_pwm_b);
//		wave_data[0] = target_speed_a;
//		wave_data[1] = pid_speed_pwm_a;
//		wave_data[2] = encoder_a;
//		wave_data[3] = encoder_b;
//		SendUserWave(wave_data,4);
//		
		
    HAL_Delay(10);
		
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */








int debug_printf(const char *format,...)
{
  va_list args;
  static char sendbuffer[1000];
  int rv;
  while(!usart_dma_tx_over);
  usart_dma_tx_over = 0;
	
  va_start(args,format);
  rv = vsnprintf((char*)sendbuffer,sizeof(sendbuffer),format,args);
  va_end(args);
 
  //HAL_UART_Transmit_DMA(&huart1,(uint8_t *)sendbuffer,rv);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)sendbuffer, strlen(sendbuffer));
  
 

  return rv;
}












void delay_us(uint32_t us)
{
	while(us--)
	{
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
}
void delay_ms(uint32_t ms)
{
  HAL_Delay(ms);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: debug_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
