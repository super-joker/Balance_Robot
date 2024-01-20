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
	    

int debug_printf(const char *format,...);
uint8_t gStartAllControl = 0;


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

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	uint16_t index = 0;


  LED_OFF();
	while(MPU_Init())
  {
    HAL_Delay(200);
  }
	while(mpu_dmp_init())
  {
    HAL_Delay(200);
  }
	gStartAllControl = 1;
	LED_ON();
	
	//gyroy
	while(1)
	{


		// debug_printf("pitch:%f,roll:%f,yaw:%f,gyrox:%d,gyroy:%d,gyroz:%d\n",pitch,roll,yaw,gyrox,gyroy,gyroz);
//    mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//??????????????????
//    usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));

		// encoder_a = ReadEncoderA();
    // encoder_b = ReadEncoderB();
		
		//debug_printf("encoder_a:%d,encoder_b:%d\n",encoder_a,encoder_b);
		
//		pid_speed_pwm_a = CalVelocityA(encoder_a, target_speed_a);
//    pid_speed_pwm_b = CalVelocityB(encoder_b, target_speed_b);
//		SetMotorA(pid_speed_pwm_a);
//    SetMotorB(-pid_speed_pwm_b);
//		wave_data[0] = target_speed_a;
//		wave_data[1] = encoder_a;
//		wave_data[2] = encoder_b;
//		wave_data[3] = encoder_a;
//		SendUserWave(wave_data,4);	


//    pid_speed_balance_a = CalBalance(pitch,0,gyroy);
//    pid_speed_balance_b = CalBalance(pitch,0,gyroy);

//    SetMotorA(pid_speed_balance_a + pid_speed_pwm_a);
//    SetMotorB(-pid_speed_balance_b - pid_speed_pwm_b);
	
	
		// SetMotorA(300);
    // SetMotorB(-300);
	  //debug_printf("pitch:%f,gyroy:%d\n",pitch,gyroy );
//		wave_data[0] = 0;
//		wave_data[1] = pitch;
//		wave_data[3] = pid_speed_balance_a;
//		SendUserWave(wave_data,4);	
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
  // while(!usart_dma_tx_over);
  // usart_dma_tx_over = 0;
	
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
