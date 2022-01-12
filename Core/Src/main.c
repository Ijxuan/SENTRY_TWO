/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "FPS_Calculate.h"

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
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t time_every100us=0;
//int time_every1s=0;
//uint32_t task2_times=0;
//uint32_t task3_times=0;
//uint32_t pendsv_times=0;
//uint32_t sysclk_to_pendsv=0;
//int guance1;
//int guance2;
uint32_t zdcsjc=0;//can���մ���
uint32_t time7_times=0;//��ʱ��7�жϽ������
uint32_t task_init_times=0;//��ʼ���������д���
uint32_t task_can_times=0;//CAN�����������д���
uint32_t task_can2_times=0;//CAN2�����������д���

uint32_t task_debug_times=0;//��λ�������������д���
uint32_t task_controul_times=0;//�ܿ����������д���
int every_1s_times=0;//�������

float yaw_trage_angle=0;
float yaw_trage_angle2;

float yaw_trage_speed=0;
float PITCH_trage_angle=0;
float PITCH_trage_angle_2=0;

int PITCH_trage_speed=0;
int CHASSIS_trage_speed=0;
int CHASSIS_trage_angle=0;

float PITCH_MAX_angle=0;
float PITCH_MIN_angle=0;

int CHASSIS_MAX_SPEED=9000;

int send_to_yaw=0;//���͸�yaw�������
int send_to_pitch=0;//���͸�pitch�������
int send_to_chassis=0;//���͸����̵�����
int send_to_SHOOT_R=0;//���͸��ҷ���Ħ���ֵ�����  +
int send_to_SHOOT_L=0;//���͸�����Ħ���ֵ�����  -
int send_to_driver=0;//���͸����̵��������

float PITCH_IMU_Kp=1;

int M_3508_error=0;

int SHOOT_L=0;//��Ħ���ֵ�Ŀ���ٶ�   Ӧ��Ϊ��ֵ
int SHOOT_R=0;//��Ħ���ֵ�Ŀ���ٶ�

bool HWswitch_R=1;
bool HWswitch_L=1;

bool HWswitch_R_last=1;
bool HWswitch_L_last=1;

int CHASSIS_L_MAX=510000;
int CHASSIS_R_MIN=0;

int ENCODER_L_MAX=510000;//������ͨ�������µ����ֵ
int ENCODER_R_MIN=0;	//������ͨ�������µ���Сֵ
int ENCODER_M_MID=250000;//������ͨ�������µ��м�ֵ

int ENCODER_ARRIVE_MAX=0;//�������ִ�����ֵ
int ENCODER_ARRIVE_MIN=0;//�������ִ����Сֵ

int speed_change=0;
int CHASSIS_MID=250000;

int driver_targe_speed=0;

short int speed_change_times=0;

float DEBUFF=0;

int uart_8_times=0;


int time_3_times=0;

float Vision_RawData_Yaw_Angle=0;
float Vision_RawData_Pitch_Angle=0;

//driver  plate
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_RNG_Init();
  MX_TIM5_Init();
  MX_UART8_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
   usart1_dr16_init();

  
//USART6->DR = '2'; 
    /*ʹ�ܶ�ʱ��1�ж�*/
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim3);
	
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
//	  	   NM_swj();
//	  osdelay();
	  HAL_Delay(1);
//	  NM_swj2();
	  	  HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//  uint32_t wait;
//uint32_t tickstart;
int my_ms_delay1(uint32_t ms_Delay)
	//ms_Delay=100
{
//	   tickstart = MY_GetTick();
//   wait = time_every100us+10*ms_Delay;

  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

  while(1)
  {
//	 if( time_every100us> wait)
//	 {
//		 	  guance1=time_every100us- wait;
//		 return 0;
//	 }
//	 	  guance2=guance1;
  }
	
}

void my_ms_delay2(uint32_t ms_Delay2)
	//ms_Delay=100
{
//	  uint32_t tickstart2 = time_every100us;
//  uint32_t wait2 = ms_Delay2*10;

  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

//  while((MY_GetTick() - tickstart2) < wait2)
//  {
//  }
	
}

uint32_t MY_GetTick(void)
{
//  return time_every100us;
	  return 1000;

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
