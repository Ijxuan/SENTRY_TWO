/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DR16_RECIVE.h"
#include "usart.h"
#include "BSP_CAN.h"
#include "GM6020.h"
#include "my_positionPID_bate.h"
#include "DJI_C_IMU.h"
#include "M3508.h"
#include "MY_CLOUD_CONTROL.h"
#include "MY_CHASSIS_CONTROL.h"
#include "my_IncrementPID_bate.h"
#include "tim.h"
#include "M2006.h"
#include "MY_SHOOT_CONTROL.h"
#include "Vision.h"
#include "user_UART.h"
#include "FPS_Calculate.h"
//#include "task.h"

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
/* USER CODE BEGIN Variables */
osMessageQId CAN1_Queue;
osMessageQId CAN2_Queue;
//����������
osThreadId RobotCtrl_Handle;
//������ں���
void Robot_Control(void const *argument);
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MYTask03Handle;
osThreadId myTask04Handle;
osThreadId myTaskforinitHandle;
osThreadId Task_Can2_ReiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void CAN1_recive(void const * argument);
void DeBug(void const * argument);
void init_task(void const * argument);
void Can2_Reive(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
		CAN1_Queue = xQueueCreate(32, sizeof(CAN_Rx_TypeDef));
	    CAN2_Queue = xQueueCreate(32, sizeof(CAN_Rx_TypeDef));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of MYTask03 */
  osThreadDef(MYTask03, CAN1_recive, osPriorityAboveNormal, 0, 128);
  MYTask03Handle = osThreadCreate(osThread(MYTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, DeBug, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTaskforinit */
  osThreadDef(myTaskforinit, init_task, osPriorityBelowNormal, 0, 128);
  myTaskforinitHandle = osThreadCreate(osThread(myTaskforinit), NULL);

  /* definition and creation of Task_Can2_Reive */
  osThreadDef(Task_Can2_Reive, Can2_Reive, osPriorityAboveNormal, 0, 256);
  Task_Can2_ReiveHandle = osThreadCreate(osThread(Task_Can2_Reive), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ...
defaultTask    	245		<1%
Task_Robot_Cont	1580		1%
Task_Can2_Reive	4325		4%   CAN2
MYTask03       	7604		7%   CAN1
myTask04       	1385		1%   DEBUG
IDLE           	80866		81%  */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CAN1_recive */
/**
* @brief Function implementing the MYTask03 thread.
* @param argument: Not used
* @retval None
*/
int last_turnCount=0;
/* USER CODE END Header_CAN1_recive */
void CAN1_recive(void const * argument)
{
  /* USER CODE BEGIN CAN1_recive */
		//CAN1���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN1_Rx_Structure;
	  task_can_times++;

	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;
  /* Infinite loop */
  for(;;)
  {
	  task_can_times++;
	  		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN1_Queue, &CAN1_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�
		if (ExitQueue_Status == pdTRUE)
		{
			/*Cloud*/
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START)
			{
				//��̨��yaw��
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}
			
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (GM6020_READID_START+1))
			{
				//��̨��yaw��
				GM6020_Pitch_getInfo(CAN1_Rx_Structure);//6400-7160
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == M3508_READID_END)
			{
				//���̵��
				M3508_getInfo(CAN1_Rx_Structure);//6400-7160
				#if 0
				if(M3508s[3].realSpeed>900||M3508s[3].realSpeed<-900)//���˶���
				{
						if(M3508s[3].realSpeed>1000)//�����˶�
						{
								if(	last_turnCount==M3508s[3].turnCount)
								{//ͬһȦ֮��
										if(M3508s[3].realSpeed<=M3508s[3].last_angle)
										{
											M_3508_error++;
										}
								}
						}
						if(M3508s[3].realSpeed<-1000)//�����˶�
						{
								if(	last_turnCount==M3508s[3].turnCount)
								{//ͬһȦ֮��
										if(M3508s[3].realSpeed>=M3508s[3].last_angle)
										{
											M_3508_error++;
										}
								}
						}
					
					
				}
				
									last_turnCount=M3508s[3].turnCount;

				#endif
				
				
				
				
			}
					if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START))
					{
					//��Ħ���ֵ�    ID1
					M3508s1_getInfo(CAN1_Rx_Structure);//
					}
					if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START+1))
					{
					//��Ħ���ֵ�	ID2
					M3508s1_getInfo(CAN1_Rx_Structure);//
					}
					if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START+2))
					{
					//���̵��		ID3
					M2006_getInfo(CAN1_Rx_Structure);//
					}
		}	
	  
	  
//osDelay(1);
}
  /* USER CODE END CAN1_recive */
}

/* USER CODE BEGIN Header_DeBug */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
char RunTimeInfo[400];		//������������ʱ����Ϣ

/* USER CODE END Header_DeBug */
void DeBug(void const * argument)
{
  /* USER CODE BEGIN DeBug */
  /* Infinite loop */
  for(;;)
  {
	  
	  		  task_debug_times++;
			if(DR16.rc.s_right==3)	//�Ƿ���λ��

//	  Update_Vision_SendData();
	  {
//	  	   NM_swj();

//memset(RunTimeInfo,0,400);				//��Ϣ����������
			vTaskGetRunTimeStats(RunTimeInfo);		//��ȡ��������ʱ����Ϣ
		  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&RunTimeInfo, 400);	
//	  HAL_UART_Transmit(&huart6, (uint8_t *)&RunTimeInfo, 400, 0xFFFF);	 
//		  for (uint16_t i = 0; i < 400; i++)
//	{
//		while ((UART6->SR & 0X40) == 0);
//		UART6->DR = RunTimeInfo[i];
//	}
	  }

	  
//			printf("������\t\t\t����ʱ��\t������ռ�ٷֱ�\r\n");
//			printf("%s\r\n",RunTimeInfo);
    osDelay(20);
  }
  /* USER CODE END DeBug */
}

/* USER CODE BEGIN Header_init_task */
/**
* @brief Function implementing the myTaskforinit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_init_task */
void init_task(void const * argument)
{
  /* USER CODE BEGIN init_task */
	
		portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿʮ����ǿ�ƽ����ܿ���
	
//DJIC_IMU.last_pitch=0;
//	taskENTER_CRITICAL(); //�����ٽ���
		CAN1_Config();
			CAN2_Config();
	//������ģʽ��ʼ��
	//AB
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
//	HAL_Delay(1000);
	
		//�Ӿ�
	__HAL_UART_CLEAR_IDLEFLAG(&huart8);

	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart8, Vision_DataBuff, Vision_BuffSize);
	
	
	
	/*
	CAN1_Filter0_Init();
		HAL_CAN_Start(&hcan1);
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	*/ 
		/*PID������ʼ��*/
#if PID_MOTOR   //�Ƿ��������PID
				P_PID_Parameter_Init(&Yaw_Speed_pid,550, 10, 0,
					120,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					2000, -2000,//�����޷���Ҳ���ǻ��ֵ������Χ
					29000, -29000);
				P_PID_Parameter_Init(&Yaw_Angle_pid,0.03, 0, 0,
					0,
					//						  float max_error, float min_error,
					//                          float alpha,
					0, 0,
					220, -220);//
	
#endif

					

#if PID_IMU
P_PID_Parameter_Init(&Yaw_IMU_Speed_pid,500, 6, 0,
					120,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					5000, -5000,//�����޷���Ҳ���ǻ��ֵ������Χ
					28000, -28000);
				P_PID_Parameter_Init(&Yaw_IMU_Angle_pid,-11, 0, 0,
					0,
					//						  float max_error, float min_error,
					//                          float alpha,
					0, 0,
					500, -500);//Yaw_IMU_Angle_pid		
#endif
				  
#if PID_PITCH_MOTOR
				P_PID_Parameter_Init(&PITCH_Angle_pid,1, 0, 0,
					0,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					220, -220,//�����޷���Ҳ���ǻ��ֵ������Χ
					220, -220);
				P_PID_Parameter_Init(&PITCH_Speed_pid,300, 1, 0,
					120,
					//						  float max_error, float min_error,
					//                          float alpha,
					10000, -10000,//�����޷���Ҳ���ǻ��ֵ������Χ
					29000, -29000);//Yaw_IMU_Angle_pid		
#endif						  
						  
						  
#if PID_PITCH_IMU
P_PID_Parameter_Init(&PITCH_IMU_Speed_pid,200, 1, 0,
					120,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					5000, -5000,//�����޷���Ҳ���ǻ��ֵ������Χ
					28000, -28000);
				P_PID_Parameter_Init(&PITCH_IMU_Angle_pid,10, 0, 0,
					0,
					//						  float max_error, float min_error,
					//                          float alpha,
					0, 0,
					500, -500);//Yaw_IMU_Angle_pid	


#endif						  
						  
#if PID_CHASSIS_MOTOR
P_PID_Parameter_Init(&CHASSIS_MOTOR_ANGLE_pid,5, 0, 0,
					10,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					50, -50,//�����޷���Ҳ���ǻ��ֵ������Χ
					4300, -4300);
				P_PID_Parameter_Init(&CHASSIS_MOTOR_SPEED_pid,15, 0.5, 0,
					500,
					//						  float max_error, float min_error,
					//                          float alpha,
					16000, -16000,
					16000, -16000);//Yaw_IMU_Angle_pid	

#endif						  
						  
						  
						  
						  
						  

 I_PID_Parameter_Init(&SHOOT_L_I_PID, 20, 0.5, 1,
                          7000, 7000, -7000,
                          0.5,
                          14000, -14000,
                          16000, -16000); //Ħ���ֵ��

//						  
I_PID_Parameter_Init(&SHOOT_R_I_PID, 20, 0.5, 1,
                          7000, 7000, -7000,
                          0.5,
                          14000, -14000,
                          16000, -14000);//Ħ���ֵ��

I_PID_Parameter_Init(&Driver_I_PID, 13, 0.3, 1,
                          5000, //���ַ���
						  5000, -5000,//������
                          0.5,
                          5000, -5000,
                          10000, -10000);//Ħ���ֵ��
						  
/////////////////////////////////////////////////////////////////////����YAW��
P_PID_Parameter_Init(&Yaw_EM_Angle_pid,0.2, 0, 0,
					0,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					0, 0,//�����޷���Ҳ���ǻ��ֵ������Χ
					25, -25);//YAW������PID,������
					
P_PID_Parameter_Init(&Yaw_EM_Speed_pid,200, 3, 0,
					120,
					//						  float max_error, float min_error,
					//                          float alpha,
					1000, -1000,
					28000, -28000);//Yaw_IMU_Angle_pid	



/////////////////////////////////////////////////////////////////////����PITCH��
P_PID_Parameter_Init(&PITCH_EM_Speed_pid,200, 1, 0,
					120,//���������ֵ�ͻ��ַ���
					//	float max_error, float min_error,
					//                          float alpha,
					5000, -5000,//�����޷���Ҳ���ǻ��ֵ������Χ
					28000, -28000);
P_PID_Parameter_Init(&PITCH_EM_Angle_pid,10, 0, 0,
					0,
					//						  float max_error, float min_error,
					//                          float alpha,
					0, 0,
					500, -500);//Yaw_IMU_Angle_pid	



HAL_Delay(100);
//			HWswitch_L			 = HAL_GPIO_ReadPin(GPIOA,HWswitch_1_Pin);
//			HWswitch_R   		 = HAL_GPIO_ReadPin(GPIOA,HWswitch_2_Pin);
			HWswitch_L_last		=HWswitch_L;
			HWswitch_R_last		=HWswitch_R;
//	yaw_trage_angle=GM6020s[0].readAngle;//�ϵ�Ƕȳ�ʼ��




	yaw_trage_angle=DJIC_IMU.total_yaw;
	PITCH_trage_angle=GM6020s[1].readAngle;//�ϵ�Ƕȳ�ʼ��
	//pitch���������ϵ���㴦��
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							  	  	  HAL_Delay(100);
if(DJIC_IMU.pitch>50&&DJIC_IMU.pitch_turnCounts==0)
	DJIC_IMU.pitch_turnCounts=-1;


							imu_angle();//ȥ���������ԣ�����  ����
PITCH_trage_angle=PITCH_MIN_angle+30;
	task_init_times++;
						CHASSIS_trage_angle=990000;

		osThreadDef(Control, Robot_Control, RobotCtrl_Priority, 0, RobotCtrl_Size);
  RobotCtrl_Handle = osThreadCreate(osThread(Control), NULL);

//	taskEXIT_CRITICAL(); //�˳��ٽ���
	
	vTaskDelete(NULL);
	
	
	
	
	
  /* Infinite loop */
  for(;;)
  {
	  	task_init_times++;
//		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

	  	  HAL_Delay(100);
  }
  /* USER CODE END init_task */
}

/* USER CODE BEGIN Header_Can2_Reive */
/**
* @brief Function implementing the Task_Can2_Reive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can2_Reive */
void Can2_Reive(void const * argument)
{
  /* USER CODE BEGIN Can2_Reive */
	//CAN2���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN2_Rx_Structure;
	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;
		task_can2_times++;

	/* Infinite loop */
	for (;;)
	{
		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN2_Queue, &CAN2_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�
		if (ExitQueue_Status == pdTRUE)
		{
			/*DJIC_IMU*/
			//IMU_Euler
			if(CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Angle)
			{
				DJI_C_Euler_getInfo(CAN2_Rx_Structure);
			}
			//Gyro
			if(CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Gyro)
			{
				DJI_C_Gyro_getInfo(CAN2_Rx_Structure);
			}
		}
Updata_Hand_Euler_Gyro_Data();
		task_can2_times++;
		//    osDelay(1);
	}
  /* USER CODE END Can2_Reive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Robot_Control(void const *argument)
{
	/* USER CODE BEGIN RobotControl */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(3); //ÿ1����ǿ�ƽ����ܿ���
	/* Infinite loop */
	for (;;)
	{
		task_controul_times++;
//7175  ��е��λ
//7160	������ϱ߽�     ��ֵ�����˶�
//7130  ����λ           ֻ���������ֵ
					
//Ŀ��Ƕ�ֻ������6435-7125֮��				
//6430  ����λ           ֻ���������ֵ
//6400  С�����±߽�	 ��ֵ�����˶�	
		
		cloud_control();

					if(GM6020s[1].totalAngle<=6430&&send_to_pitch<0)
					send_to_pitch=0;
					if(GM6020s[1].totalAngle>=7130&&send_to_pitch>0)
					send_to_pitch=0;		
			if(DR16.rc.s_left==2||DR16.rc.s_left==0)	//ʧ�ܱ���
			{	
				send_to_yaw=0;
			send_to_pitch=0;
		
				yaw_trage_angle=DJIC_IMU.total_yaw;
				//��������ʹ��˲��ᶶһ�£�Ӧ�û�Ҫ���I���ۼӣ��Ժ���ʱ����д
			}			
		

				GM6020_SetVoltage(send_to_yaw,send_to_pitch, 0, 0);//��̨  send_to_pitch
			Get_Encoder_Value(&Chassis_Encoder, &htim5);
			switch_change();
			CHASSIS_CONTROUL();	
			
			shoot_control();
			
			driver_plate_control();
			
					if(DR16.rc.s_left==2||DR16.rc.s_left==0)	//ʧ�ܱ���
					{			
						send_to_chassis=0;
						send_to_SHOOT_R=0;
						send_to_SHOOT_L=0;
						send_to_driver=0;
					}
					M3508s1_setCurrent(send_to_SHOOT_L,send_to_SHOOT_R,send_to_driver,send_to_chassis);
					vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
					
					

					
//			  	  HAL_Delay(100);
		//���̿���ִ��
//		Chassis_FUN.Chassis_Control();
//		//��̨����ִ��
//		Cloud_FUN.Cloud_Control();
//		//��������ִ��
//		Attack_FUN.Attack_Processing();
//		
//		//CAN�������ݸ����
//		//��̨
//		GM6020_SetVoltage(Cloud.yaw_output, Cloud.pitch_output, 0, 0);
//		//Ħ����
//		//M3508s1_setCurrent(Shoot.Shoot1->OutputCurrent,Shoot.Shoot2->OutputCurrent,0,0);
//		//����
//		//M2006_setCurrent(0,0,M2006s[2].OutputCurrent,0);
//		//����
//		//M3508_setCurrent(0,0,0,Chassis.EM->OutputCurrent);
//		//�������Ƕ���ͬ��CAN1���͵�
//		M3508s1_setCurrent(Shoot.Shoot1->OutputCurrent,Shoot.Shoot2->OutputCurrent,M2006s[2].OutputCurrent,Chassis.EM->OutputCurrent);
	}

	/* USER CODE END RobotControl */
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
