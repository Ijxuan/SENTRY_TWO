#include "MY_CHASSIS_CONTROL.h"
#include "CHASSIS_CONTROL_2.h"

#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
//��3508����ķ��������,��PA8
//��3508��С�ķ������ұ�,��PA9


bool Random_CHASSIS_CHOOSE=1;//�Ƿ�ѡ�����ģʽ
bool Cruise_CHASSIS_CHOOSE=0;//�Ƿ�ѡ��Ѳ��ģʽ
int arrive_speed_times=0;
int disable_times=0;
int disable_targe_times=100;//ʧ�ܳ���ʱ��:100  150  200 250  300
int next_disable_start_times=200;//�´�ʧ�ܼ��ʱ��:200  300 400 500
int whether_change_direction=0;//����ʹ��ʱ�Ƿ����
CH_DO_NOT_STOP_AT_ONE_AREA DO_NOT_STOP;//��Ҫ��ͬһ����ͣ������
void CHASSIS_CONTROUL(void)
{
	#if PID_CHASSIS_MOTOR
			#if 1	//���������Ѳ���˶�
					if(DR16.rc.s_left==3)//�Զ�����
					{
						

						
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
					
				if(DR16.rc.ch4_DW<=-400)//����
				{
					
				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
				Cruise_CHASSIS_CHOOSE=0;
				}
				
				if(DR16.rc.ch4_DW>=400)//����
				{
				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
				Random_CHASSIS_CHOOSE=0;
					
				}
				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
				Random_CHASSIS();//���ģʽ
				
//				CHASSIS_trage_speed=0;//����
					
				}
//				else
//				{
////					if( HWswitch_L==0)// �����Ӧ���ˣ������˶�
////					CHASSIS_trage_angle=-9990000;
////					else if(HWswitch_R==0)//	�ҹ���Ӧ���ˣ������˶�
////					CHASSIS_trage_angle=9990000;
////				
////					P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle

////					CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//˫��
//////					else 				//Ĭ�������˶�
//////					CHASSIS_trage_angle=990000;		
////				CHASSIS_trage_speed=0;//����
//					
//				}

					}
//			CHASSIS_MOTOR_ANGLE_pid.Max_result=1200;
			#endif
					if(DR16.rc.s_left==1)//ң��������  ����
					{
					CHASSIS_trage_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*CHASSIS_MAX_SPEED;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
					}


					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed,M3508s[3].realSpeed);
			send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;
//					Power_Calculate();
//					send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result*Chassis_PowerLimit;
#endif
	

	
}




Random_t RANDOM_CHASSIS;

const uint16_t Random_CHANGE_times = 500; //500ms�������
const uint8_t Random_Proportion = 10;      //�������(100-Random_Proportion)
const uint16_t Random_CHANGE_speed = 1500;      //�ٴα���Ҫ�ﵽ����ٶ�����

//���ģʽ
void Random_CHASSIS(void)
{
    if (abs(CHASSIS_trage_speed) != 4500*Chassis_PowerLimit)
    {
        CHASSIS_trage_speed = 4500*Chassis_PowerLimit;//����˶��Ļ����ٶ�
    }//����˶�   ��ʼ���ٶ�   ��Random_Velocity�������˶�
    RANDOM_CHASSIS.number = Get_RandomNumbers_Range(0, 100);
//					if(M3508s[3].totalAngle>(CHASSIS_R_MIN+100000)&&M3508s[3].totalAngle<(CHASSIS_L_MAX-100000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��

//    RANDOM_CHASSIS.sampling++;
	
	#if 1
	if(	abs(M3508s[3].realSpeed) >Random_CHANGE_speed	)
	    RANDOM_CHASSIS.sampling++;
	if(DO_NOT_STOP.This_area_stay_times>1000)//��ͬһ����ͣ������3s,��ʼ��ͼ
	{
		RANDOM_CHASSIS.sampling=0;
		arrive_speed_times=0;
					stop_CH_OP_BC_LESS=0;

	}
	
//		if(	arrive_targe_angle==1	)
//	    RANDOM_CHASSIS.sampling++;
	#endif
						Power_Calculate();

					if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
				{
			CHASSIS_trage_speed=4500*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
				}
			
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//����߽���� ��ʮ����ʮ��
			{
				CHASSIS_trage_speed=-4500*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}
	
    if (RANDOM_CHASSIS.sampling == Random_CHANGE_times)
    {
        if (RANDOM_CHASSIS.number >= Random_Proportion)//�Ƿ����
        {
            CHASSIS_trage_speed = -CHASSIS_trage_speed;
			arrive_targe_angle=0;
			stop_CH_OP_BC_LESS=0;
			speed_change_times++;
        }
        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
    }
	
						if(abs(CHASSIS_trage_speed)>1500)
					{
						if(abs(M3508s[3].realSpeed)>(abs(CHASSIS_trage_speed)-200))
						{
						arrive_speed_times++;	
							
						}
						
					}
					if(arrive_speed_times>150)//�й���ʧ��
					{
						if(abs(M3508s[3].realSpeed)<(abs(CHASSIS_trage_speed)-2000))
						{
						disable_times++;	
							
						}
						stop_CH_OP_BC_LESS=1;
//						CHASSIS_trage_speed=0;
						if(disable_times>80)
						{
							arrive_speed_times=0;
							disable_times=0;
							stop_CH_OP_BC_LESS=0;
						}
						
						
					}

	
}


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;

	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}


void Cruise_CHASSIS(void)//		cruise	Ѳ��
{
						
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
			CHASSIS_trage_angle=9900000;
			
			
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��
			CHASSIS_trage_angle=-9900000;
			
			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//˫��
			
				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
				//CHASSIS_MID-CHASSIS_R_MIN   һ���г�
				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//�������ٷ�֮70
				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//�������м�죬������
	
	
	
	
//    if (fabs(Chassis.Velocity.temp_Speed) != Cruise_Velocity)
//    {
//        Chassis.Velocity.temp_Speed = Cruise_Velocity;
//    }
}





Encoder_t Chassis_Encoder;

//��ȡ������ֵ����
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
	
}



/***************************************
  * @brief  :���̹��ʼ���
  * @param  :Judge_DATA.Power,Judge_DATA.Power_Buffer,Power_MAX,Power_Buffer_MAX
****************************************/
void Power_Calculate()
{
	static uint8_t flag=0;
	
	if(ext_power_heat_data.data.chassis_power_buffer < 50)flag = 1;
	if(flag==1 && ext_power_heat_data.data.chassis_power_buffer > 150)flag=0;
	
	if(flag==1)Chassis_PowerLimit = 0.65f;
	else Chassis_PowerLimit = 1.0f;
	
}













