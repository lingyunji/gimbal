#include "stm32f4xx_hal.h"
#include "BSP_Data.h"

#define TIME_MINIPCLOST 500000

typedef struct
{
	float Yaw_Now;	//δ�����ֵ
	float Pitch_Now;
	uint32_t Distance_Now;	//��λ������
	float Yaw_Que[5];
	float Pitch_Que[5];
	uint32_t Distance_Que[10];
	float Yaw_Get;	
	float Pitch_Get;
	uint32_t Distance_Get;
	uint8_t Flag_Get;
	float Yaw_Real;
	float Pitch_Real;
	float Yaw_Zero;
	float Pitch_Zero;
	uint8_t Enemy_Flag;				//�Ƿ�ʶ�𵽵��� 0����  1����
	uint8_t Enemy_Distance;			//���˾���	����10��λm
	uint8_t Used_Flag;		//������Ϣ��ʹ�ù��ı�־ 1��ʹ�ù�   0��δʹ��
	uint32_t TimeStamp;		//ʶ��ɹ���ʱ���
}MiniPC_Res;

extern void Mainifold_Res_Task(void);
extern CIRCLE_BUFF_t Que_Mainifold ; 
extern void Send_MiniPC(void);
extern MiniPC_Res MiniPC_Data;