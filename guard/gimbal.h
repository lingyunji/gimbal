#include "stm32f4xx_hal.h"


typedef struct
{
    float Pitch;
    float Yaw;
}Gimbal_Ref_t;

//�Ƕ�����ö��
typedef enum
{
    AngleMode_REL,          //��ԽǶ�
    AngleMode_ABS,           //���ԽǶ�
	AngleMode_ECD
}AngleMode_Enum;

typedef enum
{
    PID_ABS,			//ֱ����Ϊ���
    PID_SPEED,			//Yaw�����ݵ����ٶȻ�
	PID_POSITION		//����λ�û�
}PIDMode_Enum;

typedef enum
{
	Gimbal_Prepare,
	Gimbal_Stop,
	Gimbal_RC_Mode,
	Gimbal_Mouse_Mode,
	Gimbal_Auto,//����
	Gimbal_Standby,
	Gimbal_Debug,
}Gimbal_MoveMode_t;

typedef enum
{
	GMAutoPrepare,
	GMAutoReady,
	GMAutoSearch,
	GMAutoCatch,
	GMAutoShooting,
	GMAutoSlowDown,
	GMAutoCheck,
	GMAutoDefence,
}Gimbal_AutoMode_t;


void Gimbal_YawAngleSet(float Target, AngleMode_Enum mode);
void Gimbal_PitchAngleSet(float Target, AngleMode_Enum mode);
void GMCalLoop(PIDMode_Enum PIDMode);
void SetGimbalMotorOutput();

 
