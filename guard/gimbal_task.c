#include "gimbal_task.h"
#include "gimbal.h"
#include "PID.h"
#include "imu.h"
#include "q6623.h"
#include "bsp_uart.h"
#include "stdlib.h"
#include "angle.h"
#include "minipc.h"


extern PID_Regulator_t GMPPositionPID;
extern PID_Regulator_t GMPSpeedPID ;
extern PID_Regulator_t GMYPositionPID ;
extern PID_Regulator_t GMYSpeedPID ;
extern Gimbal_Ref_t GimbalRef;

Smooth_t GMYawSmooth=SMOOTH_GMINIT;
Smooth_t GMPitchSmooth=SMOOTH_GMINIT;

void Ref_Gimbal_Prepare(void)
{
	Gimbal_PitchAngleSet(-GMPitchEncoder.ecd_angle , AngleMode_REL);
	Gimbal_YawAngleSet(SmoothSigmoid(Angle_Optimize(-GMYawEncoder.ecd_angle,0),2000,&GMYawSmooth), AngleMode_REL);
  GMCalLoop(PID_POSITION);
	gimbal_send( 0,0);
	//gimbal_send( -(int16_t)GMYSpeedPID.output,0);
}


void Gimbal_control()
{
	Gimbal_YawAngleSet(-yaw_abs ,AngleMode_REL);
	Gimbal_PitchAngleSet(-pitch_abs ,AngleMode_REL);
	GMCalLoop(PID_POSITION);
	SetGimbalMotorOutput();
}

void Ref_UpdataFromRCStick(void)
{
	
		Gimbal_PitchAngleSet(GimbalRef.Pitch + (rc.ch4) * abs(rc.ch4) / 660.0 * STICK_TO_PITCH_ANGLE_INC_FACT, AngleMode_ABS);
		Gimbal_YawAngleSet(GimbalRef.Yaw + (rc.ch3) * abs((rc.ch3)) / 660.0 * STICK_TO_YAW_ANGLE_INC_FACT, AngleMode_ABS); //������
//		GimbalRef.Yaw = RC_CtrlData.rc.ch2/660.0f*60;
		GMCalLoop(PID_POSITION);
	 SetGimbalMotorOutput();
}

void Gimbal_shoot(void)
{
	Gimbal_YawAngleSet(Angle_Optimize(yaw_angle, MiniPC_Data.Yaw_Real), AngleMode_REL);
    Gimbal_PitchAngleSet(MiniPC_Data.Pitch_Real, AngleMode_ABS);
    GMCalLoop(PID_POSITION);
}
void Gimbal_free(void)
{
	
}