#include "IOTask.h"
#include "bsp_flash.h"
#include "q6623.h"
#include "imu.h"

#define LOST_ERROR_FLASH								30u	//flash calidata save error
#define LOST_ERROR_GIMBAL_YAW							7u	//
#define LOST_ERROR_GIMBAL_PITCH							8u		//
#define LOST_ERROR_IMU									1u		//mpu6050 error
#define GIMBAL_BIAS_YAW  	7530//3400//1510//1510			//633
#define GIMBAL_BIAS_PITCH 	 7042//3180//6880			//4391
uint32_t lost_err = 0;     //每一位代表一个错误

void GetGimbalCaliData(GimbalCaliStruct_t *cali_data);
void GetGyroCaliData(GyroCaliStruct_t *cali_data);
void GetAccCaliData(AccCaliStruct_t *cali_data);
void GetMagCaliData(MagCaliStruct_t *cali_data);
CALI_STATE_e CameraSaveToFlash(AppParam_t *cali_data);
void ResetCaliCmdFlag(uint32_t flag);
/* Variables -----------------------------------------------------------------*/
uint8_t IsCaliCmdFlagSet(uint32_t flag);

CALI_STATE_e GimbalCaliProcess(void);
CALI_STATE_e GyroCaliProcess(void);
CALI_STATE_e MagStartCaliProcess(void);
CALI_STATE_e MagEndCaliProcess(void);
CALI_STATE_e LittleGyroCaliProcess(void);


//读取calidata保存到gAppParamStruct中，并写入到Flash中
void SetGimbalCaliData(GimbalCaliStruct_t *cali_data);
void SetGyroCaliData(GyroCaliStruct_t *cali_data);
void SetAccCaliData(AccCaliStruct_t *cali_data);
void SetMagCaliData(MagCaliStruct_t *cali_data);
//set or reset the Cali Cmd flag

AppParam_t gAppParamStruct;					//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步

static GyroCaliStruct_t GyroCaliData;        //保存校准陀螺仪偏差值
static GimbalCaliStruct_t  GimbalCaliData;   //保存云台编码器偏差值
static MagCaliStruct_t  MagCaliData;         //保存磁力计校准值
AppParam_t PIDCaliData;  //保存pitch轴position校准值
AppParam_t AppParamRealUsed; 
//这几个变量用于在实际程序中应用


uint8_t app_param_calied_flag = 0;
uint8_t Gyro_Cali_State = 0;
uint8_t Gyro_CaliDone = 0;

uint8_t Is_Lost_Error_Set(uint32_t index)
{
	if((lost_err &(1 << index)) != 0)return 1;
	else							return 0;
}
//用于在SuperviseTask中设置错误标志位
uint8_t Is_AppParam_Calied(void)
{
	return app_param_calied_flag;    //param未初始化
}
void Set_Error_Flag(uint32_t index)
{
	lost_err |= (uint32_t)(1<<index);
}

//重置错误位
void Reset_Error_Flag(uint32_t index)
{
	lost_err &= (uint32_t)~(1<<index);
}
//用于保存数据到flash中
static uint8_t AppParamSave(void)
{
    uint8_t retval = 1;   
    retval = BSP_FLASH_Write(PARAM_SAVED_START_ADDRESS, (uint8_t *)&gAppParamStruct, sizeof(AppParam_t));    
    if(retval == 0)
    {
		Set_Error_Flag(LOST_ERROR_FLASH);
    }
	else
	{
		Reset_Error_Flag(LOST_ERROR_FLASH);
	}
    return retval;   
}

//用于从flash中读取校准数据
void AppParamInit(void)
{
    AppParam_t tmp_param;
    
    memcpy(&tmp_param, (void *)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));		//读取flash中的数据
    
//    if((PARAM_SAVED_FLAG == tmp_param.ParamSavedFlag) &&\
//		(PARAM_CALI_DONE == tmp_param.GimbalCaliData.GimbalCaliFlag) &&\
//		(PARAM_CALI_DONE == tmp_param.GyroCaliData.GyroCaliFlag))
//	    if((PARAM_SAVED_FLAG == tmp_param.ParamSavedFlag) &&\
//		(PARAM_CALI_DONE == tmp_param.GyroCaliData.GyroCaliFlag))
	{
		app_param_calied_flag =1;
        memcpy(&gAppParamStruct, &tmp_param, sizeof(AppParam_t));
    }
//    else
//    {
//		app_param_calied_flag = 0;
//        gAppParamStruct.FirmwareVersion = 0; //保留未使用
//        gAppParamStruct.ParamSavedFlag = PARAM_SAVED_FLAG;
//    }
    //if not calied before the flag is NONE the init the para with default value
    if(gAppParamStruct.GimbalCaliData.GimbalCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.GimbalCaliData.GimbalCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.GimbalCaliData.GimbalPitchOffset = 0;
        gAppParamStruct.GimbalCaliData.GimbalYawOffset = 0;
    }
    
    if(gAppParamStruct.GyroCaliData.GyroCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.GyroCaliData.GyroCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.GyroCaliData.GyroXOffset = 0;
        gAppParamStruct.GyroCaliData.GyroYOffset = 0;
        gAppParamStruct.GyroCaliData.GyroZOffset = 0;
    }
    
    if(gAppParamStruct.AccCaliData.AccCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.AccCaliData.AccCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.AccCaliData.AccXOffset = 0;
        gAppParamStruct.AccCaliData.AccYOffset = 0;
        gAppParamStruct.AccCaliData.AccZOffset = 0;
        gAppParamStruct.AccCaliData.AccXScale = 1.0;
        gAppParamStruct.AccCaliData.AccYScale = 1.0;
        gAppParamStruct.AccCaliData.AccZScale = 1.0;
    }
    
    if(gAppParamStruct.MagCaliData.MagCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.MagCaliData.MagCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.MagCaliData.MagXOffset = 0;
        gAppParamStruct.MagCaliData.MagYOffset = 0;
        gAppParamStruct.MagCaliData.MagZOffset = 0;
        gAppParamStruct.MagCaliData.MagXScale = 1.0;
        gAppParamStruct.MagCaliData.MagYScale = 1.0;
        gAppParamStruct.MagCaliData.MagZScale = 1.0;
    }
}

void SetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GimbalCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}


void SetGyroCaliData(GyroCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GyroCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}  

void SetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(&gAppParamStruct.AccCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
    }
}

void SetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.MagCaliData, cali_data, sizeof(*cali_data));   //step1: copy data to struct
		AppParamSave();	
    }
																														 //step2:write data to the flash
}

//PID offset data saved in the memory 
void SetPIDCaliData(AppParam_t *cali_data)
{
	if(cali_data != NULL)
    {	
//		memcpy(&gAppParamStruct.PitchPositionPID, &cali_data->PitchPositionPID, sizeof(cali_data->PitchPositionPID));
//		memcpy(&(gAppParamStruct.PitchSpeedPID), &(cali_data->PitchSpeedPID), sizeof((cali_data->PitchSpeedPID)));
//		memcpy(&(gAppParamStruct.YawPositionPID), &(cali_data->YawPositionPID), sizeof((cali_data->YawPositionPID)));
//		memcpy(&(gAppParamStruct.YawSpeedPID), &(cali_data->YawSpeedPID), sizeof((cali_data->YawSpeedPID)));
//		memcpy(&(gAppParamStruct.RotateSpeedPID), &(cali_data->RotateSpeedPID), sizeof((cali_data->RotateSpeedPID)));
//		memcpy(&(gAppParamStruct.ChassisSpeedPID), &(cali_data->ChassisSpeedPID), sizeof((cali_data->ChassisSpeedPID)));
		memcpy(&(cali_data->GimbalCaliData), &gAppParamStruct.GimbalCaliData, sizeof(gAppParamStruct.GimbalCaliData));//防止calidata里面的空值影响
		memcpy(&(cali_data->CamaraOffset), &gAppParamStruct.CamaraOffset, sizeof(gAppParamStruct.CamaraOffset));
		memcpy(&(cali_data->GyroCaliData), &gAppParamStruct.GyroCaliData, sizeof(gAppParamStruct.GyroCaliData));
		memcpy(&gAppParamStruct, cali_data, sizeof(AppParam_t));
			
		 AppParamSave();	
	 }
}

void GetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.GimbalCaliData, sizeof(GimbalCaliStruct_t));
    }
}

void GetGyroCaliData(GyroCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.GyroCaliData, sizeof(GyroCaliStruct_t));
    }
}

void GetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.AccCaliData, sizeof(AccCaliStruct_t));
    }
}

void GetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.MagCaliData, sizeof(MagCaliStruct_t));
    }
}

uint8_t IsGimbalCalied(void)
{
    return (gAppParamStruct.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsGyroCalied(void)
{
    return (gAppParamStruct.GyroCaliData.GyroCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsAccCalied(void)
{
    return (gAppParamStruct.AccCaliData.AccCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsMagCalied(void)
{
    return (gAppParamStruct.MagCaliData.MagCaliFlag == PARAM_CALI_DONE);
}



static uint32_t CaliCmdFlagGrp = 0;     

void SetCaliCmdFlag(uint32_t flag)  //设置校准标志位
{
	CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag(uint32_t flag)
{
	CaliCmdFlagGrp &= ~flag;
}

uint32_t GetCaliCmdFlagGrp()
{
	return CaliCmdFlagGrp;
}


uint8_t IsCaliCmdFlagSet(uint32_t flag)
{
	if(flag & CaliCmdFlagGrp)
	{
		return 1;
	}else
	{
		return 0;	
	}
}



CALI_STATE_e CameraSaveToFlash(AppParam_t *cali_data)
{
	if(cali_data!=NULL)
	{	
		memcpy(&gAppParamStruct.CamaraOffset, &(cali_data->CamaraOffset), sizeof(gAppParamStruct.CamaraOffset));
		AppParamSave();
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

//保存到中间变量
CALI_STATE_e CameraSaveToMiddleman(CameraOffset_t *cali_data)
{
	if(cali_data!=NULL)
	{	
		memcpy(&PIDCaliData.CamaraOffset, cali_data, sizeof(gAppParamStruct.CamaraOffset));
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

CALI_STATE_e  GimbalCaliProcess()     //返回校准状态   ERROR DONE
{
	static uint32_t loopCount = 0;
	static uint32_t loopTime = 10;
	static int32_t pitchSum = 0;
	static int32_t yawSum = 0;
	
	if(Is_Lost_Error_Set(LOST_ERROR_GIMBAL_YAW) || Is_Lost_Error_Set(LOST_ERROR_GIMBAL_PITCH))
	{
		return CALI_STATE_ERR;
	}
	else if(loopCount++<loopTime)   //in cali state
	{
		pitchSum += GMPitchEncoder.raw_value;
		yawSum += GMYawEncoder.raw_value;
		return CALI_STATE_IN;
	}
	else
	{		
		GimbalCaliData.GimbalPitchOffset = pitchSum/loopTime;   //读取pitch轴陀螺仪作为偏差
	    GimbalCaliData.GimbalYawOffset = yawSum/loopTime;		//读取yaw轴陀螺仪作为偏差
		GimbalCaliData.GimbalCaliFlag = PARAM_CALI_DONE;
		pitchSum = 0;
		yawSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}	
}


//给转接板校准使用
uint32_t Gyro_Cali_Time, Gyro_Time_Flag = 1, Gyro_Cali_In = 0;
CALI_STATE_e LittleGyroCaliProcess()
{
	if (Gyro_Time_Flag) //首次进入计时
	{
		Gyro_Cali_Time = Get_Time_Micros();
		Gyro_Time_Flag = 0;
	}
	if (Gyro_Cali_In != 1)
	{
		//if (Set_IMU_Calibrate() == HAL_OK)
			Gyro_Cali_In = 1;
	}
	else if (Gyro_CaliDone)
	{
//		GyroCaliData.GyroXOffset = IMU9250_Data.Cali_Gyro_Pitch;
//		GyroCaliData.GyroYOffset = IMU9250_Data.Cali_Gyro_Roll;
//		GyroCaliData.GyroZOffset = IMU9250_Data.Cali_Gyro_Yaw;
//		GyroCaliData.GyroCaliFlag = PARAM_CALI_DONE;
//		Gyro_CaliDone = 0;
//		Gyro_Time_Flag = 1;
//		Gyro_Cali_In = 0;
//		return CALI_STATE_DONE;
	}
	if (Get_Time_Micros() - Gyro_Cali_Time < 10000000)
	{
		return CALI_STATE_IN;
	}
	else //10秒超时
	{
		Gyro_Time_Flag = 1;
		Gyro_Cali_In = 0;
		return CALI_STATE_DONE;
	}
}

//给此板子校准陀螺仪
CALI_STATE_e  GyroCaliProcess()     
{
//	int16_t temp[6] = {0};
	static uint16_t loopCount = 0;
	static uint16_t loopTime = 100;			//累计20次算平均值
	static int32_t gyroXSum = 0;
	static int32_t gyroYSum = 0;
	static int32_t gyroZSum = 0;
	//将gyro值清零,如此得到的才是原始值
//	GyroSavedCaliData.GyroXOffset = 0;
//	GyroSavedCaliData.GyroYOffset = 0;
//	GyroSavedCaliData.GyroZOffset = 0;	
	AppParamRealUsed.GyroCaliData.GyroXOffset = 0;
	AppParamRealUsed.GyroCaliData.GyroYOffset = 0;
	AppParamRealUsed.GyroCaliData.GyroZOffset = 0;
	
	if(Is_Lost_Error_Set(LOST_ERROR_IMU))    //
	{
		return CALI_STATE_ERR;
	}
	else if(loopCount++<loopTime)   
	{
	//	MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
	gyroXSum += mpu_data.gx;
	gyroYSum += mpu_data.gy;
	gyroZSum += mpu_data.gz;
	return CALI_STATE_IN;
	}
	else
	{					
		GyroCaliData.GyroXOffset = gyroXSum/loopTime;   
	    GyroCaliData.GyroYOffset = gyroYSum/loopTime;
		GyroCaliData.GyroZOffset = gyroZSum/loopTime;	
		GyroCaliData.GyroCaliFlag = PARAM_CALI_DONE;
		gyroXSum = 0;
		gyroYSum = 0;
		gyroZSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}
}



CALI_STATE_e  MagStartCaliProcess()
{	
//	MagMaxMinData.MaxMagX = -4096;	//将原来的标定值清除
//	MagMaxMinData.MaxMagY = -4096;
//	MagMaxMinData.MaxMagZ = -4096;
//	MagMaxMinData.MinMagX = 4096;
//	MagMaxMinData.MinMagY = 4096;
//	MagMaxMinData.MinMagZ = 4096;
	return CALI_STATE_DONE;	
}
CALI_STATE_e  MagEndCaliProcess()
{
	if(Is_Lost_Error_Set(LOST_ERROR_IMU))    
	{
		return CALI_STATE_ERR;
	}
	else
	{
//		MagCaliData.MagXOffset = (MagMaxMinData.MaxMagX + MagMaxMinData.MinMagX)/2;
//		MagCaliData.MagYOffset = (MagMaxMinData.MaxMagY + MagMaxMinData.MinMagY)/2;
//		MagCaliData.MagZOffset = (MagMaxMinData.MaxMagZ + MagMaxMinData.MinMagZ)/2;
		MagCaliData.MagXScale = 1.0;
		MagCaliData.MagYScale = 1.0;
		MagCaliData.MagZScale = 1.0;	
		MagCaliData.MagCaliFlag = PARAM_CALI_DONE;
		return CALI_STATE_DONE;		
	}	
}


CALI_STATE_e PIDCaliProcess(AppParam_t *cali_data)
{
	if(cali_data!=NULL)
	{
		memcpy(&PIDCaliData, cali_data, sizeof(*cali_data));
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

void Sensor_Offset_Param_Init(AppParam_t *appParam)
{
//	memcpy(&MagSavedCaliData, &(appParam->MagCaliData), sizeof((appParam->MagCaliData)));
//	memcpy(&GyroSavedCaliData, &(appParam->GyroCaliData), sizeof((appParam->GyroCaliData)));
//	memcpy(&GimbalSavedCaliData, &(appParam->GimbalCaliData), sizeof((appParam->GimbalCaliData)));
//	
//	memcpy(&PitchPositionSavedPID, &(appParam->PitchPositionPID), sizeof((appParam->PitchPositionPID)));
//	memcpy(&PitchSpeedSavedPID, &(appParam->PitchSpeedPID), sizeof((appParam->PitchSpeedPID)));
//	memcpy(&YawPositionSavedPID, &(appParam->YawPositionPID), sizeof((appParam->YawPositionPID)));
//	memcpy(&YawSpeedSavedPID, &(appParam->YawSpeedPID), sizeof((appParam->YawSpeedPID)));
//	memcpy(&RotateSpeedSavedPID, &(appParam->RotateSpeedPID), sizeof((appParam->RotateSpeedPID)));
//	memcpy(&ChassisSpeedSavedPID, &(appParam->ChassisSpeedPID), sizeof((appParam->ChassisSpeedPID)));
	
	memcpy(&AppParamRealUsed, appParam, sizeof(AppParam_t));

 #if GimbalOffset_From
	GMPitchEncoder.ecd_bias = AppParamRealUsed.GimbalCaliData.GimbalPitchOffset;
	GMYawEncoder.ecd_bias = AppParamRealUsed.GimbalCaliData.GimbalYawOffset;	
 #else
	GMPitchEncoder.ecd_bias = GIMBAL_BIAS_PITCH;
	GMYawEncoder.ecd_bias = GIMBAL_BIAS_YAW;		
 #endif
}

void CalibrateLoop(void)
{
    CALI_STATE_e cali_result;    
	
	//大主控板陀螺仪校准
	if(IsCaliCmdFlagSet(CALI_FLAG_GYRO))  		
    {
        cali_result = GyroCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGyroCaliData(&GyroCaliData);  			//把得到的值更新到 gAppParamStruct 里面 并将gAppParamStruct烧进flash里
			Sensor_Offset_Param_Init(&gAppParamStruct); //将apparamStruct的数据更新到GyroSavedCaliData
			ResetCaliCmdFlag(CALI_FLAG_GYRO);		
		}
    }
	//转接板陀螺仪校准
	else if(IsCaliCmdFlagSet(CALI_FLAG_LITTLEGYRO))
	{
		cali_result = LittleGyroCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			ResetCaliCmdFlag(CALI_FLAG_LITTLEGYRO);
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			ResetCaliCmdFlag(CALI_FLAG_LITTLEGYRO);		
		}
	}
	//云台校准校准
	else if(IsCaliCmdFlagSet(CALI_FLAG_GIMBAL))
	{
	    cali_result = GimbalCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			ResetCaliCmdFlag(CALI_FLAG_GIMBAL);
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGimbalCaliData(&GimbalCaliData);          
			Sensor_Offset_Param_Init(&gAppParamStruct);  
			ResetCaliCmdFlag(CALI_FLAG_GIMBAL);
		}	
	}
	//磁力计校准
	else if(IsCaliCmdFlagSet(CALI_START_FLAG_MAG))
	{
		cali_result = MagStartCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{			
			ResetCaliCmdFlag(CALI_START_FLAG_MAG);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_MAG))
	{
		cali_result = MagEndCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetMagCaliData(&MagCaliData);                 
			Sensor_Offset_Param_Init(&gAppParamStruct);   
			ResetCaliCmdFlag(CALI_END_FLAG_MAG);
		}		
	}
	//PID校准
	else if(IsCaliCmdFlagSet(CALI_FLAG_PID))
	{
		SetPIDCaliData(&PIDCaliData);                 
		Sensor_Offset_Param_Init(&gAppParamStruct);   
		ResetCaliCmdFlag(CALI_FLAG_PID);
	}
	//相机原点校准
	else if(IsCaliCmdFlagSet(CALI_FLAG_CAMERA))
	{
		CameraSaveToFlash(&PIDCaliData);                   
		Sensor_Offset_Param_Init(&gAppParamStruct);   
		ResetCaliCmdFlag(CALI_FLAG_CAMERA);
	}
	else if(IsCaliCmdFlagSet(CALI_FLAG_LITTLEGYRO))
	{
		ResetCaliCmdFlag(CALI_FLAG_LITTLEGYRO);
	}

		
}
