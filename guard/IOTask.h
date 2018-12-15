#ifndef  __IOTASK_H
#define  __IOTASK_H

#include "stm32f4xx_hal.h"

#define VERSION_A								1u
#define VERSION_B								6u
#define VERSION_C								4u
#define VERSION_D								0u
#define VERSION									(VERSION_A<<24)|(VERSION_B<<16)|(VERSION_C<<8)|(VERSION_D)

#define PARAM_SAVED_START_ADDRESS 								ADDR_FLASH_SECTOR_11


#define PID_TYPE_POSITION   0x00
#define PID_TYPE_SPEED 		0x01
#define PID_TYPE_CIRCUIT    0x02

#define MOTOR_TYPE_PITCH 	0x00
#define MOTOR_TYPE_YAW 		0x01
#define MOTOR_TYPE_201		0x02
#define MOTOR_TYPE_202		0x03
#define MOTOR_TYPE_203		0x04
#define MOTOR_TYPE_204		0x05
//enum the cali result

typedef struct PID_OFFSET_DATA
{
	int16_t pitch_position_kp;
	int16_t pitch_position_ki;
	int16_t pitch_position_kd;
	
	int16_t pitch_speed_kp;
	int16_t pitch_speed_ki;
	int16_t pitch_speed_kd;
	
	int16_t yaw_position_kp;
	int16_t yaw_position_ki;
	int16_t yaw_position_kd;
	
	int16_t yaw_speed_kp;
	int16_t yaw_speed_ki;
	int16_t yaw_speed_kd;
}PID_OFFSET_DATA;

typedef enum
{
    CALI_STATE_ERR,
    CALI_STATE_IN,
    CALI_STATE_DONE,
}CALI_STATE_e;

typedef struct Version
{
		uint8_t A;   
		uint8_t B;	
		uint8_t C;	
		uint8_t D;	
}Version;

typedef __packed struct
{
	float yaw;
	float pitch;
}CameraOffset_t;


#define VERSION_DEFAULT	\
{\
	1,\
	6,\
	2,\
	0,\
}\

#define PARAM_SAVED_FLAG                            0x5A   
#define PARAM_CALI_DONE                             0x5A 		
#define PARAM_CALI_NONE                             0x00

#define CALI_FLAG_GYRO                    ((uint32_t)1<<2)
#define CALI_START_FLAG_ACC                   ((uint32_t)1<<3)
#define CALI_START_FLAG_MAG                   ((uint32_t)1<<4)
#define CALI_END_FLAG_MAG                     ((uint32_t)1<<5)
#define CALI_FLAG_GIMBAL                  	  ((uint32_t)1<<7)
#define CALI_FLAG_PID         				  ((uint32_t)1<<8)
#define CALI_FLAG_PITCH_SPEED_PID             ((uint32_t)1<<9)
#define CALI_FLAG_YAW_POSITION_PID            ((uint32_t)1<<10)
#define CALI_FLAG_YAW_SPEED_PID               ((uint32_t)1<<11)
#define CALI_FLAG_CAMERA         			  ((uint32_t)1<<12)
#define CALI_FLAG_LITTLEGYRO         		  ((uint32_t)1<<13)

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitchOffset;
    uint8_t     GimbalCaliFlag;
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef __packed struct
{
    int16_t     AccXOffset;
    int16_t     AccYOffset;
    int16_t     AccZOffset; 
    float       AccXScale;
    float       AccYScale;
    float       AccZScale;
    uint8_t     AccCaliFlag;
}AccCaliStruct_t;

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;    
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

typedef __packed struct
{
	int8_t pid_type;		// position PID
	int8_t motor_type;   //motor type ie: pitch yaw 201 202 203 204	
	int16_t kp_offset;
	int16_t ki_offset;
	int16_t kd_offset;
}PIDParamStruct_t;

typedef __packed struct 
{
    uint8_t     ParamSavedFlag;    				//header 
    uint32_t    FirmwareVersion;    			//version
    GimbalCaliStruct_t GimbalCaliData;    //gimbal pitch yaw encoder offset
    GyroCaliStruct_t   GyroCaliData;      //gyro offset data
    AccCaliStruct_t    AccCaliData;    		//ACC offset data
    MagCaliStruct_t    MagCaliData;				//Mag offset data
	PIDParamStruct_t   PitchPositionPID;
	PIDParamStruct_t   PitchSpeedPID;
	PIDParamStruct_t   YawPositionPID;
	PIDParamStruct_t   YawSpeedPID;
	PIDParamStruct_t   RotateSpeedPID;
	PIDParamStruct_t   ChassisSpeedPID;
	PIDParamStruct_t	TurntableSpeedPID;
	PIDParamStruct_t	TurntablePositionPID;	
	PIDParamStruct_t	FrictionWheelMaxSpeed;
	int16_t		TurntableSpeed_target;
	CameraOffset_t		CamaraOffset;	
}AppParam_t;
//上传数据的类型
typedef enum
{
	REIMU = 1,
	REMOV = 2,
	REHMC = 3,
	REOFFSET = 4,
	REVERSION = 5,
	REERROR =6,
	REPID =7,
}UploadParamType_e;

extern GimbalCaliStruct_t GimbalSavedCaliData;    //gimbal pitch yaw encoder offset
extern GyroCaliStruct_t   GyroSavedCaliData;      //gyro offset data
extern AccCaliStruct_t    AccSavedCaliData;    		//ACC offset data
extern MagCaliStruct_t    MagSavedCaliData;				//Mag offset data

extern PIDParamStruct_t PitchPostionCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t PitchSpeedCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t YawPositionCaliData;  //保存pitch轴position校准值
extern PIDParamStruct_t YawSpeedCaliData;  //保存pitch轴position校准值

extern PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
extern PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
extern PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
extern PIDParamStruct_t YawSpeedSavedPID;        	//PID offset data
extern PIDParamStruct_t RotateSpeedSavedPID;        	    //PID offset data
extern PIDParamStruct_t ChassisSpeedSavedPID;        	    //PID offset data

extern AppParam_t gAppParamStruct;
extern AppParam_t AppParamRealUsed;

void AppParamInit(void);
//读取校准参数并应用
void Sensor_Offset_Param_Init(AppParam_t *appParam);

CALI_STATE_e PIDCaliProcess(AppParam_t *cali_data);			//上位机数据保存到中间变量

CALI_STATE_e CameraSaveToMiddleman(CameraOffset_t *cali_data);

//校正云台/陀螺仪/加速计
void SetCaliCmdFlag(uint32_t flag);

//是否正在校准
uint32_t GetCaliCmdFlagGrp(void);
//是否校正
uint8_t IsGimbalCalied(void);
uint8_t IsGyroCalied(void);
uint8_t IsAccCalied(void);
uint8_t IsMagCalied(void);
uint8_t Is_AppParam_Calied(void);

void CalibrateLoop(void);
#endif
