#include "stm32f4xx_hal.h"

typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;	//��������������ı�����ֵ
	int32_t ecd_relative;
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	//int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
	float ecd_distance;						//���־��루��λcm��
}Encoder;


extern CAN_HandleTypeDef hcan1;
extern float pitch_abs;
extern float yaw_abs;
extern int32_t pitch_diff;
extern int32_t yaw_diff;
extern int32_t pitch_relative;
extern int32_t yaw_relative;
extern volatile Encoder GMYawEncoder ;
extern volatile Encoder GMPitchEncoder ;


void Motor_CAN_Send();
uint16_t Motor_CAN_Receive(void);
uint8_t yaw_count(void);
uint8_t pitch_count(void);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef* hcan);
void Motor_cailbration(void);
void Motor_Init(void);
void control(void);
void gimbal_send(int16_t pitch,int16_t yaw);
void GetEncoderBias(volatile Encoder *v, uint8_t *msg);