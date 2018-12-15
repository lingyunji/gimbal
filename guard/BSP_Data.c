#include "BSP_Data.h"

CIRCLE_BUFF_t Que_JudgeFrame = {0, 0, {0}};

CIRCLE_BUFF_t Que_Protocol_Tx  = {0, 0, {0}};
CAN_CIRCLE_BUFF_t Que_CAN1_Tx = {0};
CAN_CIRCLE_BUFF_t Que_CAN2_Tx = {0};

/**
* @brief �Ӷ�����ȡ��һ��ֵ
* @param CIRCLE_BUFF_t* buffer  ����ָ��
* @param unsigned char* _buf    �������ݵĵ�ַ
* @param uint8_t num			Ҫȡ���ĸ���
* @retval int 					ʧ��	-1
								�ɹ�	����ȡ����ֵ
*/ 
int bufferPop(CIRCLE_BUFF_t* buffer, unsigned char* _buf)
{
    if(buffer->head_pos==buffer->tail_pos)        //���ͷβ�Ӵ���ʾ������Ϊ��
	{
        *_buf=0xFF;
		return -1;
	}
	else
    {
        *_buf = buffer->circle_buffer[buffer->head_pos];    //����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
        if(++buffer->head_pos >= BUFFER_MAX)
            buffer->head_pos=0;
		return *_buf;
    }
}
/**
* @brief �Ӷ�����ȡ�����ֵ
* @param CIRCLE_BUFF_t* buffer  ����ָ��
* @param unsigned char* _buf    �������ݵĵ�ַ
* @param uint8_t num			Ҫȡ���ĸ���
* @retval unsigned char 		���ȡ����һ����
*/ 
unsigned char buffer_multiPop(CIRCLE_BUFF_t* buffer, unsigned char* rx_buf, uint16_t num)
{
    for(int i = 0; i<num; i++)
	{
		if(bufferPop(buffer, &rx_buf[i]) == -1)
			break;
	}
	return rx_buf[num-1];
}
/**
* @brief д�����ݵ�������
* @param CIRCLE_BUFF_t* buffer  ����ָ��
* @param const unsigned char _buf    Ҫд������ݵĵ�ַ
* @retval void
*/ 
void bufferPush(CIRCLE_BUFF_t* buffer, const unsigned char _buf)
{   
    buffer->circle_buffer[buffer->tail_pos]=_buf; //��β��׷��
    if(++buffer->tail_pos>=BUFFER_MAX)           //β�ڵ�ƫ��
	{
        buffer->tail_pos=0;                      //����������󳤶� ���� �γɻ��ζ���
    }
	if(buffer->tail_pos==buffer->head_pos)    //���β���ڵ�׷��ͷ���ڵ� ���޸�ͷ�ڵ�ƫ��λ�ö�����������
    {
		if(++buffer->head_pos>=BUFFER_MAX)
        {
			buffer->head_pos=0;
		}
	}
}
/**
* @brief ��ȡ�������ݳ���
* @param CIRCLE_BUFF_t* buffer  ����ָ��
* @retval uint8_t				����	
*/ 
uint16_t bufferlen(CIRCLE_BUFF_t* buffer)
{
	int len = buffer->tail_pos-buffer->head_pos;
	if(len >=0)
		return len;
	else
		return len + BUFFER_MAX;
}



//------------------------------------CAN���Ͷ���-------------------------------------------
/**
* @brief �Ӷ�����ȡ��һ��ֵ
* @param CIRCLE_BUFF_t* buffer  ����ָ��
* @param unsigned char* _buf    �������ݵĵ�ַ
* @param uint8_t num			Ҫȡ���ĸ���
* @retval int 					ʧ��	-1
								�ɹ�	����ȡ����ֵ
*/ 
int CAN_bufferPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* _buf)
{
    if(buffer->head_pos==buffer->tail_pos)        //���ͷβ�Ӵ���ʾ������Ϊ��
	{
		CanTxMsgTypeDef que_temp = {0, 0, 0};
        *_buf= que_temp;
		return -1;
	}
	else
    {
        *_buf = buffer->circle_buffer[buffer->head_pos];    //����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
        if(++buffer->head_pos >= CAN_BUFFER_MAX)
            buffer->head_pos=0;
		return 0;
    }
}


