#include "stm32f4xx_hal.h"
#include "bsp_uart.h"
#include "string.h"
#include "stdlib.h"
#include "BSP_Data.h"
#include "minipc.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

uint8_t dbus_buf[DBUS_BUFLEN];
rc_info_t rc;
#define UART6__RX_BUF_LEN 80
uint8_t UART6_RXBUFF[UART6__RX_BUF_LEN];

static int uart_receive_dma_no_it(UART_HandleTypeDef *huart,uint8_t* pData,uint8_t Size)
{
	uint32_t tmp1=0;
	tmp1=huart->RxState;
	if(tmp1 ==HAL_UART_STATE_READY)
	{
		if((pData==NULL)||(Size==0))
		{
			return HAL_ERROR ;
		}
		
		huart->pRxBuffPtr=pData;
		huart->RxXferSize=Size;
		huart->ErrorCode=HAL_UART_ERROR_NONE;
		
		HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
		
		SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
		
		return HAL_OK ;
	}
	else
	{
		return HAL_BUSY ;
	}
}


uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
	return ((uint16_t )(dma_stream->NDTR));
}

void rc_callback_handler(rc_info_t *rc,uint8_t *buff)
{
	 rc->ch1 = ((int16_t)buff[0] | (int16_t)buff[1] << 8) & 0x07FF;
  rc->ch1 -= (int16_t)1024u;
  rc->ch2 = ((int16_t)buff[1] >> 3 |(int16_t) buff[2] << 5) & 0x07FF;
  rc->ch2 -= (int16_t)1024u;
  rc->ch3 = ((int16_t)buff[2] >> 6 |(int16_t) buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= (int16_t)1024u;
  rc->ch4 = ((int16_t)buff[4] >> 1 | (int16_t)buff[5] << 7) & 0x07FF;
  rc->ch4 -= (int16_t)1024u;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch4) >200) || \
	    (abs(rc->ch3)>300))
  {
    memset(rc, 0, sizeof(rc_info_t));
  }		
}


static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	
	if(huart==&DBUS_HUART)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		if((DBUS_MAX_LEN-dma_current_data_counter (huart ->hdmarx->Instance))==DBUS_BUFLEN )
		{
			
			rc_callback_handler (&rc,dbus_buf);
			
		}
		
		__HAL_DMA_SET_COUNTER (huart ->hdmarx,DBUS_MAX_LEN);
		__HAL_DMA_ENABLE (huart ->hdmarx);
	}
//	if(huart==&huart6)
//	{
//		__HAL_DMA_DISABLE(huart->hdmarx);
//		uint16_t RX6_Length = UART6__RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
//			if(RX6_Length < UART6__RX_BUF_LEN)
//		{	
//			for(int i = 0; i<RX6_Length; i++)
//			{
//				bufferPush(&Que_Mainifold, UART6_RXBUFF[i]);	//����ѹ�����
//			}
//		}
//		__HAL_DMA_SET_COUNTER (huart ->hdmarx,80);
//		__HAL_DMA_ENABLE (huart ->hdmarx);
//		Mainifold_Res_Task();
//	}
	
}


void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)&&
		__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart );
	}
}


void uart_init(UART_HandleTypeDef *huart,uint8_t* pData,uint8_t Size)
{
	  __HAL_UART_CLEAR_IDLEFLAG(huart);
		 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
	
	  uart_receive_dma_no_it (huart,pData ,Size);
}