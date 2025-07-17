/**
使锟斤拷注锟斤拷锟斤拷锟斤拷:
    1.锟斤拷tjc_usart_hmi.c锟斤拷tjc_usart_hmi.h 锟街憋拷锟诫工锟斤拷
    2.锟斤拷锟斤拷要使锟矫的猴拷锟斤拷锟斤拷锟节碉拷头锟侥硷拷锟斤拷锟斤拷锟� #include "tjc_usart_hmi.h"
    3.使锟斤拷前锟诫将 HAL_UART_Transmit_IT() 锟斤拷锟斤拷锟斤拷锟斤拷锟轿�锟斤拷牡锟狡�锟斤拷锟侥达拷锟节凤拷锟酵碉拷锟街节猴拷锟斤拷
    3.TJCPrintf锟斤拷printf锟矫凤拷一锟斤拷

*/

#include "main.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "tjc_usart_hmi.h"
#include <stddef.h>
#include "usart.h"

#define STR_LENGTH 100

typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
}RingBuff_t;

RingBuff_t ringBuff;	//锟斤拷锟斤拷一锟斤拷ringBuff锟侥伙拷锟斤拷锟斤拷
uint8_t RxBuff[1];



/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	TJCPrintf
锟斤拷锟竭ｏ拷    	wwd
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟津串口达拷印锟斤拷锟斤拷,使锟斤拷前锟诫将USART_SCREEN_write锟斤拷锟斤拷锟斤拷锟斤拷锟轿�锟斤拷牡锟狡�锟斤拷锟侥达拷锟节凤拷锟酵碉拷锟街节猴拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�		锟轿匡拷printf
锟斤拷锟斤拷值锟斤拷 		锟斤拷印锟斤拷锟斤拷锟节碉拷锟斤拷锟斤拷
锟睫改硷拷录锟斤拷
**********************************************************/

void TJCPrintf(const char *str, ...)
{


	uint8_t end = 0xff;
	char buffer[STR_LENGTH+1];  // 锟斤拷锟捷筹拷锟斤拷
	uint8_t i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, str);
	vsnprintf(buffer, STR_LENGTH+1, str, arg_ptr);
	va_end(arg_ptr);
	while ((i < STR_LENGTH) && (i < strlen(buffer)))
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)(buffer) + i++, 1);
		while(huart1.gState != HAL_UART_STATE_READY);	//锟饺达拷锟斤拷锟斤拷锟斤拷锟�
	}

	HAL_UART_Transmit_IT(&huart1, &end, 1);			//锟斤拷锟斤拷锟斤拷锟斤拷锟轿�锟斤拷牡锟狡�锟斤拷锟侥达拷锟节凤拷锟酵碉拷锟街节猴拷锟斤拷
	while(huart1.gState != HAL_UART_STATE_READY);	//锟饺达拷锟斤拷锟斤拷锟斤拷锟�
	HAL_UART_Transmit_IT(&huart1, &end, 1);			//锟斤拷锟斤拷锟斤拷锟斤拷锟轿�锟斤拷牡锟狡�锟斤拷锟侥达拷锟节凤拷锟酵碉拷锟街节猴拷锟斤拷
	while(huart1.gState != HAL_UART_STATE_READY);	//锟饺达拷锟斤拷锟斤拷锟斤拷锟�
	HAL_UART_Transmit_IT(&huart1, &end, 1);			//锟斤拷锟斤拷锟斤拷锟斤拷锟轿�锟斤拷牡锟狡�锟斤拷锟侥达拷锟节凤拷锟酵碉拷锟街节猴拷锟斤拷
	while(huart1.gState != HAL_UART_STATE_READY);	//锟饺达拷锟斤拷锟斤拷锟斤拷锟�

}




/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	HAL_UART_RxCpltCallback
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟斤拷锟节斤拷锟斤拷锟叫讹拷,锟斤拷锟斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷写锟诫环锟轿伙拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷值锟斤拷 		void
锟睫改硷拷录锟斤拷
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)	// 锟叫讹拷锟斤拷锟斤拷锟侥革拷锟斤拷锟节达拷锟斤拷锟斤拷锟叫讹拷
	{
		writeRingBuff(RxBuff[0]);
		HAL_UART_Receive_IT(&huart1,RxBuff,1);		// 锟斤拷锟斤拷使锟杰达拷锟斤拷2锟斤拷锟斤拷锟叫讹拷
	}
}



/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	initRingBuff
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟斤拷始锟斤拷锟斤拷锟轿伙拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷值锟斤拷 		void
锟睫改硷拷录锟斤拷
**********************************************************/
void initRingBuff(void)
{
  //锟斤拷始锟斤拷锟斤拷锟斤拷锟较�
  ringBuff.Head = 0;
  ringBuff.Tail = 0;
  ringBuff.Lenght = 0;
}



/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	writeRingBuff
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟斤拷锟斤拷锟轿伙拷锟斤拷锟斤拷写锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷值锟斤拷 		void
锟睫改硷拷录锟斤拷
**********************************************************/
void writeRingBuff(uint8_t data)
{
  if(ringBuff.Lenght >= RINGBUFF_LEN) //锟叫断伙拷锟斤拷锟斤拷锟角凤拷锟斤拷锟斤拷
  {
    return ;
  }
  ringBuff.Ring_data[ringBuff.Tail]=data;
  ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//锟斤拷止越锟斤拷欠锟斤拷锟斤拷锟�
  ringBuff.Lenght++;

}




/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	deleteRingBuff
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	删锟斤拷锟斤拷锟节伙拷锟斤拷锟斤拷锟斤拷锟斤拷应锟斤拷锟饺碉拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�		要删锟斤拷锟侥筹拷锟斤拷
锟斤拷锟斤拷值锟斤拷 		void
锟睫改硷拷录锟斤拷
**********************************************************/
void deleteRingBuff(uint16_t size)
{
	if(size >= ringBuff.Lenght)
	{
	    initRingBuff();
	    return;
	}
	for(int i = 0; i < size; i++)
	{

		if(ringBuff.Lenght == 0)//锟叫断非匡拷
		{
		initRingBuff();
		return;
		}
		ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//锟斤拷止越锟斤拷欠锟斤拷锟斤拷锟�
		ringBuff.Lenght--;

	}

}



/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	read1BFromRingBuff
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟接达拷锟节伙拷锟斤拷锟斤拷锟斤拷取1锟街斤拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�		position:锟斤拷取锟斤拷位锟斤拷
锟斤拷锟斤拷值锟斤拷 		锟斤拷锟斤拷位锟矫碉拷锟斤拷锟斤拷(1锟街斤拷)
锟睫改硷拷录锟斤拷
**********************************************************/
uint8_t read1BFromRingBuff(uint16_t position)
{
	uint16_t realPosition = (ringBuff.Head + position) % RINGBUFF_LEN;

	return ringBuff.Ring_data[realPosition];
}




/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	getRingBuffLenght
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟斤拷取锟斤拷锟节伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷值锟斤拷 		锟斤拷锟节伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
锟睫改硷拷录锟斤拷
**********************************************************/
uint16_t getRingBuffLenght()
{
	return ringBuff.Lenght;
}


/********************************************************
锟斤拷锟斤拷锟斤拷锟斤拷  	isRingBuffOverflow
锟斤拷锟竭ｏ拷
锟斤拷锟节ｏ拷    	2022.10.08
锟斤拷锟杰ｏ拷    	锟叫断伙拷锟轿伙拷锟斤拷锟斤拷锟角凤拷锟斤拷锟斤拷
锟斤拷锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷值锟斤拷 		1:锟斤拷锟轿伙拷锟斤拷锟斤拷锟斤拷锟斤拷 , 2:锟斤拷锟轿伙拷锟斤拷锟斤拷未锟斤拷
锟睫改硷拷录锟斤拷
**********************************************************/
uint8_t isRingBuffOverflow()
{
	return ringBuff.Lenght == RINGBUFF_LEN;
}



