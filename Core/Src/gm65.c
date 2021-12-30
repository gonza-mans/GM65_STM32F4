/*
 * gm65.c
 *
 *  Created on: 16 dic. 2021
 *      Author: Gonzalo Mansilla
 */

#include "stm32f4xx_hal.h"


//CRC-CCITT (XModem)
uint16_t gm65_crc(uint8_t* ptr, unsigned int len)
{
unsigned int crc = 0;
while(len-- != 0)
{
for(unsigned char i = 0x80; i != 0; i >>= 1)
{
crc <<= 1;
if((crc&0x10000) !=0)
crc ^= 0x11021;
if((*ptr&i) != 0)
crc ^= 0x1021;
}
ptr++;
}
return crc;
}



void gm65_init(UART_HandleTypeDef *huart){
	uint8_t buff[9] = { 0x7E, 0,
						0x08,
						0x01,
						0x00, 0x03,
						0x03,
						0x11, 0xA9};
	uint8_t dato[9];
	HAL_StatusTypeDef status;
	uint8_t flag;
	do{
		do{
			status=HAL_UART_Transmit(huart, buff, 9, HAL_MAX_DELAY);
			//if(status == HAL_OK) break;
		}while(status != HAL_OK);

		do{
			HAL_UART_Receive(huart, dato, 7, 2000);
		}while(status != HAL_OK);

		if(dato[0]==0x20 && dato[1]==0x00 && dato[2]==0x00){
			flag=0;
		}
		else flag=1;
	}while(flag!=0);

}

void gm65_start_decode(UART_HandleTypeDef *huart){

	uint8_t buff[9] = { 0x7E, 0,
						0x08,
						0x01,
						0x00, 0x03,
						0xFF,
						0x11, 0xA9};
	uint8_t dato[9];
	HAL_StatusTypeDef status;
	uint8_t flag;
	do{
		do{
			status=HAL_UART_Transmit(huart, buff, 9, HAL_MAX_DELAY);
			//if(status == HAL_OK) break;
		}while(status != HAL_OK);

		do{
			HAL_UART_Receive(huart, dato, 7, 2000);
		}while(status != HAL_OK);

		if(dato[0]==0x20 && dato[1]==0x00 && dato[2]==0x00){
			flag=0;
		}
		else flag=1;
	}while(flag!=0);
}
