/*
 * gm65.h
 *
 *  Created on: 16 dic. 2021
 *      Author: Gonzalo Mansilla
 */

#ifndef INC_GM65_H_
#define INC_GM65_H_



#endif /* INC_GM65_H_ */

#include "stm32f4xx_hal.h"


//Tipos de Comandos para UART: Read; Write; Save EEPROM
//Types of Command for UART: Read; Write; Save EEPROM

#define	READ 													0x07
#define WRITE													0x08
#define SAVE_EEPROM												0x09


#define address0												0x0000
	#define	ad0_LED_successfully_read_on						0x80 //1000 0000
	#define	ad0_LED_successfully_read_off						0x00 //1000 0000
	#define ad0_mute_off										0x40 //0100 0000
	#define ad0_mute_on											0x00 //0000 0000
	#define ad0_aim_off											0x00 //0000 0000
	#define ad0_aim_standart									0x10 //0001 0000
	#define ad0_aim_always_on									0x20 //0010 0000
	#define ad0_light_off										0x00 //0000 0000
	#define ad0_light_standart									0x04 //0000 0100
	#define ad0_light_always_on									0x08 //0000 1000
	#define	ad0_manual_mode										0x00 //0000 0000
	#define	ad0_command_triggered_mode							0x01 //0000 0001
	#define	ad0_continuous_mode									0x02 //0000 0010
	#define	ad0_inductive_mode									0x03 //0000 0011

#define address2												0x0002
	#define ad2_output_successfully_decode_prompt 				0x40//0100 0000
	#define ad2_no_output_successfully_decode_prompt			0x00//0000 0000

#define address3												0x0003
	#define ad3_disable_setup_code								0x02 //0000 0010
	#define ad3_enable_setup_code								0x00 //0000 0000
	#define ad3_output_contents_of_setup_code					0x01 //0000 0001
	#define ad3_no_output_contents_of_setup_code				0x00 //0000 0000

#define address4												0x0004 //time for imagine stabilization 0 - 25.5s

#define address5												0x0005 //time for read interval	0 - 25.5s

#define address6												0x0006 //time for single read

#define address7							 					0x0007 //deepsleep
	#define ad7_enable_deepsleep								0x80 //1000 0000
//bit 6-0 is bit 14-8 of idle time (unit: 100ms )

#define address8												0x0008 //bit 7-0 of idle time

#define address9												0x0009//image flip
	#define ad9_enable_image_flip								01
	#define ad9_disable_image_flip								00	//bit 1-0 are image flip

#define address10												0x000A
	#define ad10_piezo_buzzer									0

#define address11												0x000B//time for successfully read sound 0-255ms

#define address12												0x000C
	#define ad12_capslock_on									1<<1 //0000 0001
	#define ad12_capslock_off									0    //0000 0000
	#define ad12_buzzer_level_low_idle							1 //0000 0001
	#define ad12_buzzer_level_high_idle							0 //0000 0000

#define address13												0x000D



void gm65_init(UART_HandleTypeDef *huart); //prepara el modulo para trabajar con el dato
void gm65_config(UART_HandleTypeDef *huart, uint8_t type, uint16_t address, uint8_t content);
void gm65_start_decode(UART_HandleTypeDef *huart);
void gm65_capture(UART_HandleTypeDef *huart);
uint16_t gm65_crc(uint8_t* ptr, unsigned int len);
