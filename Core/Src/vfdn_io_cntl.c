/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_io_cntl.c
  * @brief          : VFDN auto making IO Control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "vfdn_io_cntl.h"
#include "dwt_stm32_delay.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void readID(void);
static unsigned int readSignal(unsigned char ch);
void outSignal(unsigned char ch, unsigned int sdata);
void readSignalProcess(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

uint16_t GPIO_INPUT_PIN_DEF[8] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_13,
    GPIO_PIN_7,
    GPIO_PIN_8,
};

static unsigned int readSignal(unsigned char ch)
{
	uint32_t buf;

	HAL_GPIO_WritePin(GPIOC, GPIO_INPUT_PIN_DEF[ch], RESET); // input [ch] enable low
    DWT_Delay_us(READ_US_DLY);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET); // input clock high
    DWT_Delay_us(READ_US_DLY);

	buf = GPIOE->IDR;	// GPIO input array

    DWT_Delay_us(READ_US_DLY);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET); // input clock low

	HAL_GPIO_WritePin(GPIOC, GPIO_INPUT_PIN_DEF[ch], SET); // input [ch] enable high

	return	(buf&0x00ff);
}

void readSignalProcess(void)
{
    uint8_t read_port_data = 0;
	uint8_t ch_num;
	
	for (ch_num = 0; ch_num< IN_CH_MAX; ch_num++)
	{
		read_port_data = (uint8_t) readSignal(ch_num);	// select channel signal read

		if (read_port_data != IN_PORT_DATA[ch_num].data)
		{
			//printf("pre Data -> CH[%d]: 0x%2x\n", ch_num, IN_PORT_DATA[ch_num].data);
			//printf("cur Data -> CH[%d]: 0x%2x\n", ch_num, read_port_data);
			IN_PORT_DATA[ch_num].data = read_port_data;

			send2PC(COMMAND_IN,ch_num,read_port_data);
		}
	}
}




void send2PC(uint8_t command,uint8_t   ch_num, uint8_t read_port_data)
{
	uint8_t send_buf[7];
	send_buf[0] = COM_STX1;
	send_buf[1] = COM_STX2;
	send_buf[2] = command;				//command 0x01 in_data 0x02 out_data
	send_buf[3] = ch_num;
	send_buf[4] = read_port_data;
	send_buf[5] = COM_ETX1;
	send_buf[6] = COM_ETX2;
	HAL_NVIC_DisableIRQ(USART3_IRQn); //Rx Callback 함수 Disable
	HAL_UART_Transmit(&huart3,(uint8_t *)send_buf,sizeof(send_buf),10);
	HAL_NVIC_EnableIRQ(USART3_IRQn);  //Rx callback 함수 enable
}




uint16_t GPIO_OUTPUT_PIN_DEF[8] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_12,
    GPIO_PIN_8,
    GPIO_PIN_9,
};


void outSignal(unsigned char ch, unsigned int sdata)
{
    sdata <<= 8;
	GPIOE->ODR = sdata;
	DWT_Delay_us(1);

	HAL_GPIO_WritePin(GPIOB, GPIO_OUTPUT_PIN_DEF[ch],SET);
	DWT_Delay_us(READ_US_DLY);
	//OUT_CLK1_LOW;
	HAL_GPIO_WritePin(GPIOB,GPIO_OUTPUT_PIN_DEF[ch],RESET);
}

void outportSignal(uint8_t out_port_name, uint8_t data)
{
	uint8_t ch_num = out_port_name/8; //3
	uint8_t pin_num = out_port_name%8; //4
	uint8_t prev_port_data = OUT_PORT_DATA[ch_num].data;
	
    //printf("pre data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
    if (data == 1)
        OUT_PORT_DATA[ch_num].data  |= (1 << pin_num%8);
    else if(data == 0)
        OUT_PORT_DATA[ch_num].data  &= ~(1 << pin_num%8);
	else if(data == EXCLUSIVE)
		OUT_PORT_DATA[ch_num].data	^= (1 << pin_num%8);
	
	if (prev_port_data != OUT_PORT_DATA[ch_num].data)
	{
		//printf("pre data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
		outSignal(ch_num, OUT_PORT_DATA[ch_num].data);
		send2PC(COMMAND_OUT,ch_num,OUT_PORT_DATA[ch_num].data);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


