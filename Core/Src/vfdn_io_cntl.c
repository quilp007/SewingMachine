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
#include "vfdn_sub.h"
#include "dwt_stm32_delay.h"
#include "vfdn_com.h"

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
unsigned char	scFg = 0;	// input scan complete flag
unsigned char	rsFg = 0;	// read signal timer flag
unsigned int	rsCnt = 0;	// read signal timer counter
unsigned char	inCh = 0;	// input channel

// AV SERVO INPUT STATUS
/*--------------------------------------------
|0x01 : SERVO ALARM1~4
|0x02 : SERVO HOME1~4
|0x04 : SERVO READY1~4
|0x08 : SERVO ZERO SPEED1~4
|0x10 : SERVO TPOSITION1~4
|0x20 : RESERVED
|0x40 : RESERVED
|0x80 : RESERVED
---------------------------------------------*/
unsigned char	stSVin[4];	// Servo motor input status1~4

// AC SERVO OUTPUT STATUS0~3
/*--------------------------------------------
|0x01 : SERVO ON1~4
|0x02 : SERVO ALARM RESET1~4
|0x04 : SERVO EMERGENCY STOP1~4
|0x08 : SERVO COUNTER CLEAR1~4
|0x10 : SERVO TORQUE MODE LSB1~4
|0x20 : SERVO TORQUE MODE MSB1~4
|0x40 : SERVO CLOCK WISE LIMITE1~4
|0x80 : SERVO COUNTER CLOCK WISE LIMITE1~4
---------------------------------------------*/
// OUTPUT STATUS4
/*--------------------------------------------
|0x01 : CLAMP1 OPEN/CLOSE(1:close, 0:open)
|0x02 : CLAMP2 OPEN/CLOSE(1:close, 0:open)
|0x04 : CLAMP3 OPEN/CLOSE(1:close, 0:open)
|0x08 : CLAMP4 OPEN/CLOSE(1:close, 0:open)
|0x10 : VACCUM OPEN/CLOSE(0:close, 1:open)
|0x20 : RESERVED
|0x40 : RCPWM +5V??????
|0x80 : BAND CUT HEAT
---------------------------------------------*/
// OUTPUT STATUS5
/*--------------------------------------------
|0x01 : RESERVED
|0x02 : RESERVED
|0x04 : TOWER LAMP R(1:on, 0:off)
|0x08 : TOWER LAMP G(1:on, 0:off)
|0x10 : TOWER LAMP Y(1:on, 0:off)
|0x20 : RESERVED
|0x40 : RESERVED
|0x80 : RESERVED
---------------------------------------------*/
// OUTPUT STATUS6
/*--------------------------------------------
|0x01 : RESERVED
|0x02 : RESERVED
|0x04 : RESERVED
|0x08 : RESERVED
|0x10 : RESERVED
|0x20 : RESERVED
|0x40 : RESERVED
|0x80 : RESERVED
---------------------------------------------*/
// OUTPUT STATUS7
/*--------------------------------------------
|0x01 : RESERVED
|0x02 : RESERVED
|0x04 : RESERVED
|0x08 : RESERVED
|0x10 : RESERVED
|0x20 : RESERVED
|0x40 : RESERVED
|0x80 : RESERVED
---------------------------------------------*/
unsigned char	stOut[8];	// Servo motor output status1~4 and out signal

// SWITCH INPUT SWITCH STATUS
/*--------------------------------------------
|0x00000001 : SW EMS
|0x00000002 : SW AUTO/MANUAL
|0x00000004 : SW MOVE FORWARD/BACKWARD
|0x00000008 : SW MOVE UP/DOWN
|0x00000010 : SW AUTO START
|0x00000020 : SW AUTO STOP
|0x00000040 : SW AUTO RESET
|0x00000080 : SW TEST SEWING
|0x00000100 : SW NEEDLE HOME
|0x00000200 : SW LOOPER HOME
|0x00000400 : SW MOVING HOME
|0x00000800 : SW UP/DOWN HOME
|0x00001000 : SW CLAMP1 OPEN CLOSE
|0x00002000 : SW CLAMP2 OPEN CLOSE
|0x00004000 : SW CLAMP3 OPEN CLOSE
|0x00008000 : SW CLAMP4 OPEN CLOSE
|0x00010000 : SW VACCUM OPEN CLOSE
|0x00020000 : SW BAND CUD
---------------------------------------------*/
unsigned long	stSWin = 0;

// SENSOR INPUT SENSOR STATUS
/*--------------------------------------------
|0x00000001 : SENSOR LIFTING UP LIMITE
|0x00000002 : SENSOR LIFTING DOWN LIMITE
|0x00000004 : SENSOR LIFTING HOME
|0x00000008 : SENSOR NIDDLE HOME
|0x00000010 : SENSOR BAND(LEFT) EMPTY1
|0x00000020 : SENSOR BAND(RIGHT) EMPTY2
|0x00000040 : SENSOR BAND(LEFT) LOST1
|0x00000080 : SENSOR BAND(RIGHT) EMPTY2
|0x00000100 : SENSOR UPPER THREAD CUT OFF
|0x00000200 : SENSOR LOOPER HOME
|0x00000400 : SENSOR UNDER THREAD CUT OFF
|0x00000800 : SENSOR BAND(LEFT) INSERT1
|0x00001000 : SENSOR BAND(RIGHT) INSERT2
|0x00002000 : SENSOR MOVING FORWARD LIMITE
|0x00004000 : SENSOR MOVING BACKWARD LIMITE
|0x00008000 : SENSOR MOVING HOME
|0x00010000 : SENSOR FABRIC DETECT1(LEFT)
|0x00020000 : SENSOR FABRIC DETECT2(RIGHT)
|0x00040000 : SENSOR FABRIC CLAMP OPEN(LEFT-RIGHT AND)
|0x00080000 : SENSOR FABRIC CLAMP CLOSE(LEFT-RIGHT AND)
|0x00100000 : SENSOR BAND CLAMP OPEN(LEFT-RIGHT AND)
|0x00200000 : SENSOR BAND CLAMP CLOSE(LEFT-RIGHT AND)
---------------------------------------------*/
unsigned long	stSENin = 0;


#ifdef __DEBUG_IO__
unsigned long   stSWPrev = 255;
unsigned long   stSENPrev = 255;
unsigned char   stSVPrev[4];
#endif
/* USER CODE BEGIN PV */
unsigned int   buff=0;
unsigned char   chan=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void readID(void);
static unsigned int readSignal(unsigned char ch);
void outSignal(unsigned char ch, unsigned int sdata);
void readSignalProcess(void);
void chkError(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief Board ID read function
  * @param None
  * @retval None
  */
void readID(void)
{
	unsigned int buf=0;

	// ID:0 - sewing machine
	// ID:1 - reserved
	//	:
	// ID:15 - reserved

	buf = GPIOD->IDR;	// GPIO input array

	buf >>= 4;

	bdID = (unsigned char)(buf & 0x000f);
}

/**
  * @brief Board ID read function
  * @param None
  * @retval None
  */
void outTest(void)
{
    if(rsFg == READ_SIG_NOW)
    {
        outSignal(chan,buff);

        chan++;

        if(chan == 8)
        {
            chan = 0;

            buff++;

            if(buff == 0x0100)    buff = 0;
        }
        
        rsCnt = 99;
        rsFg = READ_SIG_TSTART;
    }
}

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

	//buf = 0b1010101010101010;

	//if (!SW_AUTO_MANUAL) printf("sw_auto_manual : 0");


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

		if (read_port_data != IN_PORT_DATA[ch_num].data){
			printf("pre Data -> CH[%d]: 0x%2x\n", ch_num, IN_PORT_DATA[ch_num].data);
			printf("cur Data -> CH[%d]: 0x%2x\n", ch_num, read_port_data);
			IN_PORT_DATA[ch_num].data = read_port_data;
		}
	}

    //scFg = 1;
 
    //if(sysStartFg == 0) sysStartFg = 1;

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
	
    //printf("pre data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
    if (data == 1)
        OUT_PORT_DATA[ch_num].data  |= (1 << pin_num%8);
    else
        OUT_PORT_DATA[ch_num].data  &= ~(1 << pin_num%8);


	outSignal(ch_num, OUT_PORT_DATA[ch_num].data);
    //printf("cur data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
}


/*


void out_control_signal(uint8_t out_port_name, uint8_t data)
{
	uint8_t ch_num = out_port_name/8; //3
	uint8_t pin_num = out_port_name%8; //4
	
    //printf("pre data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
    if (data == 1)
        OUT_PORT_DATA[ch_num]  |= (1 << pin_num%8); 
    else
        OUT_PORT_DATA[ch_num]  &= ~(1 << pin_num%8); 


	outSignal(ch_num, pin_num);
    //printf("cur data: [%d]: 0x%02x\n", out_port_name/8, OUT_PORT_DATA[out_port_name/8]);
}
*/

/**
  * @brief output signal function
  * @param channel(OUT_CLK1,OUT_CLK2,OUT_CLK3,OUT_CLK4,OUT_CLK5,OUT_CLK6,OUT_CLK7,OUT_CLK8), send data
  * @retval None
  */
  /*
void outSignal(unsigned char ch, unsigned int sdata)
{
    sdata <<= 8;
	GPIOE->ODR = sdata;
	DWT_Delay_us(1);
	
	switch(ch)
	{
		case OUT_CH1:	// SV_ON1, ALRM_RST1, EMG_STOP1, CNT_CLR1, MODE_SEL01, MODE_SEL11, CW_LIM1, CCW_LIM1
						//OUT_CLK1_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK1_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK1_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET);
			break;

		case OUT_CH2:	// SV_ON2, ALRM_RST2, EMG_STOP2, CNT_CLR2, MODE_SEL02, MODE_SEL12, CW_LIM2, CCW_LIM2
						//OUT_CLK2_LOW;
						//DWT_Delay_us(1);
						//OUT_CLK2_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK2_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,RESET);
			break;

		case OUT_CH3:	// SV_ON3, ALRM_RST3, EMG_STOP3, CNT_CLR3, MODE_SEL03, MODE_SEL13, CW_LIM3, CCW_LIM3
						//OUT_CLK3_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK3_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK3_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,RESET);
			break;

		case OUT_CH4:	// SV_ON4, ALRM_RST4, EMG_STOP4, CNT_CLR4, MODE_SEL04, MODE_SEL14, CW_LIM4, CCW_LIM4
						//OUT_CLK4_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK4_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK4_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,RESET);
			break;

		case OUT_CH5:	// OUTPUT1(FABRIC_CLAMP1), OUTPUT2(FABRIC_CLAMP2), OUTPUT3(BAND_CLAMP1), OUTPUT4(BAND_CLAMP2), OUTPUT5(RES_OUTPUT5), OUTPUT6(RES_OUTPUT6), OUTPUT7(RES_OUTPUT7), OUTPUT8(BAND_CUT_HEAT)
						//OUT_CLK5_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK5_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK5_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,RESET);
			break;

		case OUT_CH6:	// OUTPUT9(RES_OUTPUT9), OUTPUT10(RES_OUTPUT10), OUTPUT11(TL_R), OUTPUT12(TL_G), OUTPUT13(TL_Y), OUTPUT14(RES_OUTPUT14), OUTPUT15(RES_OUTPUT15), OUTPUT16(RES_OUTPUT16)
						//OUT_CLK6_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK6_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK6_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET);
			break;

		case OUT_CH7:	// OUTPUT17(RES_OUTPUT17), OUTPUT18(RES_OUTPUT18), OUTPUT19(RES_OUTPUT18), OUTPUT20(RES_OUTPUT20), OUTPUT21(RES_OUTPUT21), OUTPUT22(RES_OUTPUT22), OUTPUT23(RES_OUTPUT23), OUTPUT24(RES_OUTPUT24)
						//OUT_CLK7_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK7_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK7_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,RESET);
			break;

		case OUT_CH8:	// OUTPUT25(RES_OUTPUT25), OUTPUT26(RES_OUTPUT26), RESERVED(NC), RESERVED(NC), RESERVED(NC), RESERVED(NC), RESERVED(NC), RESERVED(NC)
						//OUT_CLK8_LOW;
						//DWT_Delay_us(READ_US_DLY);
						//OUT_CLK8_HIGH;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,SET);
						DWT_Delay_us(READ_US_DLY);
						//OUT_CLK8_LOW;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,RESET);
			break;

		default:
			break;
	}
}
*/

#if 0
/**
  * @brief read signal function
  * @param channel(IN_CLK1,IN_CLK2,IN_CLK3,IN_CLK4,IN_CLK5,IN_CLK6,IN_CLK7,IN_CLK8)
  * @retval input signal(16bit) or error
  */
static unsigned int readSignal(unsigned char ch)
{
	unsigned int buf=0;

    #if 0
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
    DWT_Delay_us(READ_US_DLY);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET); // input clock low
    DWT_Delay_us(READ_US_DLY);
    #endif

	switch(ch)
	{
		case IN_CH1:	// IN_CLK1 : SV_ALRM1, SV_HOME1, SV_RDY1, ZERO_SPD1, T_POS1, SW_IN1(SW_MOV_HOME), SW_IN2(SW_UD_HOME), SW_IN3(SW_CLAMP1_OPEN_CLOSE)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,RESET); // input en1 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
            break;

		case IN_CH2:	// IN_CLK2 : SV_ALRM2, SV_HOME2, SV_RDY2, ZERO_SPD2, T_POS2, SW_IN4(SW_CLAMP2_OPEN_CLOSE), SW_IN5(SW_CLAMP3_OPEN_CLOSE), SW_IN6(SW_CLAMP4_OPEN_CLOSE)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,RESET); // input en2 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH3:	// IN_CLK3 : SV_ALRM3, SV_HOME3, SV_RDY3, ZERO_SPD3, T_POS3, SW_IN7(SW_VACCUM_OPEN_CLOSE), SW_IN8(SW_BAND_CUT), SW_IN9(RES_INPUT1)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,RESET); // input en3 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH4:	// IN_CLK4 : SV_ALRM4, SV_HOME4, SV_RDY4, ZERO_SPD4, T_POS4, SW_IN10(RES_INPUT2), SW_IN11(RES_INPUT3), SW_IN12(RES_INPUT4)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,RESET); // input en4 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH5:	// IN_CLK5 : INPUT1(SW_EMS), INPUT2(SW_A_M), INPUT3(LIFT_UP_LIM), INPUT4(LIFT_DN_LIM), INPUT5(LIFT_HOME), INPUT6(LIFT_HOME), INPUT7(BAND1_EMPTY), INPUT8(BAND2_EMPTY)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,RESET); // input en5 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH6:	// IN_CLK6 : INPUT9(BAND1_LOST), INPUT10(BAND2_LOST), INPUT11(UP_THREAD_CUT_OFF), INPUT12(LOOPER_HOME), INPUT13(UN_THREAD_CUT_OFF), INPUT14(BAND_INSERT1), INPUT15(BAND_INSERT2), INPUT16(MOV_FORWARD_LIM)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,RESET); // input en6 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH7:	// IN_CLK7 : INPUT17(MOV_REVERSE_LIM), INPUT18(MOV_HOME), INPUT19(FABR_DETECT1), INPUT20(FABR_DETECT2), INPUT21(FABR_CLAMP_OPEN), INPUT22(FABR_CLAMP_CLOSE), INPUT23(BAND_CLAMP_OPEN), INPUT24(BAND_CLAMP_CLOSE)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,RESET); // input en6 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		case IN_CH8:	// IN_CLK8 : INPUT25(SW_MOV_FR), INPUT26(SW_MOV_UD), INPUT27(SW_AUTO_START), INPUT28(SW_AUTO_STOP), INPUT29(SW_AUTO_RESET), INPUT30(SW_TEST_SEW), INPUT31(SW_NEEDLE_HOME), INPUT32(SW_LOOPER_HOME)
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,RESET); // input en6 low
						DWT_Delay_us(READ_US_DLY);
                        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
                        //DWT_Delay_us(READ_US_DLY);
			break;

		default:
			break;
	}

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
    DWT_Delay_us(READ_US_DLY);

	buf = GPIOE->IDR;	// GPIO input array

    DWT_Delay_us(READ_US_DLY);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET); // input clock low

    if(ch == IN_CH1)        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,SET); // input EN1 high
    else if(ch == IN_CH2)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,SET); // input EN2 high
    else if(ch == IN_CH3)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,SET); // input EN3 high
    else if(ch == IN_CH4)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,SET); // input EN4 high
    else if(ch == IN_CH5)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,SET); // input EN5 high
    else if(ch == IN_CH6)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,SET); // input EN6 high
    else if(ch == IN_CH7)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,SET); // input EN7 high
    else if(ch == IN_CH8)   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET); // input EN8 high

	return	(buf&0x00ff);
}
#endif
#if 0
/**
  * @brief output signal function
  * @param channel(OUT_CLK1,OUT_CLK2,OUT_CLK3,OUT_CLK4,OUT_CLK5,OUT_CLK6,OUT_CLK7,OUT_CLK8), send data
  * @retval None
  */
void readSignalProcess(void)
{
	unsigned int buf=0;

	//if(rsFg == READ_SIG_NOW)
	//{
		buf = readSignal(inCh);	// select channel signal read

		buf &= 0x00ff;

		switch(inCh)
		{
			case IN_CH1:	// IN_CLK1 : SV_ALRM1, SV_HOME1, SV_RDY1, ZERO_SPD1, T_POS1, SW_IN1(SW_MOV_HOME), SW_IN2(SW_UD_HOME), SW_IN3(SW_CLAMP1_OPEN_CLOSE)
                            if(!(buf & SW_VACCUM_OPEN_CLOSE))	stSWin |= ST_SW_VACCUM_OPEN_CLOSE;		// vaccum select(close) switch(manual mode)
							else							    stSWin &= ~ST_SW_VACCUM_OPEN_CLOSE;		// vaccum select(open) switch(manual mode)
                break;

			case IN_CH2:	// IN_CLK2 : SV_ALRM2, SV_HOME2, SV_RDY2, ZERO_SPD2, T_POS2, SW_IN4(SW_CLAMP2_OPEN_CLOSE), SW_IN5(SW_CLAMP3_OPEN_CLOSE), SW_IN6(SW_CLAMP4_OPEN_CLOSE)
							
				break;

			case IN_CH3:	// IN_CLK3 : SV_ALRM3, SV_HOME3, SV_RDY3, ZERO_SPD3, T_POS3, SW_IN7(SW_VACCUM_OPEN_CLOSE), SW_IN8(SW_BAND_CUT), SW_IN9(RES_INPUT1)
							if(!(buf & SW_ROE_X))	stSWin |= ST_SW_ROE_X;		// ROE X 5select switch
							else					stSWin &= ~ST_SW_ROE_X;		// ROE X 5select switch

                            if(!(buf & SW_ROE_Y))	stSWin |= ST_SW_ROE_Y;		// ROE Y 5select switch
							else					stSWin &= ~ST_SW_ROE_Y;		// ROE Y 5select switch
				break;

			case IN_CH4:	// IN_CLK4 : SV_ALRM4, SV_HOME4, SV_RDY4, ZERO_SPD4, T_POS4, SW_IN10(RES_INPUT2), SW_IN11(RES_INPUT3), SW_IN12(RES_INPUT4)
							if(!(buf & SW_ROE_Z))	stSWin |= ST_SW_ROE_Z;		// ROE Z 5select switch
							else					stSWin &= ~ST_SW_ROE_Z;		// ROE Z 5select switch

                            if(!(buf & SW_ROE_X1))	stSWin |= ST_SW_ROE_X1;		// ROE X1 3select switch
							else					stSWin &= ~ST_SW_ROE_X1;    // ROE X1 3select switch

                            if(!(buf & SW_ROE_X10))	stSWin |= ST_SW_ROE_X10;    // ROE X10 3select switch
							else					stSWin &= ~ST_SW_ROE_X10;   // ROE X10 3select switch
				break;

			case IN_CH5:	// IN_CLK5 : INPUT1(SW_EMS), INPUT2(SW_A_M), INPUT3(SEN_LIFT_UP_LIM), INPUT4(SEN_LIFT_DN_LIM), INPUT5(SEN_LIFT_HOME), INPUT6(SEN_NEDDLE_HOME), INPUT7(SEN_BAND_EMPTY1), INPUT8(SEN_BAND_EMPTY2)
							if(buf & SEN_LIFTING_DN_LIM)	stSENin |= ST_SEN_LIFTING_DN_LIM;	// lifting down limite sensor on
							else							stSENin &= ~ST_SEN_LIFTING_DN_LIM;	// lifting down limite sensor off

							if(buf & SEN_LIFTING_HOME)	stSENin |= ST_SEN_LIFTING_HOME;	// lifting home sensor on
							else							stSENin &= ~ST_SEN_LIFTING_HOME;	// lifting home sensor off

                            if(buf & SEN_BAND_EMPTY1)	stSENin |= ST_SEN_BAND_EMPTY1;	// band empty1 sensor on
							else							stSENin &= ~ST_SEN_BAND_EMPTY1;	// band empty1 sensor off

							//if(buf & SEN_BAND_EMPTY2)	stSENin |= ST_SEN_BAND_EMPTY2;	// band empty2 sensor on
							//else						stSENin &= ~ST_SEN_BAND_EMPTY2;	// band empty2 sensor off

							if(buf & SEN_MOVING_HOME)	stSENin |= ST_SEN_MOVING_HOME;	// upper thread cut off check sensor on
							else						stSENin &= ~ST_SEN_MOVING_HOME;	// upper thread cut off check sensor off

                            if(buf & SEN_BAND_LOST1)	stSENin |= ST_SEN_BAND_LOST1;	// band lost1 sensor on
							else						stSENin &= ~ST_SEN_BAND_LOST1;	// band lost1 sensor off

							if(buf & SEN_BAND_LOST2)	stSENin |= ST_SEN_BAND_LOST2;	// band lost2 sensor on
							else						stSENin &= ~ST_SEN_BAND_LOST2;	// band lost2 sensor off
				break;

			case IN_CH6:	// IN_CLK6 : INPUT9(SEN_BAND_LOST1), INPUT10(SEN_BAND_LOST2), INPUT11(SEN_UP_THREAD_CUT_OFF), INPUT12(SEN_LOOPER_HOME), INPUT13(SEN_UN_THREAD_CUT_OFF), INPUT14(SEN_BAND_INSERT1), INPUT15(SEN_BAND_INSERT2), INPUT16(SEN_MOVING_BACKWARD_LIM)
							if(buf & SEN_UPPER_THREAD_CUT_OFF)	stSENin |= ST_SEN_UPPER_THREAD_CUT_OFF;	// upper thread cut off check sensor on
							else								stSENin &= ~ST_SEN_UPPER_THREAD_CUT_OFF;	// upper thread cut off check sensor off

							if(!(buf & SEN_LOOPER_HOME))	stSENin |= ST_SEN_LOOPER_HOME;	// looper home sensor on
							//if(buf & SEN_LOOPER_HOME)	stSENin |= ST_SEN_LOOPER_HOME;	// looper home sensor on
							else							stSENin &= ~ST_SEN_LOOPER_HOME;	// looper home sensor off
				break;

			case IN_CH7:	// IN_CLK7 : INPUT17(SEN_MOVING_FORWARD_LIM), INPUT18(SEN_MOVING_HOME), INPUT19(SEN_FABRIC_DETECT1), INPUT20(SEN_FABRIC_DETECT2), INPUT21(SEN_FABRIC_CLAMP_OPEN), INPUT22(SEN_FABRIC_CLAMP_CLOSE), INPUT23(SEN_BAND_CLAMP_OPEN), INPUT24(SEN_BAND_CLAMP_CLOSE)
							if(!(buf & SW_AUTO_MANUAL))	stSWin |= ST_SW_AUTO_MANUAL;	// auto/manual switch on(manual)
							else						stSWin &= ~ST_SW_AUTO_MANUAL;	// auto/manual switch off(auto)

                            if(!(buf & SW_AUTO_START))	stSWin |= ST_SW_AUTO_START;	 // auto start push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_START; // auto start push switch off(auto mode)

							if(!(buf & SW_AUTO_STOP))	stSWin |= ST_SW_AUTO_STOP;  // auto stop push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_STOP; // auto stop push switch off(auto mode)

							if(!(buf & SW_AUTO_RESET))	stSWin |= ST_SW_AUTO_RESET;  // auto reset push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_RESET; // auto reset push switch off(auto mode)

							if(!(buf & SW_TEST_SEWING))	stSWin |= ST_SW_TEST_SEWING;  // test sewing push switch on
							else						stSWin &= ~ST_SW_TEST_SEWING; // test sewing push switch off

                            if(buf & SEN_LIFTING_UP_LIM)	stSENin |= ST_SEN_LIFTING_UP_LIM;	// lifting up limite sensor on
							else							stSENin &= ~ST_SEN_LIFTING_UP_LIM;	// lifting up limite sensor off

                            if(!(buf & SEN_NEEDLE_HOME))	stSENin |= ST_SEN_NEEDLE_HOME;	// needle home sensor on
                            //if(buf & SEN_NEEDLE_HOME)	stSENin |= ST_SEN_NEEDLE_HOME;	// needle home sensor on
							else							stSENin &= ~ST_SEN_NEEDLE_HOME;	// needle home sensor off
				break;

			case IN_CH8:	// IN_CLK8 : INPUT25(SW_MOVING_FB), INPUT26(SW_MOVING_UD), INPUT27(SW_AUTO_START), INPUT28(SW_AUTO_STOP), INPUT29(SW_AUTO_RESET), INPUT30(SW_TEST_SEWING), INPUT31(SW_NEEDLE_HOME), INPUT32(SW_LOOPER_HOME)
							if(!(buf & SW_NEEDLE_HOME))	stSWin |= ST_SW_NEEDLE_HOME;  // needle home push switch on
							else						stSWin &= ~ST_SW_NEEDLE_HOME; // needle home push switch off
							
							if(!(buf & SW_LOOPER_HOME))	stSWin |= ST_SW_LOOPER_HOME;  // looper home push switch on
							else						stSWin &= ~ST_SW_LOOPER_HOME; // looper home push switch off

                            if(!(buf & SW_MOVING_HOME))	stSWin |= ST_SW_MOVING_HOME;	// moving home switch on(manual mode)
							else						stSWin &= ~ST_SW_MOVING_HOME;	// moving home switch off(manual mode)

							if(!(buf & SW_UD_HOME))	stSWin |= ST_SW_UD_HOME;		// lifting home switch on(manual mode)
							else					stSWin &= ~ST_SW_UD_HOME;		// lifting home switch off(manual mode)

							if(!(buf & SW_CLAMP1_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP1_OPEN_CLOSE;		// clamp1(left ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP1_OPEN_CLOSE;		// clamp1(left ???) select(open) switch(manual mode)

                            if(!(buf & SW_CLAMP2_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP2_OPEN_CLOSE;		// clamp2(right ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP2_OPEN_CLOSE;		// clamp2(right ???) select(open) switch(manual mode)

							if(!(buf & SW_CLAMP3_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP3_OPEN_CLOSE;		// clamp3(left ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP3_OPEN_CLOSE;		// clamp3(left ???) select(open) switch(manual mode)

							if(!(buf & SW_CLAMP4_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP4_OPEN_CLOSE;		// clamp4(right ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP4_OPEN_CLOSE;		// clamp4(right ???) select(open) switch(manual mode)
				break;
			
			default:
				break;
		}
        
		if(inCh == IN_CH_MAX)
		{
			scFg = 1;
			inCh = 0;
            
            if(sysStartFg == 0) sysStartFg = 1;
                
#ifdef __DEBUG_IO__
if(stSVPrev[IN_CH1] != stSVin[IN_CH1])
{
    ;
}

if(stSVPrev[IN_CH2] != stSVin[IN_CH2])
{
    ;
}

if(stSVPrev[IN_CH3] != stSVin[IN_CH3])
{
    ;
}

if(stSVPrev[IN_CH4] != stSVin[IN_CH4])
{
    ;
}

if(stSWPrev != stSWin)
{
    if((stSWPrev & ST_SW_EMS) != (stSWin & ST_SW_EMS))
    {
        if(stSWin & ST_SW_EMS)  putStr(&huart4,(const unsigned char *)"SW EMS LOCK\n");
        else                    putStr(&huart4,(const unsigned char *)"SW EMS UNLOCK\n");
    }
    
    if((stSWPrev & ST_SW_AUTO_MANUAL) != (stSWin & ST_SW_AUTO_MANUAL))
    {
        if(stSWin & ST_SW_AUTO_MANUAL)  putStr(&huart4,(const unsigned char *)"SW MANUAL\n");
        else                            putStr(&huart4,(const unsigned char *)"SW AUTO\n");
    }
    
    if((stSWPrev & ST_SW_AUTO_START) != (stSWin & ST_SW_AUTO_START))
    {
        if(stSWin & ST_SW_AUTO_START)   putStr(&huart4,(const unsigned char *)"SW AUTO START ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW AUTO START OFF\n");
    }

    if((stSWPrev & ST_SW_AUTO_STOP) != (stSWin & ST_SW_AUTO_STOP))
    {
        if(stSWin & ST_SW_AUTO_STOP)    putStr(&huart4,(const unsigned char *)"SW AUTO STOP ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW AUTO STOP OFF\n");
    }
    
    if((stSWPrev & ST_SW_AUTO_RESET) != (stSWin & ST_SW_AUTO_RESET))
    {
        if(stSWin & ST_SW_AUTO_RESET)   putStr(&huart4,(const unsigned char *)"SW AUTO RESET ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW AUTO RESET OFF\n");
    }
    
    if((stSWPrev & ST_SW_TEST_SEWING) != (stSWin & ST_SW_TEST_SEWING))
    {
        if(stSWin & ST_SW_TEST_SEWING)  putStr(&huart4,(const unsigned char *)"SW TEST SEWING ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW TEST SEWING OFF\n");
    }
    
    if((stSWPrev & ST_SW_NEEDLE_HOME) != (stSWin & ST_SW_NEEDLE_HOME))
    {
        if(stSWin & ST_SW_NEEDLE_HOME)  putStr(&huart4,(const unsigned char *)"SW NEEDLE HOME ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW NEEDLE HOME OFF\n");
    }
    
    if((stSWPrev & ST_SW_LOOPER_HOME) != (stSWin & ST_SW_LOOPER_HOME))
    {
        if(stSWin & ST_SW_LOOPER_HOME)  putStr(&huart4,(const unsigned char *)"SW LOOPER HOME ON\n");
        else						    putStr(&huart4,(const unsigned char *)"SW LOOPER HOME OFF\n");
    }
    
    if((stSWPrev & ST_SW_MOVING_HOME) != (stSWin & ST_SW_MOVING_HOME))
    {
        if(stSWin & ST_SW_MOVING_HOME)  putStr(&huart4,(const unsigned char *)"SW MV HOME ON\n");
        else                            putStr(&huart4,(const unsigned char *)"SW MV HOME OFF\n");
    }
    
    if((stSWPrev & ST_SW_UD_HOME) != (stSWin & ST_SW_UD_HOME))
    {
        if(stSWin & ST_SW_UD_HOME)  putStr(&huart4,(const unsigned char *)"SW UD HOWE ON\n");
        else                        putStr(&huart4,(const unsigned char *)"SW UD HOWE OFF\n");
    }
    
    if((stSWPrev & ST_SW_CLAMP1_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP1_OPEN_CLOSE))
    {
        if(stSWin & ST_SW_CLAMP1_OPEN_CLOSE)    putStr(&huart4,(const unsigned char *)"SW CLAMP1 OP/CL ON\n");
        else                                    putStr(&huart4,(const unsigned char *)"SW CLAMP1 OP/CL OFF\n");
    }
    
    if((stSWPrev & ST_SW_CLAMP2_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP2_OPEN_CLOSE))
    {
        if(stSWin & ST_SW_CLAMP2_OPEN_CLOSE) putStr(&huart4,(const unsigned char *)"SW CLAMP2 OP/CL ON\n");
        else                                 putStr(&huart4,(const unsigned char *)"SW CLAMP2 OP/CL OFF\n");
    }
    
    if((stSWPrev & ST_SW_CLAMP3_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP3_OPEN_CLOSE))
    {
        if(stSWin & ST_SW_CLAMP3_OPEN_CLOSE)    putStr(&huart4,(const unsigned char *)"SW CLAMP3 OP/CL ON\n");
        else								    putStr(&huart4,(const unsigned char *)"SW CLAMP3 OP/CL OFF\n");
    }
    
    if((stSWPrev & ST_SW_CLAMP4_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP4_OPEN_CLOSE))
    {
        if(stSWin & ST_SW_CLAMP4_OPEN_CLOSE)    putStr(&huart4,(const unsigned char *)"SW CLAMP4 OP/CL ON\n");
        else                                    putStr(&huart4,(const unsigned char *)"SW CLAMP4 OP/CL OFF\n");
    }
    
    if((stSWPrev & ST_SW_VACCUM_OPEN_CLOSE) != (stSWin & ST_SW_VACCUM_OPEN_CLOSE))
    {
        if(stSWin & ST_SW_VACCUM_OPEN_CLOSE)    putStr(&huart4,(const unsigned char *)"SW VACCUM OP/CL ON\n");
        else                                    putStr(&huart4,(const unsigned char *)"SW VACCUM OP/CL OFF\n");
    }
    
    if((stSWPrev & ST_SW_BAND_CUT) != (stSWin & ST_SW_BAND_CUT))
    {
        if(stSWin & ST_SW_BAND_CUT) putStr(&huart4,(const unsigned char *)"SW BAND CUT ON\n");
        else                        putStr(&huart4,(const unsigned char *)"SW BAND CUT OFF\n");
    }

    if((stSWPrev & ST_SW_ROE_X) != (stSWin & ST_SW_ROE_X))
    {
        if(stSWin & ST_SW_ROE_X) putStr(&huart4,(const unsigned char *)"SW ROE X ON\n");
        else                     putStr(&huart4,(const unsigned char *)"SW ROE X OFF\n");
    }

    if((stSWPrev & ST_SW_ROE_Y) != (stSWin & ST_SW_ROE_Y))
    {
        if(stSWin & ST_SW_ROE_Y) putStr(&huart4,(const unsigned char *)"SW ROE Y ON\n");
        else                     putStr(&huart4,(const unsigned char *)"SW ROE Y OFF\n");
    }

    if((stSWPrev & ST_SW_ROE_Z) != (stSWin & ST_SW_ROE_Z))
    {
        if(stSWin & ST_SW_ROE_Z) putStr(&huart4,(const unsigned char *)"SW ROE Z ON\n");
        else                     putStr(&huart4,(const unsigned char *)"SW ROE Z OFF\n");
    }

	if((stSWPrev & ST_SW_ROE_X1) != (stSWin & ST_SW_ROE_X1))
    {
        if(stSWin & ST_SW_ROE_X1) putStr(&huart4,(const unsigned char *)"SW ROE x1 ON\n");
        else                      putStr(&huart4,(const unsigned char *)"SW ROE x1 OFF\n");
    }

	if((stSWPrev & ST_SW_ROE_X10) != (stSWin & ST_SW_ROE_X10))
    {
        if(stSWin & ST_SW_ROE_X10) putStr(&huart4,(const unsigned char *)"SW ROE x10 ON\n");
        else                       putStr(&huart4,(const unsigned char *)"SW ROE x10 OFF\n");
    }
	
    stSWPrev = stSWin;
}

if(stSENPrev != stSENin)
{
    if((stSENPrev & ST_SEN_LIFTING_UP_LIM) != (stSENin & ST_SEN_LIFTING_UP_LIM))
    {
        if(stSENin & ST_SEN_LIFTING_UP_LIM) putStr(&huart4,(const unsigned char *)"SEN LF UP LIM ON\n");
        else	                            putStr(&huart4,(const unsigned char *)"SEN LF UP LIM OFF\n");
    }
    
    if((stSENPrev & ST_SEN_LIFTING_DN_LIM) != (stSENin & ST_SEN_LIFTING_DN_LIM))
    {
        if(stSENin & ST_SEN_LIFTING_DN_LIM) putStr(&huart4,(const unsigned char *)"SEN LF DN LIM ON\n");
        else	                            putStr(&huart4,(const unsigned char *)"SEN LF DN LIM OFF\n");
    }
    
    if((stSENPrev & ST_SEN_LIFTING_HOME) != (stSENin & ST_SEN_LIFTING_HOME))
    {
        if(stSENin & ST_SEN_LIFTING_HOME)   putStr(&huart4,(const unsigned char *)"SEN LF HOME ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN LF HOME OFF\n");
    }
    
    if((stSENPrev & ST_SEN_NEEDLE_HOME) != (stSENin & ST_SEN_NEEDLE_HOME))
    {
        if(stSENin & ST_SEN_NEEDLE_HOME)    putStr(&huart4,(const unsigned char *)"SEN NEEDLE HOME ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN NEEDLE HOME OFF\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_EMPTY1) != (stSENin & ST_SEN_BAND_EMPTY1))
    {
        if(stSENin & ST_SEN_BAND_EMPTY1)    putStr(&huart4,(const unsigned char *)"SEN BAND1 EMPTY\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN BAND1 EMPTY NOR\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_EMPTY2) != (stSENin & ST_SEN_BAND_EMPTY2))
    {
        if(stSENin & ST_SEN_BAND_EMPTY2)    putStr(&huart4,(const unsigned char *)"SEN BAND2 EMPTY\n");
        else                                putStr(&huart4,(const unsigned char *)"SEN BAND2 EMPTY NOR\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_LOST1) != (stSENin & ST_SEN_BAND_LOST1))
    {
        if(stSENin & ST_SEN_BAND_LOST1) putStr(&huart4,(const unsigned char *)"SEN BAND1 LOST\n");
        else	                        putStr(&huart4,(const unsigned char *)"SEN BAND1 NOR\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_LOST2) != (stSENin & ST_SEN_BAND_LOST2))
    {
        if(stSENin & ST_SEN_BAND_LOST2) putStr(&huart4,(const unsigned char *)"SEN BAND2 LOST\n");
        else                            putStr(&huart4,(const unsigned char *)"SEN BAND2 NOR\n");
    }
    
    if((stSENPrev & ST_SEN_UPPER_THREAD_CUT_OFF) != (stSENin & ST_SEN_UPPER_THREAD_CUT_OFF))
    {
        if(stSENin & ST_SEN_UPPER_THREAD_CUT_OFF)   putStr(&huart4,(const unsigned char *)"SEN UP THR CUT OFF\n");
        else                                        putStr(&huart4,(const unsigned char *)"SEN UP THR NOR\n");
    }
    
    if((stSENPrev & ST_SEN_LOOPER_HOME) != (stSENin & ST_SEN_LOOPER_HOME))
    {
        if(stSENin & ST_SEN_LOOPER_HOME)    putStr(&huart4,(const unsigned char *)"SEN LOOPER HOME ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN LOOPER HOME OFF\n");
    }
    
    if((stSENPrev & ST_SEN_UNDER_THREAD_CUT_OFF) != (stSENin & ST_SEN_UNDER_THREAD_CUT_OFF))
    {
        if(stSENin & ST_SEN_UNDER_THREAD_CUT_OFF)   putStr(&huart4,(const unsigned char *)"SEN UN THR CUT OFF\n");
        else									    putStr(&huart4,(const unsigned char *)"SEN UN THR NOR\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_INSERT1) != (stSENin & ST_SEN_BAND_INSERT1))
    {
        if(stSENin & ST_SEN_BAND_INSERT1)   putStr(&huart4,(const unsigned char *)"SEN BAND1 INSERT\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN BAND1 NO INSERT\n");
    }
    
    if((stSENPrev & ST_SEN_BAND_INSERT2) != (stSENin & ST_SEN_BAND_INSERT2))
    {
        if(stSENin & ST_SEN_BAND_INSERT2)   putStr(&huart4,(const unsigned char *)"SEN BAND2 INSERT\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN BAND2 NO INSERT\n");
    }
    
    if((stSENPrev & ST_SEN_MOVING_BACKWARD_LIM) != (stSENin & ST_SEN_MOVING_BACKWARD_LIM))
    {
        if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)    putStr(&huart4,(const unsigned char *)"SEN MV BACKWARD LIM ON\n");
        else	                                    putStr(&huart4,(const unsigned char *)"SEN MV BACKWARD LIM OFF\n");
    }
    
    if((stSENPrev & ST_SEN_MOVING_FORWARD_LIM) != (stSENin & ST_SEN_MOVING_FORWARD_LIM))
    {
        if(stSENin & ST_SEN_MOVING_FORWARD_LIM) putStr(&huart4,(const unsigned char *)"SEN MV FORWARD LIM ON\n");
        else	                                putStr(&huart4,(const unsigned char *)"SEN MV FORWARD LIM OFF\n");
    }
    
    if((stSENPrev & ST_SEN_MOVING_HOME) != (stSENin & ST_SEN_MOVING_HOME))
    {
        if(stSENin & ST_SEN_MOVING_HOME)    putStr(&huart4,(const unsigned char *)"SEN MV HOME ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN MV HOME OFF\n");
    }
    
    if((stSENPrev & ST_SEN_FABRIC_DETECT1) != (stSENin & ST_SEN_FABRIC_DETECT1))
    {
        if(stSENin & ST_SEN_FABRIC_DETECT1) putStr(&huart4,(const unsigned char *)"SEN FABR1 DET ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN FABR1 DET OFF\n");
    }
    
    if((stSENPrev & ST_SEN_FABRIC_DETECT2) != (stSENin & ST_SEN_FABRIC_DETECT2))
    {
        if(stSENin & ST_SEN_FABRIC_DETECT2) putStr(&huart4,(const unsigned char *)"SEN FABR2 DET ON\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN FABR2 DET OFF\n");
    }
    
    if((stSENPrev & ST_SEN_CLAMP12_OPEN) != (stSENin & ST_SEN_CLAMP12_OPEN))
    {
        if(stSENin & ST_SEN_CLAMP12_OPEN)   putStr(&huart4,(const unsigned char *)"SEN CLAMP12 OPEN\n\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN CLAMP12 OPEN OFF\n");
    }
    
    if((stSENPrev & ST_SEN_CLAMP12_OPEN) != (stSENin & ST_SEN_CLAMP12_CLOSE))
    {
        if(stSENin & ST_SEN_CLAMP12_CLOSE)  putStr(&huart4,(const unsigned char *)"SEN CLAMP12 CLOSE\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN CLAMP12 CLOSE OFF\n");
    }
    
    if((stSENPrev & ST_SEN_CLAMP34_OPEN) != (stSENin & ST_SEN_CLAMP34_OPEN))
    {
        if(stSENin & ST_SEN_CLAMP34_OPEN)   putStr(&huart4,(const unsigned char *)"SEN CLAMP34 OPEN\n");
        else                                putStr(&huart4,(const unsigned char *)"SEN CLAMP34 OPEN OFF\n");
    }
    
    if((stSENPrev & ST_SEN_CLAMP34_CLOSE) != (stSENin & ST_SEN_CLAMP34_CLOSE))
    {
        if(stSENin & ST_SEN_CLAMP34_CLOSE)  putStr(&huart4,(const unsigned char *)"SEN CLAMP34 CLOSE\n");
        else							    putStr(&huart4,(const unsigned char *)"SEN CLAMP34 CLOSE OFF\n");
    }
	
	if((stSWPrev & ST_SEN_MOVING_HOME) != (stSWin & ST_SEN_MOVING_HOME))
    {
        if(stSWin & ST_SEN_MOVING_HOME)	putStr(&huart4,(const unsigned char *)"SEN MOV HOME ON\n");
        else                        	putStr(&huart4,(const unsigned char *)"SEN MOV HOME OFF\n");
    }

    stSENPrev = stSENin;
}
#endif
		}
		else
		{
			inCh++;
		}

		//rsCnt = READ_SIG_TCNT;
		//rsFg = READ_SIG_TSTART;
	//}
}
#endif

#if 0
/**
  * @brief output signal function
  * @param channel(OUT_CLK1,OUT_CLK2,OUT_CLK3,OUT_CLK4,OUT_CLK5,OUT_CLK6,OUT_CLK7,OUT_CLK8), send data
  * @retval None
  */
void readSignalProcess(void)
{
	unsigned int buf=0;

	//if(rsFg == READ_SIG_NOW)
	//{
		buf = readSignal(inCh);	// select channel signal read

		buf &= 0x00ff;

		switch(inCh)
		{
			case IN_CH1:	// IN_CLK1 : SV_ALRM1, SV_HOME1, SV_RDY1, ZERO_SPD1, T_POS1, SW_IN1(SW_MOV_HOME), SW_IN2(SW_UD_HOME), SW_IN3(SW_CLAMP1_OPEN_CLOSE)
							stSVin[IN_CH1] = (~buf & 0x05);	// SV_ALRM1, SV_HOME1, SV_RDY1, ZERO_SPD1, T_POS1

							if(!(buf & SW_MOVING_HOME))	stSWin |= ST_SW_MOVING_HOME;	// moving home switch on(manual mode)
							else						stSWin &= ~ST_SW_MOVING_HOME;	// moving home switch off(manual mode)

							if(!(buf & SW_UD_HOME))	stSWin |= ST_SW_UD_HOME;		// lifting home switch on(manual mode)
							else					stSWin &= ~ST_SW_UD_HOME;		// lifting home switch off(manual mode)

							if(!(buf & SW_CLAMP1_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP1_OPEN_CLOSE;		// clamp1(left ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP1_OPEN_CLOSE;		// clamp1(left ???) select(open) switch(manual mode)
				break;

			case IN_CH2:	// IN_CLK2 : SV_ALRM2, SV_HOME2, SV_RDY2, ZERO_SPD2, T_POS2, SW_IN4(SW_CLAMP2_OPEN_CLOSE), SW_IN5(SW_CLAMP3_OPEN_CLOSE), SW_IN6(SW_CLAMP4_OPEN_CLOSE)
							stSVin[IN_CH2] = (~buf & 0x05);	// SV_ALRM2, SV_HOME2, SV_RDY2, ZERO_SPD2, T_POS2

							if(!(buf & SW_CLAMP2_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP2_OPEN_CLOSE;		// clamp2(right ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP2_OPEN_CLOSE;		// clamp2(right ???) select(open) switch(manual mode)

							if(!(buf & SW_CLAMP3_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP3_OPEN_CLOSE;		// clamp3(left ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP3_OPEN_CLOSE;		// clamp3(left ???) select(open) switch(manual mode)

							if(!(buf & SW_CLAMP4_OPEN_CLOSE))	stSWin |= ST_SW_CLAMP4_OPEN_CLOSE;		// clamp4(right ???) select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_CLAMP4_OPEN_CLOSE;		// clamp4(right ???) select(open) switch(manual mode)
				break;

			case IN_CH3:	// IN_CLK3 : SV_ALRM3, SV_HOME3, SV_RDY3, ZERO_SPD3, T_POS3, SW_IN7(SW_VACCUM_OPEN_CLOSE), SW_IN8(SW_BAND_CUT), SW_IN9(RES_INPUT1)
							stSVin[IN_CH3] = (~buf & 0x05);	// SV_ALRM3, SV_HOME3, SV_RDY3, ZERO_SPD3, T_POS3

							if(!(buf & SW_VACCUM_OPEN_CLOSE))	stSWin |= ST_SW_VACCUM_OPEN_CLOSE;		// vaccum select(close) switch(manual mode)
							else								stSWin &= ~ST_SW_VACCUM_OPEN_CLOSE;		// vaccum select(open) switch(manual mode)

							if(!(buf & SW_BAND_CUT))	stSWin |= ST_SW_BAND_CUT;		// band cut switch on(manual mode)
							else						stSWin &= ~ST_SW_BAND_CUT;		// band cut switch on(manual mode)
				break;

			case IN_CH4:	// IN_CLK4 : SV_ALRM4, SV_HOME4, SV_RDY4, ZERO_SPD4, T_POS4, SW_IN10(RES_INPUT2), SW_IN11(RES_INPUT3), SW_IN12(RES_INPUT4)
							stSVin[IN_CH4] = (~buf & 0x05);	// SV_ALRM4, SV_HOME4, SV_RDY4, ZERO_SPD4, T_POS4
				break;

			case IN_CH5:	// IN_CLK5 : INPUT1(SW_EMS), INPUT2(SW_A_M), INPUT3(SEN_LIFT_UP_LIM), INPUT4(SEN_LIFT_DN_LIM), INPUT5(SEN_LIFT_HOME), INPUT6(SEN_NEDDLE_HOME), INPUT7(SEN_BAND_EMPTY1), INPUT8(SEN_BAND_EMPTY2)
							if(!(buf & SW_EMS))	stSWin |= ST_SW_EMS;	// emergency stop switch on
							else				stSWin &= ~ST_SW_EMS;	// emergency stop switch off

							if(!(buf & SW_AUTO_MANUAL))	stSWin |= ST_SW_AUTO_MANUAL;	// auto/manual switch on(manual)
							else						stSWin &= ~ST_SW_AUTO_MANUAL;	// auto/manual switch off(auto)

							if(!(buf & SEN_LIFTING_UP_LIM))	stSENin |= ST_SEN_LIFTING_UP_LIM;	// lifting up limite sensor on
							else							stSENin &= ~ST_SEN_LIFTING_UP_LIM;	// lifting up limite sensor off

							if(!(buf & SEN_LIFTING_DN_LIM))	stSENin |= ST_SEN_LIFTING_DN_LIM;	// lifting down limite sensor on
							else							stSENin &= ~ST_SEN_LIFTING_DN_LIM;	// lifting down limite sensor off

							if(!(buf & SEN_LIFTING_HOME))	stSENin |= ST_SEN_LIFTING_HOME;	// lifting home sensor on
							else							stSENin &= ~ST_SEN_LIFTING_HOME;	// lifting home sensor off

							if(!(buf & SEN_NEEDLE_HOME))	stSENin |= ST_SEN_NEEDLE_HOME;	// needle home sensor on
							else							stSENin &= ~ST_SEN_NEEDLE_HOME;	// needle home sensor off

							if(!(buf & SEN_BAND_EMPTY1))	stSENin |= ST_SEN_BAND_EMPTY1;	// band empty1 sensor on
							else							stSENin &= ~ST_SEN_BAND_EMPTY1;	// band empty1 sensor off

							if(!(buf & SEN_BAND_EMPTY2))	stSENin |= ST_SEN_BAND_EMPTY2;	// band empty2 sensor on
							else							stSENin &= ~ST_SEN_BAND_EMPTY2;	// band empty2 sensor off
				break;

			case IN_CH6:	// IN_CLK6 : INPUT9(SEN_BAND_LOST1), INPUT10(SEN_BAND_LOST2), INPUT11(SEN_UP_THREAD_CUT_OFF), INPUT12(SEN_LOOPER_HOME), INPUT13(SEN_UN_THREAD_CUT_OFF), INPUT14(SEN_BAND_INSERT1), INPUT15(SEN_BAND_INSERT2), INPUT16(SEN_MOVING_BACKWARD_LIM)
							if(!(buf & SEN_BAND_LOST1))	stSENin |= ST_SEN_BAND_LOST1;	// band lost1 sensor on
							else						stSENin &= ~ST_SEN_BAND_LOST1;	// band lost1 sensor off

							if(!(buf & SEN_BAND_LOST2))	stSENin |= ST_SEN_BAND_LOST2;	// band lost2 sensor on
							else						stSENin &= ~ST_SEN_BAND_LOST2;	// band lost2 sensor off

							if(!(buf & SEN_UPPER_THREAD_CUT_OFF))	stSENin |= ST_SEN_UPPER_THREAD_CUT_OFF;	// upper thread cut off check sensor on
							else									stSENin &= ~ST_SEN_UPPER_THREAD_CUT_OFF;	// upper thread cut off check sensor off

							if(!(buf & SEN_LOOPER_HOME))	stSENin |= ST_SEN_LOOPER_HOME;	// looper home sensor on
							else							stSENin &= ~ST_SEN_LOOPER_HOME;	// looper home sensor off

							if(!(buf & SEN_UNDER_THREAD_CUT_OFF))	stSENin |= ST_SEN_UNDER_THREAD_CUT_OFF;	// upper thread cut off check sensor on
							else									stSENin &= ~ST_SEN_UNDER_THREAD_CUT_OFF;	// upper thread cut off check sensor off

							if(!(buf & SEN_BAND_INSERT1))	stSENin |= ST_SEN_BAND_INSERT1;	// band(left???) insert check sensor on for clamp
							else							stSENin &= ~ST_SEN_BAND_INSERT1;	// band(left???) insert check sensor off for clamp

							if(!(buf & SEN_BAND_INSERT2))	stSENin |= ST_SEN_BAND_INSERT2;	// band(right???) insert check sensor on for clamp
							else							stSENin &= ~ST_SEN_BAND_INSERT2;	// band(right???) insert check sensor off for clamp
							
							if(!(buf & SEN_MOVING_BACKWARD_LIM))	stSENin |= ST_SEN_MOVING_BACKWARD_LIM;	// moving backward limit sensor on
							else									stSENin &= ~ST_SEN_MOVING_BACKWARD_LIM;	// moving backward limit sensor off
				break;

			case IN_CH7:	// IN_CLK7 : INPUT17(SEN_MOVING_FORWARD_LIM), INPUT18(SEN_MOVING_HOME), INPUT19(SEN_FABRIC_DETECT1), INPUT20(SEN_FABRIC_DETECT2), INPUT21(SEN_FABRIC_CLAMP_OPEN), INPUT22(SEN_FABRIC_CLAMP_CLOSE), INPUT23(SEN_BAND_CLAMP_OPEN), INPUT24(SEN_BAND_CLAMP_CLOSE)
							if(!(buf & SEN_MOVING_FORWARD_LIM))	stSENin |= ST_SEN_MOVING_FORWARD_LIM;	// moving forward limit sensor on
							else								stSENin &= ~ST_SEN_MOVING_FORWARD_LIM;	// moving forward limit sensor off
							
							if(!(buf & SEN_MOVING_HOME))	stSENin |= ST_SEN_MOVING_HOME;	// moving home sensor on
							else							stSENin &= ~ST_SEN_MOVING_HOME;	// moving home sensor off

							if(!(buf & SEN_FABRIC_DETECT1))	stSENin |= ST_SEN_FABRIC_DETECT1;	// fabric detect1 sensor on
							else							stSENin &= ~ST_SEN_FABRIC_DETECT1;	// fabric detect1 sensor off

							if(!(buf & SEN_FABRIC_DETECT2))	stSENin |= ST_SEN_FABRIC_DETECT2;	// fabric detect2 sensor on
							else							stSENin &= ~ST_SEN_FABRIC_DETECT2;	// fabric detect2 sensor off

							if(!(buf & SEN_CLAMP12_OPEN))	stSENin |= ST_SEN_CLAMP12_OPEN;	// clamp12 open check sensor on - Fabric
							else							stSENin &= ~ST_SEN_CLAMP12_OPEN;	// clamp12 open check sensor off - Fabric

							if(!(buf & SEN_CLAMP12_CLOSE))	stSENin |= ST_SEN_CLAMP12_CLOSE;	// clamp12 close check sensor on - Fabric
							else							stSENin &= ~ST_SEN_CLAMP12_CLOSE;	// clamp12 close check sensor off - Fabric

							if(!(buf & SEN_CLAMP34_OPEN))	stSENin |= ST_SEN_CLAMP34_OPEN;	// clamp34 open check sensor on - Band
							else							stSENin &= ~ST_SEN_CLAMP34_OPEN;	// clamp34 open check sensor off - Band
							
							if(!(buf & SEN_CLAMP34_CLOSE))	stSENin |= ST_SEN_CLAMP34_CLOSE;	// clamp34 close check sensor on - Band
							else							stSENin &= ~ST_SEN_CLAMP34_CLOSE;	// clamp34 close check sensor off - Band
				break;

			case IN_CH8:	// IN_CLK8 : INPUT25(SW_MOVING_FB), INPUT26(SW_MOVING_UD), INPUT27(SW_AUTO_START), INPUT28(SW_AUTO_STOP), INPUT29(SW_AUTO_RESET), INPUT30(SW_TEST_SEWING), INPUT31(SW_NEEDLE_HOME), INPUT32(SW_LOOPER_HOME)
							if(!(buf & SW_MOVING_FB))	stSWin |= ST_SW_MOVING_FB;  // moving forward/backward 3select switch on
							else						stSWin &= ~ST_SW_MOVING_FB; // moving forward/backward 3select switch off
							
							if(!(buf & SW_MOVING_UD))	stSWin |= ST_SW_MOVING_UD;  // moving up/down 3select switch on
							else						stSWin &= ~ST_SW_MOVING_UD; // moving up/down 3select switch off

							if(!(buf & SW_AUTO_START))	stSWin |= ST_SW_AUTO_START;	 // auto start push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_START; // auto start push switch off(auto mode)

							if(!(buf & SW_AUTO_STOP))	stSWin |= ST_SW_AUTO_STOP;  // auto stop push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_STOP; // auto stop push switch off(auto mode)

							if(!(buf & SW_AUTO_RESET))	stSWin |= ST_SW_AUTO_RESET;  // auto reset push switch on(auto mode)
							else						stSWin &= ~ST_SW_AUTO_RESET; // auto reset push switch off(auto mode)

							if(!(buf & SW_TEST_SEWING))	stSWin |= ST_SW_TEST_SEWING;  // test sewing push switch on
							else						stSWin &= ~ST_SW_TEST_SEWING; // test sewing push switch off

							if(!(buf & SW_NEEDLE_HOME))	stSWin |= ST_SW_NEEDLE_HOME;  // needle home push switch on
							else						stSWin &= ~ST_SW_NEEDLE_HOME; // needle home push switch off
							
							if(!(buf & SW_LOOPER_HOME))	stSWin |= ST_SW_LOOPER_HOME;  // looper home push switch on
							else						stSWin &= ~ST_SW_LOOPER_HOME; // looper home push switch off
				break;
			
			default:
				break;
		}
        
		if(inCh == IN_CH_MAX)
		{
			scFg = 1;
			inCh = 0;
            
            if(sysStartFg == 0) sysStartFg = 1;
                
#ifdef __DEBUG_IO__
if(stSVPrev[IN_CH1] != stSVin[IN_CH1])
{
    if((stSVPrev[IN_CH1] & SV_ALRM) != (stSVin[IN_CH1] & SV_ALRM)) putStr(&huart4,(const unsigned char *)"SERVO1 ALARM\n");
    //if((stSVPrev[IN_CH1] & SV_HOME) != (stSVin[IN_CH1] & SV_HOME)) putStr(&huart4,(const unsigned char *)"SERVO1 HOME\n");
    if((stSVPrev[IN_CH1] & SV_RDY) != (stSVin[IN_CH1] & SV_RDY)) putStr(&huart4,(const unsigned char *)"SERVO1 RDY\n");
    //if((stSVPrev[IN_CH1] & ZERO_SPD) != (stSVin[IN_CH1] & ZERO_SPD)) putStr(&huart4,(const unsigned char *)"SERVO1 ZS\n");

    stSVPrev[IN_CH1] = stSVin[IN_CH1];
}

if(stSVPrev[IN_CH2] != stSVin[IN_CH2])
{
    if((stSVPrev[IN_CH2] & SV_ALRM) != (stSVin[IN_CH2] & SV_ALRM)) putStr(&huart4,(const unsigned char *)"SERVO2 ALARM\n");
    //if((stSVPrev[IN_CH2] & SV_HOME) != (stSVin[IN_CH2] & SV_HOME)) putStr(&huart4,(const unsigned char *)"SERVO2 HOME\n");
    if((stSVPrev[IN_CH2] & SV_RDY) != (stSVin[IN_CH2] & SV_RDY)) putStr(&huart4,(const unsigned char *)"SERVO2 RDY\n");
    //if((stSVPrev[IN_CH2] & ZERO_SPD) != (stSVin[IN_CH2] & ZERO_SPD)) putStr(&huart4,(const unsigned char *)"SERVO2 ZS\n");
    
    stSVPrev[IN_CH2] = stSVin[IN_CH2];
}

if(stSVPrev[IN_CH3] != stSVin[IN_CH3])
{
    if((stSVPrev[IN_CH3] & SV_ALRM) != (stSVin[IN_CH3] & SV_ALRM)) putStr(&huart4,(const unsigned char *)"SERVO3 ALARM\n");
    //if((stSVPrev[IN_CH3] & SV_HOME) != (stSVin[IN_CH3] & SV_HOME)) putStr(&huart4,(const unsigned char *)"SERVO3 HOME\n");
    if((stSVPrev[IN_CH3] & SV_RDY) != (stSVin[IN_CH3] & SV_RDY)) putStr(&huart4,(const unsigned char *)"SERVO3 RDY\n");
    //if((stSVPrev[IN_CH3] & ZERO_SPD) != (stSVin[IN_CH3] & ZERO_SPD)) putStr(&huart4,(const unsigned char *)"SERVO3 ZS\n");
    
    stSVPrev[IN_CH3] = stSVin[IN_CH3];
}

if(stSVPrev[IN_CH4] != stSVin[IN_CH4])
{
    if((stSVPrev[IN_CH4] & SV_ALRM) != (stSVin[IN_CH4] & SV_ALRM)) putStr(&huart4,(const unsigned char *)"SERVO4 ALARM\n");
    //if((stSVPrev[IN_CH4] & SV_HOME) != (stSVin[IN_CH4] & SV_HOME)) putStr(&huart4,(const unsigned char *)"SERVO4 HOME\n");
    if((stSVPrev[IN_CH4] & SV_RDY) != (stSVin[IN_CH4] & SV_RDY)) putStr(&huart4,(const unsigned char *)"SERVO4 RDY\n");
    //if((stSVPrev[IN_CH4] & ZERO_SPD) != (stSVin[IN_CH4] & ZERO_SPD)) putStr(&huart4,(const unsigned char *)"SERVO4 ZS\n");
    
    stSVPrev[IN_CH4] = stSVin[IN_CH4];
}

if(stSWPrev != stSWin)
{
    if((stSWPrev & ST_SW_EMS) != (stSWin & ST_SW_EMS))  putStr(&huart4,(const unsigned char *)"SW EMS LOCK\n");
    else                                                putStr(&huart4,(const unsigned char *)"SW EMS UNLOCK\n");
    if((stSWPrev & ST_SW_AUTO_MANUAL) != (stSWin & ST_SW_AUTO_MANUAL))  putStr(&huart4,(const unsigned char *)"SW MANUAL\n");
    else                                                                putStr(&huart4,(const unsigned char *)"SW AUTO\n");
    if((stSWPrev & ST_SW_MOVING_FB) != (stSWin & ST_SW_MOVING_FB))    putStr(&huart4,(const unsigned char *)"SW MV FB ON\n");
    else                                                              putStr(&huart4,(const unsigned char *)"SW MV FB OFF\n");
    if((stSWPrev & ST_SW_MOVING_UD) != (stSWin & ST_SW_MOVING_UD))    putStr(&huart4,(const unsigned char *)"SW MV UD ON\n");
    else						                                       putStr(&huart4,(const unsigned char *)"SW MV UD OFF\n");
    if((stSWPrev & ST_SW_AUTO_START) != (stSWin & ST_SW_AUTO_START)) putStr(&huart4,(const unsigned char *)"SW AUTO START ON\n");
    else						                                     putStr(&huart4,(const unsigned char *)"SW AUTO START OFF\n");
    if((stSWPrev & ST_SW_AUTO_STOP) != (stSWin & ST_SW_AUTO_STOP)) putStr(&huart4,(const unsigned char *)"SW AUTO STOP ON\n");
    else						                                   putStr(&huart4,(const unsigned char *)"SW AUTO STOP OFF\n");
    if((stSWPrev & ST_SW_AUTO_RESET) != (stSWin & ST_SW_AUTO_RESET)) putStr(&huart4,(const unsigned char *)"SW AUTO RESET ON\n");
    else						                                     putStr(&huart4,(const unsigned char *)"SW AUTO RESET OFF\n");
    if((stSWPrev & ST_SW_TEST_SEWING) != (stSWin & ST_SW_TEST_SEWING)) putStr(&huart4,(const unsigned char *)"SW TEST SEWING ON\n");
    else						                                       putStr(&huart4,(const unsigned char *)"SW TEST SEWING OFF\n");
    if((stSWPrev & ST_SW_NEEDLE_HOME) != (stSWin & ST_SW_NEEDLE_HOME)) putStr(&huart4,(const unsigned char *)"SW NEEDLE HOME ON\n");
    else						                                       putStr(&huart4,(const unsigned char *)"SW NEEDLE HOME OFF\n");
    if((stSWPrev & ST_SW_LOOPER_HOME) != (stSWin & ST_SW_LOOPER_HOME)) putStr(&huart4,(const unsigned char *)"SW LOOPER HOME ON\n");
    else						                                       putStr(&huart4,(const unsigned char *)"SW LOOPER HOME OFF\n");
    if((stSWPrev & ST_SW_MOVING_HOME) != (stSWin & ST_SW_MOVING_HOME)) putStr(&huart4,(const unsigned char *)"SW MV HOME ON\n");
    else                                                               putStr(&huart4,(const unsigned char *)"SW MV HOME OFF\n");
    if((stSWPrev & ST_SW_UD_HOME) != (stSWin & ST_SW_UD_HOME)) putStr(&huart4,(const unsigned char *)"SW UD HOWE ON\n");
    else                                                       putStr(&huart4,(const unsigned char *)"SW UD HOWE OFF\n");
    if((stSWPrev & ST_SW_CLAMP1_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP1_OPEN_CLOSE)) putStr(&huart4,(const unsigned char *)"SW CLAMP1 OP/CL ON\n");
    else                                                                           putStr(&huart4,(const unsigned char *)"SW CLAMP1 OP/CL OFF\n");
    if((stSWPrev & ST_SW_CLAMP2_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP2_OPEN_CLOSE)) putStr(&huart4,(const unsigned char *)"SW CLAMP2 OP/CL ON\n");
    else								                                           putStr(&huart4,(const unsigned char *)"SW CLAMP2 OP/CL OFF\n");
    if((stSWPrev & ST_SW_CLAMP3_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP3_OPEN_CLOSE)) putStr(&huart4,(const unsigned char *)"SW CLAMP3 OP/CL ON\n");
    else								                                           putStr(&huart4,(const unsigned char *)"SW CLAMP3 OP/CL OFF\n");
    if((stSWPrev & ST_SW_CLAMP4_OPEN_CLOSE) != (stSWin & ST_SW_CLAMP4_OPEN_CLOSE)) putStr(&huart4,(const unsigned char *)"SW CLAMP4 OP/CL ON\n");
    else							  	                                           putStr(&huart4,(const unsigned char *)"SW CLAMP4 OP/CL OFF\n");
    if((stSWPrev & ST_SW_VACCUM_OPEN_CLOSE) != (stSWin & ST_SW_VACCUM_OPEN_CLOSE)) putStr(&huart4,(const unsigned char *)"SW VACCUM OP/CL ON\n");
    else								                                           putStr(&huart4,(const unsigned char *)"SW VACCUM OP/CL OFF\n");
    if((stSWPrev & ST_SW_BAND_CUT) != (stSWin & ST_SW_BAND_CUT)) putStr(&huart4,(const unsigned char *)"SW BAND CUT ON\n");
    else						                                 putStr(&huart4,(const unsigned char *)"SW BAND CUT OFF\n");

    stSWPrev = stSWin;
}

#if 0
if(stSWPrev != stSENin)
{
    if((stSENPrev & ST_SEN_LIFTING_UP_LIM) != (stSENin & ST_SEN_LIFTING_UP_LIM)) putStr(&huart4,(const unsigned char *)"SEN LF UP LIM ON\n");
    else							                                             putStr(&huart4,(const unsigned char *)"SEN LF UP LIM OFF\n");
    if((stSENPrev & ST_SEN_LIFTING_DN_LIM) != (stSENin & ST_SEN_LIFTING_DN_LIM)) putStr(&huart4,(const unsigned char *)"SEN LF DN LIM ON\n");
    else							                                             putStr(&huart4,(const unsigned char *)"SEN LF DN LIM OFF\n");
    if((stSENPrev & ST_SEN_LIFTING_HOME) != (stSENin & ST_SEN_LIFTING_HOME)) putStr(&huart4,(const unsigned char *)"SEN LF HOME ON\n");
    else							                                         putStr(&huart4,(const unsigned char *)"SEN LF HOME OFF\n");
    if((stSENPrev & ST_SEN_NEEDLE_HOME) != (stSENin & ST_SEN_NEEDLE_HOME)) putStr(&huart4,(const unsigned char *)"SEN NEEDLE HOME ON\n");
    else							                                       putStr(&huart4,(const unsigned char *)"SEN NEEDLE HOME OFF\n");
    if((stSENPrev & ST_SEN_BAND_EMPTY1) != (stSENin & ST_SEN_BAND_EMPTY1)) putStr(&huart4,(const unsigned char *)"SEN BAND1 EMPTY\n");
    else							                                       putStr(&huart4,(const unsigned char *)"SEN BAND1 EMPTY NOR\n");
    if((stSENPrev & ST_SEN_BAND_EMPTY2) != (stSENin & ST_SEN_BAND_EMPTY2)) putStr(&huart4,(const unsigned char *)"SEN BAND2 EMPTY\n");
    else							                                       putStr(&huart4,(const unsigned char *)"SEN BAND2 EMPTY NOR\n");
    if((stSENPrev & ST_SEN_BAND_LOST1) != (stSENin & ST_SEN_BAND_LOST1)) putStr(&huart4,(const unsigned char *)"SEN BAND1 LOST\n");
    else						                                         putStr(&huart4,(const unsigned char *)"SEN BAND1 NOR\n");
    if((stSENPrev & ST_SEN_BAND_LOST2) != (stSENin & ST_SEN_BAND_LOST2)) putStr(&huart4,(const unsigned char *)"SEN BAND2 LOST\n");
    else						                                         putStr(&huart4,(const unsigned char *)"SEN BAND2 NOR\n");
    if((stSENPrev & ST_SEN_UPPER_THREAD_CUT_OFF) != (stSENin & ST_SEN_UPPER_THREAD_CUT_OFF)) putStr(&huart4,(const unsigned char *)"SEN UP THR CUT OFF\n");
    else									                                                 putStr(&huart4,(const unsigned char *)"SEN UP THR NOR\n");
    if((stSENPrev & ST_SEN_LOOPER_HOME) != (stSENin & ST_SEN_LOOPER_HOME)) putStr(&huart4,(const unsigned char *)"SEN LOOPER HOME ON\n");
    else							                                       putStr(&huart4,(const unsigned char *)"SEN LOOPER HOME OFF\n");
    if((stSENPrev & ST_SEN_UNDER_THREAD_CUT_OFF) != (stSENin & ST_SEN_UNDER_THREAD_CUT_OFF)) putStr(&huart4,(const unsigned char *)"SEN UN THR CUT OFF\n");
    else									                                                 putStr(&huart4,(const unsigned char *)"SEN UN THR NOR\n");
    if((stSENPrev & ST_SEN_BAND_INSERT1) != (stSENin & ST_SEN_BAND_INSERT1)) putStr(&huart4,(const unsigned char *)"SEN BAND1 INSERT\n");
    else							                                         putStr(&huart4,(const unsigned char *)"SEN BAND1 NO INSERT\n");
    if((stSENPrev & ST_SEN_BAND_INSERT2) != (stSENin & ST_SEN_BAND_INSERT2)) putStr(&huart4,(const unsigned char *)"SEN BAND2 INSERT\n");
    else							                                         putStr(&huart4,(const unsigned char *)"SEN BAND2 NO INSERT\n");
    if((stSENPrev & ST_SEN_MOVING_BACKWARD_LIM) != (stSENin & ST_SEN_MOVING_BACKWARD_LIM)) putStr(&huart4,(const unsigned char *)"SEN MV BACKWARD LIM ON\n");
    else									                                               putStr(&huart4,(const unsigned char *)"SEN MV BACKWARD LIM OFF\n");
    if((stSENPrev & ST_SEN_MOVING_FORWARD_LIM) != (stSENin & ST_SEN_MOVING_FORWARD_LIM)) putStr(&huart4,(const unsigned char *)"SEN MV FORWARD LIM ON\n");
    else								                                                 putStr(&huart4,(const unsigned char *)"SEN MV FORWARD LIM OFF\n");
    if((stSENPrev & ST_SEN_MOVING_HOME) != (stSENin & ST_SEN_MOVING_HOME)) putStr(&huart4,(const unsigned char *)"SEN MV HOME ON\n");
    else							                                       putStr(&huart4,(const unsigned char *)"SEN MV HOME OFF\n");
    if((stSENPrev & ST_SEN_FABRIC_DETECT1) != (stSENin & ST_SEN_FABRIC_DETECT1)) putStr(&huart4,(const unsigned char *)"SEN FABR1 DET ON\n");
    else							                                             putStr(&huart4,(const unsigned char *)"SEN FABR1 DET OFF\n");
    if((stSENPrev & ST_SEN_FABRIC_DETECT2) != (stSENin & ST_SEN_FABRIC_DETECT2)) putStr(&huart4,(const unsigned char *)"SEN FABR2 DET ON\n");
    else							                                             putStr(&huart4,(const unsigned char *)"SEN FABR2 DET OFF\n");
    if((stSENPrev & ST_SEN_CLAMP12_OPEN) != (stSENin & ST_SEN_CLAMP12_OPEN))  putStr(&huart4,(const unsigned char *)"SEN CLAMP12 OPEN\n\n");
    else							                                          putStr(&huart4,(const unsigned char *)"SEN CLAMP12 OPEN OFF\n");
    if((stSENPrev & ST_SEN_CLAMP12_OPEN) != (stSENin & ST_SEN_CLAMP12_CLOSE)) putStr(&huart4,(const unsigned char *)"SEN CLAMP12 CLOSE\n");
    else							                                          putStr(&huart4,(const unsigned char *)"SEN CLAMP12 CLOSE OFF\n");
    if((stSENPrev & ST_SEN_CLAMP34_OPEN) != (stSENin & ST_SEN_CLAMP34_OPEN)) putStr(&huart4,(const unsigned char *)"SEN CLAMP34 OPEN\n");
    else							                                         putStr(&huart4,(const unsigned char *)"SEN CLAMP34 OPEN OFF\n");
    if((stSENPrev & ST_SEN_CLAMP34_CLOSE) != (stSENin & ST_SEN_CLAMP34_CLOSE)) putStr(&huart4,(const unsigned char *)"SEN CLAMP34 CLOSE\n");
    else							                                           putStr(&huart4,(const unsigned char *)"SEN CLAMP34 CLOSE OFF\n");

	putChar(&huart4,'\n');

    stSWPrev = stSENin;
}
#endif
#endif
		}
		else
		{
			inCh++;
		}

		//rsCnt = READ_SIG_TCNT;
		//rsFg = READ_SIG_TSTART;
	//}
}
#endif

/**
  * @brief output signal function
  * @param None
  * @retval None
  */
void chkError(void)
{
    if(scFg == 1)
    {
        scFg = 0;
        /*--------------------------------------
        |BUF11
        |0x01 : EMERGENCY STOP
        |0x02 : COMMUNICATION ERROR
        |0x04 : HEAT OVER or UNDER CURRENT
        |0x08 : AIR PRESSURE LOW
        |0x10 : UP LIMIT SENSOR INPUT
        |0x02 : DOWN LIMIT SENSOR INPUT
        |0x40 : UP/DOWN HOME ERROR
        |0x80 : NEEDLE HOME ERROR
        --------------------------------------*/
    	if(stSWin & ST_SW_EMS) errorCode[0] |= 0x01;
    	else                   errorCode[0] &= ~0x01;

    	if(comCycleFg == COM_PACKET_TIM_ERROR) errorCode[0] |= 0x02;
    	else                                   errorCode[0] &= ~0x02;

    	// HEAT CURRENT -> adcCheck()
    	
        if((apkPa > AP_KPA_MIN) && (apkPa < AP_KPA_MAX))    errorCode[0] &= ~0x08;
        else                                                errorCode[0] |= 0x08;

        if(stSENin & ST_SEN_LIFTING_UP_LIM) errorCode[0] |= 0x10;
    	else                                errorCode[0] &= ~0x10;

        if(stSENin & ST_SEN_LIFTING_DN_LIM) errorCode[0] |= 0x20;
    	else                                errorCode[0] &= ~0x20;

        // ud home -> initMachine()

        // needle home -> initMachine()

        /*--------------------------------------
        |BUF12
        |0x01 : LOOPER HOME ERROR
        |0x02 : MOVING BACKWARD(LEFT) LIMIT SENSOR INPUT
        |0x04 : MOVING FORWARD(RIGHT) LIMIT SENSOR INPUT
        |0x08 : MOVING HOME ERROR
        |0x10 : BAND EMPTY1 - EMPTY
        |0x02 : BAND EMPTY2 - EMPTY
        |0x40 : BAND LOST1 - LOST
        |0x80 : BAND LOST2 - LOST
        --------------------------------------*/

        // looper home -> initMachine()

        if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)    errorCode[1] |= 0x02;
        else                                        errorCode[1] &= ~0x02;

        if(stSENin & ST_SEN_MOVING_FORWARD_LIM)    errorCode[1] |= 0x04;
        else                                        errorCode[1] &= ~0x04;

        // moving home -> initMachine()

        if(stSENin & ST_SEN_BAND_EMPTY1)    errorCode[1] |= 0x10;
        else                                errorCode[1] &= ~0x10;

        if(stSENin & ST_SEN_BAND_EMPTY2)    errorCode[1] |= 0x20;
        else                                errorCode[1] &= ~0x20;

        if(stSENin & ST_SEN_BAND_LOST1)    errorCode[1] |= 0x40;
        else                               errorCode[1] &= ~0x40;

        if(stSENin & ST_SEN_BAND_LOST2)    errorCode[1] |= 0x80;
        else                               errorCode[1] &= ~0x80;

        /*--------------------------------------
        |BUF13
        |0x01 : UP THREAD CUT - CUT
        |0x02 : DN THREAD CUT - CUT
        |0x04 : CLAMP12 OPEN ERROR
        |0x08 : CLAMP12 CLOSE ERROR
        |0x10 : CLAMP34 OPEN ERROR
        |0x02 : CLAMP12 CLOSE ERROR
        |0x40 : FABRIC1 DETECT
        |0x80 : FABRIC2 DETECT
        --------------------------------------*/

        if(stSENin & ST_SEN_UPPER_THREAD_CUT_OFF)   errorCode[2] |= 0x01;
        else                                        errorCode[2] &= ~0x01;

        if(stSENin & ST_SEN_UNDER_THREAD_CUT_OFF)   errorCode[2] |= 0x02;
        else                                        errorCode[2] &= ~0x02;

        // CLAMP12 OPEN ERROR
        if(stSENin & ST_SEN_CLAMP12_OPEN)   errorCode[2] |= 0x04;
        else                                errorCode[2] &= ~0x04;
        // CLAMP12 CLOSE ERROR
        if(stSENin & ST_SEN_CLAMP12_CLOSE)  errorCode[2] |= 0x08;
        else                                errorCode[2] &= ~0x08;

        // CLAMP34 OPEN ERROR
        if(stSENin & ST_SEN_CLAMP34_OPEN)   errorCode[2] |= 0x10;
        else                                errorCode[2] &= ~0x10;
        // CLAMP34 CLOSE ERROR
        if(stSENin & ST_SEN_CLAMP34_CLOSE)  errorCode[2] |= 0x20;
        else                                errorCode[2] &= ~0x20;

        // FABRIC1 DETECT
        //if(stSENin & ST_SEN_FABRIC_DETECT1) errorCode[2] |= 0x40;
        //else                                errorCode[2] &= ~0x40;
        
        // FABRIC2 DETECT
        //if(stSENin & ST_SEN_FABRIC_DETECT2) errorCode[2] |= 0x80;
        //else                                errorCode[2] &= ~0x80;

        /*--------------------------------------
        |BUF14
        |0x01 : SERVO1(NEEDEL) ALARM
        |0x02 : SERVO2(LOOPER) ALARM
        |0x04 : SERVO3(MOVING) ALARM
        |0x08 : SERVO4(LIFTING) ALARM
        |0x10 : RESERVED
        |0x02 : RESERVED
        |0x40 : RESERVED
        |0x80 : SOFTWARE STOP(from PC)
        --------------------------------------*/
        if(stSVin[0] & ST_SV_ALRM)  errorCode[3] |= 0x01;
        else                        errorCode[3] &= ~0x01;

        if(stSVin[1] & ST_SV_ALRM)  errorCode[3] |= 0x02;
        else                        errorCode[3] &= ~0x02;

        if(stSVin[2] & ST_SV_ALRM)  errorCode[3] |= 0x04;
        else                        errorCode[3] &= ~0x04;

        if(stSVin[3] & ST_SV_ALRM)  errorCode[3] |= 0x08;
        else                        errorCode[3] &= ~0x08;

        if(stSVin[0] & ST_SV_RDY)  svRdy |= 0x01;
        else                       svRdy &= ~0x01;

        if(stSVin[1] & ST_SV_RDY)  svRdy |= 0x02;
        else                       svRdy &= ~0x02;

        if(stSVin[2] & ST_SV_RDY)  svRdy |= 0x04;
        else                       svRdy &= ~0x04;

        if(stSVin[3] & ST_SV_RDY)  svRdy |= 0x08;
        else                       svRdy &= ~0x08;
    }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

