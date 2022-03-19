/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern	ADC_HandleTypeDef hadc1;

extern	I2C_HandleTypeDef hi2c1;

extern	RTC_HandleTypeDef hrtc;

extern	SPI_HandleTypeDef hspi2;
extern	SPI_HandleTypeDef hspi3;

extern	TIM_HandleTypeDef htim2;
extern	TIM_HandleTypeDef htim3;
extern	TIM_HandleTypeDef htim4;
extern	TIM_HandleTypeDef htim8;

extern	UART_HandleTypeDef huart4;
extern	UART_HandleTypeDef huart1;
extern	UART_HandleTypeDef huart2;
extern	UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define __DEBUG__
/* PORTA */
//		UART4_TX					GPIO_PIN_0
//		UART4_RX					GPIO_PIN_1
//		UART2_TX					GPIO_PIN_2
//		UART2_RX					GPIO_PIN_3
//		ADC1_IN4					GPIO_PIN_4
#define ROE_A						GPIO_PIN_5
//#define ROE_A_EXTI_IRQn			EXTI9_5_IRQn
#define RC_PULSE1					GPIO_PIN_6
//		RESERVED					GPIO_PIN_7
#define UART2_RW					GPIO_PIN_8
//		UART1_TX					GPIO_PIN_9
//		UART1_RX					GPIO_PIN_10
#define SYS_RUN						GPIO_PIN_11
#define EEP_WP						GPIO_PIN_12
//		SWDIO						GPIO_PIN_13
//		SWCLK						GPIO_PIN_14
//		RESERVED					GPIO_PIN_15

#define	UART2_READ					HAL_GPIO_WritePin(GPIOA, UART2_RW, GPIO_PIN_RESET)
#define	UART2_WRITE					HAL_GPIO_WritePin(GPIOA, UART2_RW, GPIO_PIN_SET)

#define	LED_RUN_ON					HAL_GPIO_WritePin(GPIOA, SYS_RUN, GPIO_PIN_RESET)
#define	LED_RUN_OFF					HAL_GPIO_WritePin(GPIOA, SYS_RUN, GPIO_PIN_SET)
#define	LED_RUN_TOG					HAL_GPIO_TogglePin(GPIOA, SYS_RUN)

#define	EEP_WRITE					HAL_GPIO_WritePin(GPIOA, EEP_WP, GPIO_PIN_SET)
#define	EEP_PROTECT					HAL_GPIO_WritePin(GPIOA, EEP_WP, GPIO_PIN_RESET)



/* PORTB */
#define OUT_CLK1					GPIO_PIN_0
#define OUT_CLK2					GPIO_PIN_1
#define OUT_CLK3					GPIO_PIN_2
#define ROE_B						GPIO_PIN_3
#define OUT_CLK4					GPIO_PIN_4
#define OUT_CLK5					GPIO_PIN_5
//		I2C_SCL						GPIO_PIN_6
//		I2C_SDA						GPIO_PIN_7
#define OUT_CLK7					GPIO_PIN_8
#define OUT_CLK8					GPIO_PIN_9
//		UART1_TX					GPIO_PIN_10
//		UART1_RX					GPIO_PIN_11
#define OUT_CLK6					GPIO_PIN_12
//		SPI2_SCK					GPIO_PIN_13
#define IN_CLK                      GPIO_PIN_14
//		SPI2_MOSI					GPIO_PIN_15

#define	OUT_CLK1_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define	OUT_CLK1_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)	

#define	OUT_CLK2_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define	OUT_CLK2_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

#define	OUT_CLK3_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define	OUT_CLK3_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)

#define	OUT_CLK4_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define	OUT_CLK4_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

#define	OUT_CLK5_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define	OUT_CLK5_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)

#define	OUT_CLK6_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define	OUT_CLK6_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)

#define	OUT_CLK7_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define	OUT_CLK7_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)

#define	OUT_CLK8_HIGH				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define	OUT_CLK8_LOW				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

#define	IN_CLK_HIGH                 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define	IN_CLK_LOW                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)


/* PORTC */
#define IN_EN1						GPIO_PIN_0
#define IN_EN2						GPIO_PIN_1
//		SPI2_MISO					GPIO_PIN_2
#define IN_EN3						GPIO_PIN_3
#define IN_EN4						GPIO_PIN_4
#define IN_EN5						GPIO_PIN_5
#define RC_PULSE2					GPIO_PIN_6
#define IN_EN7						GPIO_PIN_7
#define IN_EN8						GPIO_PIN_8
#define UART1_RW					GPIO_PIN_9
//		SPI3_SCK					GPIO_PIN_10
//		SPI3_MISO					GPIO_PIN_11
//		SPI2_MOSI					GPIO_PIN_12
#define	IN_EN6						GPIO_PIN_13		// LOW
//		RCC_OSC32_IN				GPIO_PIN_14
//		RCC_OSC32_OUT				GPIO_PIN_15

#define	IN_EN1_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN1, GPIO_PIN_SET)
#define	IN_EN1_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN1, GPIO_PIN_RESET)

#define	IN_EN2_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN2, GPIO_PIN_SET)
#define	IN_EN2_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN2, GPIO_PIN_RESET)

#define	IN_EN3_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN3, GPIO_PIN_SET)
#define	IN_EN3_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN3, GPIO_PIN_RESET)

#define	IN_EN4_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN4, GPIO_PIN_SET)
#define	IN_EN4_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN4, GPIO_PIN_RESET)

#define	IN_EN5_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN5, GPIO_PIN_SET)
#define	IN_EN5_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN5, GPIO_PIN_RESET)

#define	IN_EN6_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN6, GPIO_PIN_SET)
#define	IN_EN6_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN6, GPIO_PIN_RESET)

#define	IN_EN7_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN7, GPIO_PIN_SET)
#define	IN_EN7_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN7, GPIO_PIN_RESET)

#define	IN_EN8_HIGH                 HAL_GPIO_WritePin(GPIOC, IN_EN8, GPIO_PIN_SET)
#define	IN_EN8_LOW					HAL_GPIO_WritePin(GPIOC, IN_EN8, GPIO_PIN_RESET)

#define	UART1_READ					HAL_GPIO_WritePin(GPIOC, UART1_RW, GPIO_PIN_RESET)
#define	UART1_WRITE					HAL_GPIO_WritePin(GPIOC, UART1_RW, GPIO_PIN_SET)



/* PORTD */
#define SV_PULSE1_IN				GPIO_PIN_0
//#define SV_PULSE1D0_EXTI_IRQn 	EXTI0_IRQn
#define SV_PULSE2_IN				GPIO_PIN_1
//#define SV_PULSE2D1_EXTI_IRQn	 EXTI1_IRQn
#define SV_PULSE3_IN				GPIO_PIN_2
//#define SV_PULSE3D2_EXTI_IRQn		EXTI2_IRQn
#define SV_PULSE4_IN				GPIO_PIN_3
//#define SV_PULSE4D3_EXTI_IRQn		EXTI3_IRQn
#define ID_D0						GPIO_PIN_4
#define ID_D1						GPIO_PIN_5
#define ID_D2						GPIO_PIN_6
#define ID_D3						GPIO_PIN_7
#define SV_SIGN1					GPIO_PIN_8
#define SV_SIGN2					GPIO_PIN_9
#define SV_SIGN3					GPIO_PIN_10
#define SV_SIGN4					GPIO_PIN_11
#define SV_PULSE1					GPIO_PIN_12
#define SV_PULSE2					GPIO_PIN_13
#define SV_PULSE3					GPIO_PIN_14
#define SV_PULSE4					GPIO_PIN_15	

#define	SV1_DIR_CW					HAL_GPIO_WritePin(GPIOD,SV_SIGN1,GPIO_PIN_SET)
#define	SV1_DIR_CCW					HAL_GPIO_WritePin(GPIOD,SV_SIGN1,GPIO_PIN_RESET)

#define	SV2_DIR_CW					HAL_GPIO_WritePin(GPIOD,SV_SIGN2,GPIO_PIN_SET)
#define	SV2_DIR_CCW					HAL_GPIO_WritePin(GPIOD,SV_SIGN2,GPIO_PIN_RESET)

#define	SV3_DIR_CW					HAL_GPIO_WritePin(GPIOD,SV_SIGN3,GPIO_PIN_SET)
#define	SV3_DIR_CCW					HAL_GPIO_WritePin(GPIOD,SV_SIGN3,GPIO_PIN_RESET)	

#define	SV4_DIR_CW					HAL_GPIO_WritePin(GPIOD,SV_SIGN4,GPIO_PIN_SET)
#define	SV4_DIR_CCW					HAL_GPIO_WritePin(GPIOD,SV_SIGN4,GPIO_PIN_RESET)



/* PORTE */
#define IN0							GPIO_PIN_0
#define IN1 						GPIO_PIN_1
#define IN2 						GPIO_PIN_2
#define IN3 						GPIO_PIN_3
#define IN4 						GPIO_PIN_4
#define IN5 						GPIO_PIN_5
#define IN6 						GPIO_PIN_6
#define IN7 						GPIO_PIN_7
#define OUT0 						GPIO_PIN_8
#define OUT1 						GPIO_PIN_9
#define OUT2 						GPIO_PIN_10
#define OUT3 						GPIO_PIN_11
#define OUT4 						GPIO_PIN_12
#define OUT5 						GPIO_PIN_13
#define OUT6 						GPIO_PIN_14
#define OUT7 						GPIO_PIN_15

/* OUT1,2,3,4 */
#define	SV_ON						OUT0
#define	ALRM_RST					OUT1
#define	EMG_STOP					OUT2
#define	CNT_CLR						OUT3
#define	MODE_SEL0					OUT4
#define	MODE_SEL1					OUT5
#define	CW_LIM						OUT6
#define	CCW_LIM						OUT7

/* OUT5 */
#define	OUTPUT1						OUT0
#define	OUTPUT2						OUT1
#define	OUTPUT3						OUT2
#define	OUTPUT4						OUT3
#define	OUTPUT5						OUT4
#define	OUTPUT6						OUT5
#define	OUTPUT7						OUT6
#define	OUTPUT8						OUT7

/* OUT6 */
#define	OUTPUT9						OUT0
#define	OUTPUT10					OUT1
#define	OUTPUT11					OUT2
#define	OUTPUT12					OUT3
#define	OUTPUT13					OUT4
#define	OUTPUT14					OUT5
#define	OUTPUT15					OUT6
#define	OUTPUT16					OUT7

/* OUT7 */
#define	OUTPUT17					OUT0
#define	OUTPUT18					OUT1
#define	OUTPUT19					OUT2
#define	OUTPUT20					OUT3
#define	OUTPUT21					OUT4
#define	OUTPUT22					OUT5
#define	OUTPUT23					OUT6
#define	OUTPUT24					OUT7

/* OUT8 */
#define	OUTPUT25					OUT0
#define	OUTPUT26					OUT1
#define	R_OUTPUT1					OUT2
#define	R_OUTPUT2					OUT3
#define	R_OUTPUT3					OUT4
#define	R_OUTPUT4					OUT5
#define	R_OUTPUT5					OUT6
#define	R_OUTPUT6					OUT7

/* OUTPUT REDEFINE */
#define	CLAMP1_CNT					OUTPUT1		// 1 : CLOSE, 0 : OPEN
#define	CLAMP2_CNT					OUTPUT2		// 1 : CLOSE, 0 : OPEN
#define	CLAMP3_CNT					OUTPUT3		// 1 : CLOSE, 0 : OPEN
#define	CLAMP4_CNT					OUTPUT4		// 1 : CLOSE, 0 : OPEN
#define	VACCUM_CNT					OUTPUT5		// 1 : OPEN, 0 : CLOSE
#define	RES_OUTPUT6					OUTPUT6
#define	RES_OUTPUT7					OUTPUT7
#define	BAND_CUT_HEAT				OUTPUT8		// 1 : HEAT ON, 0 : HEAT OFF
#define	RES_OUTPUT9					OUTPUT9
#define	RES_OUTPUT10				OUTPUT10
#define	TL_R						OUTPUT11	// 1 : TOWER LAMP RED ON, 0 : TOWER LAMP RED OFF
#define	TL_G						OUTPUT12	// 1 : TOWER LAMP GREEN ON, 0 : TOWER LAMP GREEN OFF
#define	TL_Y						OUTPUT13	// 1 : TOWER LAMP YELLOW ON, 0 : TOWER LAMP YELLOW OFF
#define	RES_OUTPUT14				OUTPUT14
#define	RES_OUTPUT15				OUTPUT15
#define	RES_OUTPUT16				OUTPUT16
#define	RES_OUTPUT17				OUTPUT17
#define	RES_OUTPUT18				OUTPUT18
#define	RES_OUTPUT19				OUTPUT19
#define	RES_OUTPUT20				OUTPUT20
#define	RES_OUTPUT21				OUTPUT21
#define	RES_OUTPUT22				OUTPUT22
#define	RES_OUTPUT23				OUTPUT23
#define	RES_OUTPUT24				OUTPUT24
#define	RES_OUTPUT25				OUTPUT25
#define	RES_OUTPUT26				OUTPUT26

/* IN1,2,3,4 */
#define	SV_ALRM						IN0
#define	SV_HOME						IN1
#define	SV_RDY						IN2
#define	ZERO_SPD					IN3
#define	T_POS						IN4	

/* IN1 */
#define	SW_IN1						IN5	// INPUT33
#define	SW_IN2						IN6	// INPUT34
#define	SW_IN3						IN7	// INPUT35	

/* IN2 */
#define	SW_IN4						IN5	// INPUT36
#define	SW_IN5						IN6	// INPUT37
#define	SW_IN6						IN7	// INPUT38

/* IN3 */
#define	SW_IN7						IN5	// INPUT39
#define	SW_IN8						IN6	// INPUT40
#define	SW_IN9						IN7	// INPUT41

/* IN4 */
#define	SW_IN10						IN5	// INPUT42
#define	SW_IN11						IN6	// INPUT43
#define	SW_IN12						IN7	// INPUT44

/* IN5 */
#define	INPUT1						IN0
#define	INPUT2						IN1
#define	INPUT3						IN2
#define	INPUT4						IN3
#define	INPUT5						IN4
#define	INPUT6						IN5
#define	INPUT7						IN6
#define	INPUT8						IN7

/* IN6 */
#define	INPUT9						IN0
#define	INPUT10						IN1
#define	INPUT11						IN2
#define	INPUT12						IN3
#define	INPUT13						IN4
#define	INPUT14						IN5
#define	INPUT15						IN6
#define	INPUT16						IN7

/* IN7 */
#define	INPUT17						IN0
#define	INPUT18						IN1
#define	INPUT19						IN2
#define	INPUT20						IN3
#define	INPUT21						IN4
#define	INPUT22						IN5
#define	INPUT23						IN6
#define	INPUT24						IN7

/* IN8 */
#define	INPUT25						IN0
#define	INPUT26						IN1
#define	INPUT27						IN2
#define	INPUT28						IN3
#define	INPUT29						IN4
#define	INPUT30						IN5
#define	INPUT31						IN6
#define	INPUT32						IN7

/* INPUT REDEFINE */
#if 0
#define	SW_EMS						INPUT1	// PUSH LOCK SWITCH1  1 : EMS SW ON(EMS OCCUR), 0 : EMS SW OFF(NORMAL)
#define	SW_AUTO_MANUAL				INPUT2	// 2SELECT SWITCH1  1 : MANUAL SELECT(MANUAL MODE), 0 : AUTO SELECT(AUTO MODE)
#define	SEN_LIFTING_UP_LIM			INPUT3	// SENSOR  1 : LIFTING UP LIMITE SENSOR ON, 0 : LIFTING UP LIMITE SENSRO OFF
#define	SEN_LIFTING_DN_LIM			INPUT4	// SENSOR  1 : LIFTING DOWN LIMITE SENSOR ON, 0 : LIFTING DOWN LIMITE SENSOR OFF
#define	SEN_LIFTING_HOME			INPUT5	// SENSOR  1 : LIFTING HOME SENSOR ON, 0 : LIFTING HOME SENSOR OFF
#define	SEN_NEEDLE_HOME				INPUT6	// SENSOR  1 : NIDDLE HOME SENSOR ON, 0 : NEDDLE HOME SENSOR OFF
#define	SEN_BAND_EMPTY1				INPUT7	// SENSOR  1 : BAND1 EMPTY SENSOR ON(������), 0 : BAND1 EMPTY SENSOR OFF(��峲��)
#define	SEN_BAND_EMPTY2				INPUT8	// SENSOR  1 : BAND2 EMPTY SENSOR ON(������), 0 : BAND2 EMPTY SENSOR OFF(��峲��)
#define	SEN_BAND_LOST1				INPUT9	// SENSOR  1 : BAND1 LOST SENSOR ON(��� cut off), 0 : BAND1 EMPTY SENSOR OFF(���OK)
#define	SEN_BAND_LOST2				INPUT10	// SENSOR  1 : BAND2 LOST SENSOR ON(��� cut off), 0 : BAND2 EMPTY SENSOR OFF(���OK)
#define	SEN_UPPER_THREAD_CUT_OFF	INPUT11	// SENSOR  1 : UPPER SEWING THREAD CUT OFF DETECT, 0 : UPPER SEWING THREAD NORMAL
#define	SEN_LOOPER_HOME				INPUT12	// SENSOR  1 : LOOPER HOME SENSOR ON, 0 : LOOPER HOME SENSOR OFF
#define	SEN_UNDER_THREAD_CUT_OFF	INPUT13	// SENSOR  1 : UNDER SEWING THREAD CUT OFF DETECT, 0 : UNDER SEWING THREAD NORMAL
#define	SEN_BAND_INSERT1			INPUT14	// SENSOR  1 : BAND INSERT OK, 0 : NOT INSERT BAND for CLAMP
#define	SEN_BAND_INSERT2			INPUT15	// SENSOR  1 : BAND INSERT OK, 0 : NOT INSERT BAND for CLAMP
#define	SEN_MOVING_FORWARD_LIM		INPUT16	// SENSOR  1 : MOVING FORWARD LIMITE SENSOR ON, 0 : MOVING FORWARD LIMITE SENSOR OFF
#define	SEN_MOVING_BACKWARD_LIM		INPUT17	// SENSOR  1 : MOVING REVERSE LIMITE SENSOR ON, 0 : MOVING REVERSE LIMITE SENSOR OFF
#define	SEN_MOVING_HOME				INPUT18	// SENSOR  1 : MOVING HOME SENSOR ON, 0 : MOVING HOME SENSOR OFF
#define	SEN_FABRIC_DETECT1			INPUT19	// SENSOR  1 : FABRIC NOT DETECT, 0 : FABRIC DETECT
#define	SEN_FABRIC_DETECT2			INPUT20	// SENSOR  1 : FABRIC NOT DETECT, 0 : FABRIC DETECT
#define	SEN_CLAMP12_OPEN			INPUT21	// SENSOR  CLAMP12_OPEN-1,CLAMP12_CLOSE-0 : OPEN
#define	SEN_CLAMP12_CLOSE			INPUT22	// SENSOR  CLAMP12_OPEN-0,CLAMP12_CLOSE-1 : CLOSE
#define	SEN_CLAMP34_OPEN			INPUT23	// SENSOR  CLAMP34_OPEN-1,CLAMP34_CLOSE-0 : OPEN
#define	SEN_CLAMP34_CLOSE			INPUT24	// SENSOR  CLAMP34_OPEN-0,CLAMP34_CLOSE-1 : CLOSE
#define	SW_MOVING_FB				INPUT25	// 3SELECT SWITCH1  1 : MOVING FORWARD/BACKWARD
#define	SW_MOVING_UD				INPUT26	// 3SELECT SWITCH1  1 : MOVING UP/DOWN
#define	SW_AUTO_START				INPUT27	// PUSH BUTTON1  1 : AUTO MODE START
#define	SW_AUTO_STOP				INPUT28	// PUSH BUTTON2  1 : AUTO MODE STOP
#define	SW_AUTO_RESET				INPUT29	// PUSH BUTTON3  1 : AUTO MODE RESET - all position �ʱ�ȭ
#define	SW_TEST_SEWING				INPUT30	// PUSH BUTTON4  1 : TEST SEWING
#define	SW_NEEDLE_HOME				INPUT31	// PUSH BUTTON5  1 : UPPER SEWING MACHINE NEEDLE MOVING for HOME POSITION
#define	SW_LOOPER_HOME				INPUT32	// PUSH BUTTON6  1 : UNDER SEWING MACHINE LOOPER MOVING for HOME POSITION
#define	SW_MOVING_HOME				SW_IN1	// PUSH BUTTON7  1 : MOVING FORWARD-REVERSE for HOME POSITION
#define	SW_UD_HOME					SW_IN2	// PUSH BUTTON8  1 : MOVING UP-DOWN for HOME POSITION
#define	SW_CLAMP1_OPEN_CLOSE		SW_IN3	// 2SELECT SWITCH2  1 : CLAMP1 CLOSE, 0 : CLAMP1 OPEN
#define	SW_CLAMP2_OPEN_CLOSE		SW_IN4	// 2SELECT SWITCH3  1 : CLAMP2 CLOSE, 0 : CLAMP2 OPEN
#define	SW_CLAMP3_OPEN_CLOSE		SW_IN5	// 2SELECT SWITCH4  1 : CLAMP3 CLOSE, 0 : CLAMP2 OPEN
#define	SW_CLAMP4_OPEN_CLOSE		SW_IN6	// 2SELECT SWITCH5  1 : CLAMP4 CLOSE, 0 : CLAMP2 OPEN
#define	SW_VACCUM_OPEN_CLOSE		SW_IN7	// 2SELECT SWITCH6  1 : VACCUM OPEN, 0 : VACCUM CLOSE
#define	SW_BAND_CUT					SW_IN8	// PUSH BUTTON9  1 : BAND CUT(with HEAT THREAD), 0 : NO ACTION
#define	RES_INPUT1					SW_IN9
#define	RES_INPUT2					SW_IN10
#define	RES_INPUT3					SW_IN11
#define	RES_INPUT4					SW_IN12
#endif

#define	SEN_LIFTING_DN_LIM			INPUT1	// SENSOR  1 : LIFTING DOWN LIMITE SENSOR ON, 0 : LIFTING DOWN LIMITE SENSOR OFF
#define	SEN_LIFTING_HOME			INPUT2	// SENSOR  1 : LIFTING HOME SENSOR ON, 0 : LIFTING HOME SENSOR OFF
#define	SEN_BAND_EMPTY1				INPUT3	// SENSOR  1 : BAND1 EMPTY SENSOR ON(������), 0 : BAND1 EMPTY SENSOR OFF(��峲��)
//#define	SEN_BAND_EMPTY2				INPUT4	// SENSOR  1 : BAND2 EMPTY SENSOR ON(������), 0 : BAND2 EMPTY SENSOR OFF(��峲��)
#define	SEN_MOVING_HOME				INPUT4	// SENSOR  1 : MOVING HOME SENSOR ON, 0 : MOVING HOME SENSOR OFF
#define	SEN_BAND_LOST1				INPUT5	// SENSOR  1 : BAND1 LOST SENSOR ON(��� cut off), 0 : BAND1 EMPTY SENSOR OFF(���OK)
#define	SEN_BAND_LOST2				INPUT6	// SENSOR  1 : BAND2 LOST SENSOR ON(��� cut off), 0 : BAND2 EMPTY SENSOR OFF(���OK)
#define	SEN_UPPER_THREAD_CUT_OFF	INPUT7	// SENSOR  1 : UPPER SEWING THREAD CUT OFF DETECT, 0 : UPPER SEWING THREAD NORMAL
#define	SEN_LOOPER_HOME				INPUT9	// SENSOR  1 : LOOPER HOME SENSOR ON, 0 : LOOPER HOME SENSOR OFF

#define	SEN_NEEDLE_HOME				INPUT17	// SENSOR  1 : NIDDLE HOME SENSOR ON, 0 : NEDDLE HOME SENSOR OFF
#define	SEN_LIFTING_UP_LIM			INPUT18	// SENSOR  1 : LIFTING UP LIMITE SENSOR ON, 0 : LIFTING UP LIMITE SENSRO OFF

#define	SW_AUTO_MANUAL				INPUT20	// 2SELECT SWITCH1  1 : MANUAL SELECT(MANUAL MODE), 0 : AUTO SELECT(AUTO MODE)
#define	SW_AUTO_START				INPUT21	// PUSH BUTTON1  1 : AUTO MODE START
#define	SW_AUTO_STOP				INPUT22	// PUSH BUTTON2  1 : AUTO MODE STOP
#define	SW_AUTO_RESET				INPUT23	// PUSH BUTTON3  1 : AUTO MODE RESET - all position �ʱ�ȭ
#define	SW_TEST_SEWING				INPUT24	// PUSH BUTTON4  1 : TEST SEWING
#define	SW_NEEDLE_HOME				INPUT25	// PUSH BUTTON5  1 : UPPER SEWING MACHINE NEEDLE MOVING for HOME POSITION
#define	SW_LOOPER_HOME				INPUT26	// PUSH BUTTON6  1 : UNDER SEWING MACHINE LOOPER MOVING for HOME POSITION
#define	SW_MOVING_HOME				INPUT27	// PUSH BUTTON7  1 : MOVING FORWARD-REVERSE for HOME POSITION
#define	SW_UD_HOME					INPUT28	// PUSH BUTTON8  1 : MOVING UP-DOWN for HOME POSITION
#define	SW_CLAMP1_OPEN_CLOSE		INPUT29	// 2SELECT SWITCH2  1 : CLAMP1 CLOSE, 0 : CLAMP1 OPEN
#define	SW_CLAMP2_OPEN_CLOSE		INPUT30	// 2SELECT SWITCH3  1 : CLAMP2 CLOSE, 0 : CLAMP2 OPEN
#define	SW_CLAMP3_OPEN_CLOSE		INPUT31	// 2SELECT SWITCH4  1 : CLAMP3 CLOSE, 0 : CLAMP2 OPEN
#define	SW_CLAMP4_OPEN_CLOSE		INPUT32	// 2SELECT SWITCH5  1 : CLAMP4 CLOSE, 0 : CLAMP2 OPEN
#define	SW_VACCUM_OPEN_CLOSE		SW_IN1	// 2SELECT SWITCH6  1 : VACCUM OPEN, 0 : VACCUM CLOSE

#define SW_ROE_X                    SW_IN8 // ROE SELECT SWITCH1 1 : FB, 0 : OFF or another
#define SW_ROE_Y                    SW_IN9 // ROE SELECT SWITCH1 1 : UD, 0 : OFF or another
#define SW_ROE_Z                    SW_IN10 // ROE SELECT SWITCH1 1 : NEEDLE? and LOOPER?, 0 : OFF or another
#define SW_ROE_X1                   SW_IN11 // ROE SELECT SWITCH2 1 : ROE 1pulse/len = 1mm, 0 : OFF or another
#define SW_ROE_X10                  SW_IN12 // ROE SELECT SWITCH2 1 : ROE 1pulse/len = 10mm, 0 : OFF or another



// -------------------------------------------------------------------------------------------------------------
// INPUT CH 1
#define new_SW_VACCUM_OPEN_CLOSE        IN_PORT_DATA[IN_CH1].bit5   // vaccum select(open/close) switch(manual mode)

// INPUT CH 2
//
// INPUT CH 3
#define new_SW_ROE_X                    IN_PORT_DATA[IN_CH3].bit6   // [SWITCH] 1: FB, 0: OFF or another
#define new_SW_ROE_Y                    IN_PORT_DATA[IN_CH3].bit7   // [SWITCH] 1: UD, 0: OFF or another

// INPUT CH 4
#define new_SW_ROE_Z                    IN_PORT_DATA[IN_CH4].bit5   // [SWITCH] 1: NEEDLE? and LOOPER?,     0 : OFF or another
#define new_SW_ROE_X1                   IN_PORT_DATA[IN_CH4].bit6   // [SWITCH] 1: ROE 1pulse/len = 1mm,    0 : OFF or another
#define new_SW_ROE_X10                  IN_PORT_DATA[IN_CH4].bit7   // [SWITCH] 1: ROE 1pulse/len = 10mm,   0 : OFF or another

// INPUT CH 5
#define new_SEN_LIFTING_DN_LIM          IN_PORT_DATA[IN_CH5].bit0   // [SENSOR] LIFTING DOWN LIMITE         1: ON, 0: OFF
#define new_SEN_LIFTING_HOME            IN_PORT_DATA[IN_CH5].bit1   // [SENSOR] LIFTING HOME                1: ON, 0: OFF
#define new_SEN_BAND_EMPTY1             IN_PORT_DATA[IN_CH5].bit2   // [SENSOR] BAND1 EMPTY                 1: ON, 0: OFF
#define new_SEN_MOVING_HOME             IN_PORT_DATA[IN_CH5].bit3   // [SENSOR] MOVING HOME                 1: ON, 0: OFF
#define new_SEN_BAND_LOST1              IN_PORT_DATA[IN_CH5].bit4   // [SENSOR] BAND1 LOST                  1: ON, 0: OFF
#define new_SEN_BAND_LOST2              IN_PORT_DATA[IN_CH5].bit5   // [SENSOR] BAND2 LOST                  1: ON, 0: OFF

// INPUT CH 6
#define new_SEN_UPPER_THREAD_CUT_OFF    IN_PORT_DATA[IN_CH6].bit6   // [SENSOR] UPPER SEWING THREAD         1: CUT OFF DETECT, 0: NORMAL
#define new_SEN_LOOPER_HOME             IN_PORT_DATA[IN_CH6].bit0   // [SENSOR] LOOPER HOME                 1: ON, 0: OFF

// INPUT CH 7
#define new_SW_AUTO_MANUAL              IN_PORT_DATA[IN_CH7].bit3   // [SWITCH] Mode select  1: MANUAL MODE, 0: AUTO MODE
#define new_SW_AUTO_START               IN_PORT_DATA[IN_CH7].bit4   // [PUSH BT] 1: AUTO MODE START
#define new_SW_AUTO_STOP                IN_PORT_DATA[IN_CH7].bit5   // [PUSH BT] 1: AUTO MODE STOP
#define new_SW_AUTO_RESET               IN_PORT_DATA[IN_CH7].bit6   // [PUSH BT] 1: AUTO MODE RESET
#define new_SW_TEST_SEWING              IN_PORT_DATA[IN_CH7].bit7   // [PUSH BT] 1: TEST SEWING
#define new_SEN_LIFTING_UP_LIM          IN_PORT_DATA[IN_CH7].bit1   // [SENSOR] LIFTING UP LIMITE           1: ON, 0: OFF
#define new_SEN_NEEDLE_HOME             IN_PORT_DATA[IN_CH7].bit0   // [SENSOR] NEEDLE HOME                 1: ON, 0: OFF

// INPUT CH 8


// -------------------------------------------------------------------------------------------------------------

#if 0
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOE
#define IN3_Pin GPIO_PIN_3
#define IN3_GPIO_Port GPIOE
#define IN4_Pin GPIO_PIN_4
#define IN4_GPIO_Port GPIOE
#define IN5_Pin GPIO_PIN_5
#define IN5_GPIO_Port GPIOE
#define IN6_Pin GPIO_PIN_6
#define IN6_GPIO_Port GPIOE
#define IN_CLK1_Pin GPIO_PIN_0
#define IN_CLK1_GPIO_Port GPIOC
#define IN_CLK2_Pin GPIO_PIN_1
#define IN_CLK2_GPIO_Port GPIOC
#define IN_CLK3_Pin GPIO_PIN_3
#define IN_CLK3_GPIO_Port GPIOC
#define ROE_A_Pin GPIO_PIN_5
#define ROE_A_GPIO_Port GPIOA
#define ROE_A_EXTI_IRQn EXTI9_5_IRQn
#define RC_PULSE1_Pin GPIO_PIN_6
#define RC_PULSE1_GPIO_Port GPIOA
#define IN_CLK4_Pin GPIO_PIN_4
#define IN_CLK4_GPIO_Port GPIOC
#define IN_CLK5_Pin GPIO_PIN_5
#define IN_CLK5_GPIO_Port GPIOC
#define OUT_CLK1_Pin GPIO_PIN_0
#define OUT_CLK1_GPIO_Port GPIOB
#define OUT_CLK2_Pin GPIO_PIN_1
#define OUT_CLK2_GPIO_Port GPIOB
#define OUT_CLK3_Pin GPIO_PIN_2
#define OUT_CLK3_GPIO_Port GPIOB
#define IN7_Pin GPIO_PIN_7
#define IN7_GPIO_Port GPIOE
#define OUT0_Pin GPIO_PIN_8
#define OUT0_GPIO_Port GPIOE
#define OUT1_Pin GPIO_PIN_9
#define OUT1_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_10
#define OUT2_GPIO_Port GPIOE
#define OUT3_Pin GPIO_PIN_11
#define OUT3_GPIO_Port GPIOE
#define OUT4_Pin GPIO_PIN_12
#define OUT4_GPIO_Port GPIOE
#define OUT5_Pin GPIO_PIN_13
#define OUT5_GPIO_Port GPIOE
#define OUT6_Pin GPIO_PIN_14
#define OUT6_GPIO_Port GPIOE
#define OUT7_Pin GPIO_PIN_15
#define OUT7_GPIO_Port GPIOE
#define OUT_CLK6_Pin GPIO_PIN_12
#define OUT_CLK6_GPIO_Port GPIOB
#define SV_SIGN1_Pin GPIO_PIN_8
#define SV_SIGN1_GPIO_Port GPIOD
#define SV_SIGN2_Pin GPIO_PIN_9
#define SV_SIGN2_GPIO_Port GPIOD
#define SV_SIGN3_Pin GPIO_PIN_10
#define SV_SIGN3_GPIO_Port GPIOD
#define SV_SIGN4_Pin GPIO_PIN_11
#define SV_SIGN4_GPIO_Port GPIOD
#define SV_PULSE1_Pin GPIO_PIN_12
#define SV_PULSE1_GPIO_Port GPIOD
#define SV_PULSE2_Pin GPIO_PIN_13
#define SV_PULSE2_GPIO_Port GPIOD
#define SV_PULSE3_Pin GPIO_PIN_14
#define SV_PULSE3_GPIO_Port GPIOD
#define SV_PULSE4_Pin GPIO_PIN_15
#define SV_PULSE4_GPIO_Port GPIOD
#define RC_PULSE2_Pin GPIO_PIN_6
#define RC_PULSE2_GPIO_Port GPIOC
#define IN_CLK7_Pin GPIO_PIN_7
#define IN_CLK7_GPIO_Port GPIOC
#define IN_CLK8_Pin GPIO_PIN_8
#define IN_CLK8_GPIO_Port GPIOC
#define UART1_RW_Pin GPIO_PIN_9
#define UART1_RW_GPIO_Port GPIOC
#define UART2_RW_Pin GPIO_PIN_8
#define UART2_RW_GPIO_Port GPIOA
#define SYS_RUN_Pin GPIO_PIN_11
#define SYS_RUN_GPIO_Port GPIOA
#define EEP_WP_Pin GPIO_PIN_12
#define EEP_WP_GPIO_Port GPIOA
#define IN_CLK6_Pin GPIO_PIN_15
#define IN_CLK6_GPIO_Port GPIOA
#define SV_PULSE1D0_Pin GPIO_PIN_0
#define SV_PULSE1D0_GPIO_Port GPIOD
#define SV_PULSE1D0_EXTI_IRQn EXTI0_IRQn
#define SV_PULSE2D1_Pin GPIO_PIN_1
#define SV_PULSE2D1_GPIO_Port GPIOD
#define SV_PULSE2D1_EXTI_IRQn EXTI1_IRQn
#define SV_PULSE3D2_Pin GPIO_PIN_2
#define SV_PULSE3D2_GPIO_Port GPIOD
#define SV_PULSE3D2_EXTI_IRQn EXTI2_IRQn
#define SV_PULSE4D3_Pin GPIO_PIN_3
#define SV_PULSE4D3_GPIO_Port GPIOD
#define SV_PULSE4D3_EXTI_IRQn EXTI3_IRQn
#define ID_D0_Pin GPIO_PIN_4
#define ID_D0_GPIO_Port GPIOD
#define ID_D1_Pin GPIO_PIN_5
#define ID_D1_GPIO_Port GPIOD
#define ID_D2_Pin GPIO_PIN_6
#define ID_D2_GPIO_Port GPIOD
#define ID_D3_Pin GPIO_PIN_7
#define ID_D3_GPIO_Port GPIOD
#define ROE_B_Pin GPIO_PIN_3
#define ROE_B_GPIO_Port GPIOB
#define OUT_CLK4_Pin GPIO_PIN_4
#define OUT_CLK4_GPIO_Port GPIOB
#define OUT_CLK5_Pin GPIO_PIN_5
#define OUT_CLK5_GPIO_Port GPIOB
#define OUT_CLK7_Pin GPIO_PIN_8
#define OUT_CLK7_GPIO_Port GPIOB
#define OUT_CLK8_Pin GPIO_PIN_9
#define OUT_CLK8_GPIO_Port GPIOB
#define IN0_Pin GPIO_PIN_0
#define IN0_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOE
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
