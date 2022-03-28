/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "vfdn_adc.h"
#include "vfdn_spi.h"
#include "vfdn_sub.h"
#include "vfdn_com.h"
#include "vfdn_servo.h"
#include "dwt_stm32_delay.h"

#include <stdio.h>
#include <stdbool.h>

void _write(int file, uint8_t* p, int len)
{
	HAL_UART_Transmit(&huart4, p, len, 500);
}

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_TIM8_Init();
    
	/* USER CODE BEGIN 2 */
    DWT_Delay_Init();
    HAL_TIM_Base_Start_IT(&htim2);

    //putChar(&huart4,'%');
    DWT_Delay_us(3000000);

    initVariables();
	readID();

    putStr(&huart4,(const unsigned char *)"NEW_System Start!\n");
	printf("TEST!!!!!!!!!!!!!!!!\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
	
		//chkError();
		//scanSw();
		readSignalProcess();

		if (SW_AUTO_MANUAL)
			manualMode1();
		else
			autoMode1();
		if(IN_PORT_DATA[0].bit0 == 1)//
			emergnecyMode();


				
		
		//adcCheck();
		//apCheck();
		//comCheck();
		//comAnalysis();
		//cntlTLamp();

#if 0
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET); // input clock high
        
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET); // out en1
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,SET); // out en2
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,SET); // out en3
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,SET); // out en4
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET); // out en5
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,SET); // out en6
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,SET); // out en7
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET); // out en8

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET); // out clock
                
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,SET); // in clock1
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,SET); // in clock2
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,SET); // in clock3
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,SET); // in clock4
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,SET); // in clock5
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,SET); // in clock6
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,SET); // in clock7
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET); // in clock8
        
        DWT_Delay_us(100);
        
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET); // input clock high

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET); // out en1
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,RESET); // out en2
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,RESET); // out en3
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,RESET); // out en4
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,RESET); // out en5
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,RESET); // out en6
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,RESET); // out en7
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET); // out en8

        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET); // out clock

        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,RESET); // in en1
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,RESET); // in en2
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,RESET); // in en3
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,RESET); // in en4
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,RESET); // in en5
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,RESET); // in en6
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,RESET); // in en7
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,RESET); // in en8

        DWT_Delay_us(100);
#endif
        /* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void initVariables(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,RESET); // input clock low
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,SET); // input en1 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,SET); // input en2 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,SET); // input en3 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,SET); // input en4 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,SET); // input en5 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,SET); // input en6 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,SET); // input en7 high
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,SET); // input en8 high




	OUT_PORT_DATA[OUT_CH0].data=0b00000000;
	OUT_PORT_DATA[OUT_CH1].data=0b00000000;
	OUT_PORT_DATA[OUT_CH2].data=0b00000000;
	OUT_PORT_DATA[OUT_CH3].data=0b00000000;
	OUT_PORT_DATA[OUT_CH4].data=0b11111111;
	OUT_PORT_DATA[OUT_CH5].data=0b11111111;
	OUT_PORT_DATA[OUT_CH6].data=0b11111111;
	OUT_PORT_DATA[OUT_CH7].data=0b11111111;
	outportSignal(needle_alram_reset,SET);
	outportSignal(looper_alram_reset,SET);
	outportSignal(moving_alram_reset,SET);
	outportSignal(updown_alram_reset,SET);
	outportSignal(needle_servo_on,RESET);
	outportSignal(looper_servo_on,RESET);
	outportSignal(moving_servo_on,RESET);
	outportSignal(updown_servo_on,RESET);
	
	

	g_auto_sewing_length = 50;//1500mm
	g_test_sewing_length = 50;//1000mm
	g_init_speed =FRQ_200Hz;
	g_auto_sewing_speed = FRQ_15KHz;
	g_test_sewing_speed = FRQ_2KHz;
	g_lifting_speed = FRQ_2KHz;
	g_moving_speed = FRQ_5KHz;
	g_move_target_count = (((float)g_auto_sewing_length)/ONE_PULSE_MV);//g_auto_sewing_length/ONE_PULSE_MV;//MOV_1500MM_CLOCK;//auto sewing count    1500mm CLOCK    move length(mm)/ONE_PULSE_MOV==>1500/ONE_PULSE_MV

	g_manual_status=MANUAL_READY;
	g_auto_status=AUTO_READY;
	g_autosewing_status=AUTO_INIT;
	g_auto_wait = false;
	g_rotaryencorder_status=OFF;
	g_rc_delay_time=5;//5ms
}

void manualMode1(void)
{
	if(g_manual_status==MANUAL_READY)
	{
		if(SW_NEEDLE_HOME == 0) //init needle pushbutton ON   
			g_manual_status=INIT_NEEDLE_PUSH;
		else if(SW_LOOPER_HOME == 0)
			g_manual_status=INIT_LOOFER_PUSH;
		else if(SW_MOVING_HOME == 0)
			g_manual_status=INIT_MOVING_PUSH;
		else if(SW_LIFTING_HOME == 0)
			g_manual_status=INIT_LIFTING_PUSH;
		else if(SW_TEST_SEWING == 0)
			g_manual_status=INIT_TEST_PUSH;
		else if( (SW_ROE_X == 0) || (SW_ROE_Y == 0) || (SW_ROE_Z == 0) )
			g_manual_status=RUN_ROTARYENCORDER_PUSH;
		else 
		{
			g_needle_servor_status=OFF;
			g_looper_servor_status=OFF;
			g_moving_servor_status=OFF;
			//g_sewing_servor_status=OFF;
			g_rotaryencorder_status=OFF;
		}

		if(SW_CLAMP1_OPEN_CLOSE==0)					//band clamp 
			exeClamp1(band_clamp_1_on,RESET);
		else
			exeClamp1(band_clamp_1_on,SET);
		if(SW_CLAMP2_OPEN_CLOSE==0)					
			exeClamp1(band_clamp_2_on,RESET);
		else
			exeClamp1(band_clamp_2_on,SET);
		if(SW_CLAMP3_OPEN_CLOSE==0)					//fablic clamp 
		 	exeClamp1(fablic_clamp_1_on,RESET);
		else
			exeClamp1(fablic_clamp_1_on,SET);
		if(SW_CLAMP4_OPEN_CLOSE==0)					
		 	exeClamp1(fablic_clamp_2_on,RESET);
		else
			exeClamp1(fablic_clamp_2_on,SET);
		if(SW_VACCUM_OPEN_CLOSE==0)					//???vaccum
		 	exeVaccum1(vaccum_on,RESET);
		else
			exeVaccum1(vaccum_on,SET);
	}

	
	if(SW_MANUAL_STOP==0)//manual stop
	{
		waitManual();
		g_manual_status=MANUAL_READY;
	}

	switch(g_manual_status)
	{
		case INIT_NEEDLE_PUSH:
			initNeedle1();
			break;
		case INIT_LOOFER_PUSH:
			initLooper1();
			break;
		case INIT_LIFTING_PUSH:
			initLifting1();
			break;
		case INIT_MOVING_PUSH:
			initMoving1();
			break;
		case INIT_TEST_PUSH:
			exeTestSewing(g_test_sewing_length,g_test_sewing_speed);
			break;
		case RUN_ROTARYENCORDER_PUSH:
			exeRotaryEncorder();
			break;
		default : 
			break;
	}

}

void autoMode1(void)
{
	if(g_auto_status==AUTO_READY)
	{
		g_sewing_servor_status=OFF;//init setting
		if(SW_AUTO_START == 0)
		{
			if(g_auto_wait==false)
			{
				g_autosewing_status=AUTO_INIT;
			}
			g_auto_status=AUTO_SEWING_PUSH;
			g_auto_wait=false;
		}
	}
	if(SW_AUTO_STOP == 0)
	{
		waitAutoSewing();
		g_auto_wait=true;
		g_auto_status=AUTO_READY;
	}


	if(g_auto_status==AUTO_SEWING_PUSH)
		autoSewing();
}

void emergnecyMode(void)
{

}





void waitAutoSewing(void)
{
	switch (g_autosewing_status)
	{
		case AUTO_SEWING_RUN:
			TIM4->CCR1 = 0;	
			TIM4->CCR2 = 0;	
			break;
		case AUTO_MOVE_BEFOREHOME:
			TIM4->CCR3 = 0;	
			break;
		default : 
			break;
	}
}

void autoSewing(void)
{
	if(g_sewing_err)
	{
		printf("g_sewing_err==>[%d]",g_sewing_err);
		return;
	}
	switch (g_autosewing_status)
	{
		case AUTO_INIT:
			//시작시 필요한 초기화
			g_autosewing_status = AUTO_MOVING_HOME_CHECK;
			g_needle_servor_status=OFF;
			g_looper_servor_status=OFF;
			g_lifting_servor_status=OFF;
			g_moving_servor_status=OFF;
			g_rc_servor_status=OFF;
			break;
		case AUTO_MOVING_HOME_CHECK:
			g_autosewing_status = AUTO_NEDDLE_HOME_CHECK;
			/*if(SEN_MOVING_HOME == 1)//1: ON, 0: OFF
				g_autosewing_status = AUTO_NEDDLE_HOME_CHECK;
			else
				g_sewing_err = MOVING_INIT_FAIL;*/
			break;
		case AUTO_NEDDLE_HOME_CHECK:
			initNeedle1();
			if(g_needle_servor_status==OFF&&SEN_NEEDLE_HOME == 0)//0: ON, 1: OFF
				g_autosewing_status = AUTO_LOOPER_HOME_CHECK;
			else if(g_needle_servor_status==ON)
				;
			else
				g_sewing_err = NEEDLE_INIT_FAIL;
			break;
		case AUTO_LOOPER_HOME_CHECK:
			initLooper1();
			if(g_looper_servor_status==OFF&&SEN_LOOPER_HOME == 0)//0: ON, 1: OFF
				g_autosewing_status = AUTO_LIFTING_HOME_CHECK;
			else if(g_looper_servor_status==ON)
				;
			else
				g_sewing_err = LOOPER_INIT_FAIL;
			break;
		case AUTO_LIFTING_HOME_CHECK:
			initLifting1();
			if(g_lifting_servor_status==OFF&&SEN_LIFTING_HOME == 1)//1: ON, 0: OFF
				g_autosewing_status = AUTO_BANDCLAMP_CLOSE_CHECK;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = LIFTING_INIT_FAIL;
			break;
		case AUTO_BANDCLAMP_CLOSE_CHECK:
			exeClamp1(band_clamp_1_on,RESET);
			exeClamp1(band_clamp_2_on,RESET);
			if(SEN_FABRIC_CHECK1 == 1&&SEN_FABRIC_CHECK2 == 1)//1: ON, 0: OFF
				g_autosewing_status = AUTO_FABLICCLAMP_CLOSE_CHECK;
			else
				g_sewing_err = BANDCLAMP_INIT_FAIL;
			break;
		case AUTO_FABLICCLAMP_CLOSE_CHECK:
			exeClamp1(fablic_clamp_1_on,RESET);
			exeClamp1(fablic_clamp_2_on,RESET);
			g_autosewing_status = AUTO_LIFTUP_10MM_UP;
			break;
		case AUTO_LIFTUP_10MM_UP:
			exeLifting1(15,ON_UP);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVE_100MM;
			else if(g_lifting_servor_status==ON_UP||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVE_100MM:
			exeMoving1(100,g_moving_speed,ON_RIGHT);
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_LIFTUP_10MM_DOWN;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_MOVE_100MM_FAIL;//time check
			break;
		case AUTO_LIFTUP_10MM_DOWN:
			exeLifting1(15,ON_DOWN);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_SEWING_RUN;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_SEWING_RUN:
			exeAutoSewing1();
			if(g_sewing_servor_status==OFF)
				g_autosewing_status = AUTO_LOOPER_BACK;
				//g_auto_status = AUTO_READY;	
			else if(g_sewing_servor_status==ON || g_sewing_servor_status==MOVE_END )
				;
			else
				g_sewing_err = AUTO_SEWING_RUN_FAIL;//time check
			break;
		case AUTO_LOOPER_BACK:
			exeLooper1(150,BACKWORD);
			if(g_looper_servor_status==OFF)
				g_autosewing_status = AUTO_SEWINGMOVE_100MM;
			else if(g_looper_servor_status==ON_FORWORD||g_looper_servor_status==ON_BACKWORD)
				;
			else
				g_sewing_err = AUTO_LOOPER_BACK_FAIL;//time check
			break;
		case AUTO_SEWINGMOVE_100MM:
			exeSewingMoving1(100,FRQ_2KHz);
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_LOOPER_FRONT;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_SEWINGMOVE_100MM_FAIL;//time check
			break;
		case AUTO_LOOPER_FRONT:
			exeLooper1(150,FORWORD);
			if(g_looper_servor_status==OFF)
				g_autosewing_status = AUTO_VOCCUM_ON;
			else if(g_looper_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_LOOPER_FRONT_FAIL;//time check
			break;
		case AUTO_VOCCUM_ON:
			exeVaccum1(vaccum_on,RESET);
			g_autosewing_status = AUTO_HEATING_ON;
			break;
		case AUTO_HEATING_ON:
			exelHeat1(heating_on,RESET);
			g_autosewing_status = AUTO_RC_TOP;
			break;
		case AUTO_RC_TOP:
			exeRC1(RC_INCREASE);
			if(g_rc_servor_status==OFF)
			{
				g_autosewing_status = AUTO_CUTDELAY;
				g_timer_rc_delay= OFF;
			}
			else if(g_rc_servor_status==RC_INCREASE)
				;
			else
				g_sewing_err = AUTO_RC_TOP_FAIL;//time check
			break;
		case AUTO_CUTDELAY:
			exeDelay1(2,TIMECHECK_CUT);
			if(g_timer_cut_delay==OFF)
				g_autosewing_status = AUTO_HEATING_OFF;
			break;
		case AUTO_HEATING_OFF:
			exelHeat1(heating_on,SET);
			g_autosewing_status = AUTO_RC_HOME;
			break;
		case AUTO_RC_HOME:
			exeRC1(RC_DECREASE);
			if(g_rc_servor_status==OFF)
				g_autosewing_status = AUTO_FABLICCLAMP_OPEN_CHECK;
			else if(g_rc_servor_status==RC_DECREASE)
				;
			else
				g_sewing_err = AUTO_RC_HOME_FAIL;//time check
			break;
		case AUTO_FABLICCLAMP_OPEN_CHECK:
			exeClamp1(fablic_clamp_1_on,SET);
			exeClamp1(fablic_clamp_2_on,SET);
			g_autosewing_status = AUTO_BANDCLAMP_OPEN_CHECK;
			break;
		case AUTO_BANDCLAMP_OPEN_CHECK:
			exeClamp1(band_clamp_1_on,SET);
			exeClamp1(band_clamp_2_on,SET);
			g_autosewing_status = AUTO_LIFTUP_10MM_2;
			break;
		case AUTO_LIFTUP_10MM_2:
			exeLifting1(10,ON_UP);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVE_BEFOREHOME;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVE_BEFOREHOME:
			exeMoving1(100,g_moving_speed,ON_LEFT);
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_LIFTUP_10MM_3;
			else if(g_moving_servor_status==ON_RIGHT||g_moving_servor_status==ON_LEFT)
				;
			else
				g_sewing_err = AUTO_MOVE_100MM_FAIL;//time check
			break;
		case AUTO_LIFTUP_10MM_3:
			exeLifting1(10,ON_DOWN);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVING_HOME_CHECK2;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVING_HOME_CHECK2:
			initMoving1();
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_BANDCLAMP_CLOSE_CHECK_2;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_MOVING_HOME_CHECK2_FAIL;
			break;
		case AUTO_BANDCLAMP_CLOSE_CHECK_2:
			exeClamp1(band_clamp_1_on,RESET);
			exeClamp1(band_clamp_2_on,RESET);
			if(SEN_FABRIC_CHECK1 == 1&&SEN_FABRIC_CHECK2 == 1)//1: ON, 0: OFF
				g_autosewing_status = AUTO_VOCCUM_OFF;
			else
				g_sewing_err = BANDCLAMP_INIT_FAIL;
			break;
		case AUTO_VOCCUM_OFF:
			printf("AUTO_VOCCUM_OFF\n");
			exeVaccum1(vaccum_on,SET);
			g_autosewing_status = AUTO_NEDDLE_HOME_CHECK2;
			break;
		case AUTO_NEDDLE_HOME_CHECK2:
			initNeedle1();
			if(g_needle_servor_status==OFF&&SEN_NEEDLE_HOME == 0)//0: ON, 1: OFF
				g_auto_status = AUTO_READY;	
			else if(g_needle_servor_status==ON)
				;
			else
				g_sewing_err = NEEDLE_INIT_FAIL;
			break;
		default : 
			break;
	}
}


void initNeedle1(void)
{
	if(SEN_NEEDLE_HOME == 0) //0: ON, 1: OFF
	{
		servorStop(TIM_CHANNEL_1);	// needle servo stop
		g_manual_status=MANUAL_READY;
		g_needle_servor_status=OFF;
	}
	else 
	{
		switch (g_needle_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_1,g_init_speed,ROTATE_TYPE_CCW);
				g_needle_servor_status = ON;
				break;
			case ON:
				//time over err
				break;
			default : 
				break;
		}
	}
}

void initLooper1(void)
{
	if(SEN_LOOPER_HOME == 0) //0: ON, 1: OFF
	{
		servorStop(TIM_CHANNEL_2);	// needle servo stop
		g_manual_status=MANUAL_READY;
		g_looper_servor_status=OFF;
	}
	else // needle home sensor off
	{
		switch (g_looper_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_2,g_init_speed,ROTATE_TYPE_CCW);
				g_looper_servor_status = ON;
				break;
			case ON:
				//time over err
				break;
			default : 
				break;
		}
	}
}


void initLifting1(void)
{
	if( g_lifting_servor_status!=ON_UP && SEN_LIFTING_HOME == 1 ) //1: ON, 0: OFF
	{
		servorStop(TIM_CHANNEL_4);	
		g_manual_status=MANUAL_READY;
		g_lifting_servor_status=OFF;
		printf("init OFF\n");
	}
	else // needle home sensor off
	{
		switch (g_lifting_servor_status)
		{
			case OFF://SV4_DIR_CW; 
				servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CW);
				g_lifting_servor_status = ON_UP;
				break;
			case ON_UP:
				if(SEN_LIFTING_UP_LIM == 1)//1: ON, 0: OFF
				{
					printf("top sensor sensor on");
					servorStop(TIM_CHANNEL_4);	
					servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CCW);
					g_lifting_servor_status=ON_DOWN;
				}
				break;
			case ON_DOWN:
				if(SEN_LIFTING_DN_LIM == 1)//1: ON, 0: OFF
					servorStop(TIM_CHANNEL_4);//error
				break;
			default : 
				break;
		}
	}
}


void initMoving1(void)
{
	if(SEN_MOVING_HOME == 1)//1: ON, 0: OFF 
	{
		servorStop(TIM_CHANNEL_3);	
		g_manual_status=MANUAL_READY;
		g_moving_servor_status=OFF;
		printf("init OFF\n");
	}
	else 
	{
		switch (g_moving_servor_status)
		{
			case OFF://SV4_DIR_CW; 
				servorStart(TIM_CHANNEL_3,FRQ_2KHz,ROTATE_TYPE_CCW);//FRQ_200Hz g_moving_speed
				g_moving_servor_status = ON;
				break;
			case ON:
				//time over err
				break;
			default : 
				break;
		}
	}
}

void waitManual(void)
{
	switch (g_manual_status)
	{
		case INIT_NEEDLE_PUSH:
			servorStop(TIM_CHANNEL_1);	// needle servo stop
			g_needle_servor_status=OFF;
			break;
		case INIT_LOOFER_PUSH:
			servorStop(TIM_CHANNEL_2);	// needle servo stop
			g_looper_servor_status=OFF;
			break;
		case INIT_LIFTING_PUSH:
			servorStop(TIM_CHANNEL_4);	// needle servo stop
			g_lifting_servor_status=OFF;
			printf("PUSH OFF\n");
			break;
		case INIT_MOVING_PUSH:
			servorStop(TIM_CHANNEL_3);	
			g_moving_servor_status=OFF;
			break;
		case INIT_TEST_PUSH:
			g_sewing_servor_status=MOVE_END;
			g_move_total_count = g_move_target_count;
			break;
		default : 
			break;
	}
}

void exeTestSewing(unsigned int sewingLength,unsigned int Speed)
{
	
	if(g_sewing_servor_status==OFF && (SEN_NEEDLE_HOME == 1 || SEN_LOOPER_HOME == 1)) //1: ON, 0: OFF
	{
		g_manual_status=MANUAL_READY;
		g_sewing_servor_status=OFF;
		return;
	}

	if(g_sewing_servor_status==COMPLATE) 
	{
		g_manual_status=MANUAL_READY;
		g_sewing_servor_status=OFF;
	}
	else
	{
		switch (g_sewing_servor_status)
		{
			case OFF:
				g_move_target_count=(unsigned long)((float)sewingLength/ONE_PULSE_MV);// g_test_sewing_length g_test_sewing_speed
				g_needle_puls_count=0;
				g_move_count=0;
				g_move_total_count=0;
				servorStart(TIM_CHANNEL_1,Speed,ROTATE_TYPE_CCW);
				servorStart(TIM_CHANNEL_2,Speed,ROTATE_TYPE_CCW);
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
				g_sewing_servor_status = ON;
				break;
			case ON:
				//time over err
				break;
			case MOVE_END:
				//needle looper init
				break;
			default : 
				break;
		}	
	}
}


void exeRotaryEncorder(void)
{
	if(g_prev_rotaryencorder_z_on==0 && SW_ROE_Z==1)//SWITCH 0:ON  1:OFF  ===>   Z-push button rising
			g_sewing_servor_status=MOVE_END;
	if( (SW_ROE_X == 1) && (SW_ROE_Y == 1) && (SW_ROE_Z == 1) )//SWITCH 0: ON, 1: OFF
	{
		g_manual_status=MANUAL_READY;
		g_rotaryencorder_count=0;
		g_move_count=0;
		g_lift_count=0;
		g_rotaryencorder_status=OFF;
		g_prev_rotaryencorder_z_on=IN_PORT_DATA[3].bit5;

		return;
	}
	
	switch (g_rotaryencorder_status)
	{
		case OFF:
			g_prev_rorate=FORWORD;
			if(SW_ROE_X == 0)//SWITCH 0: ON, 1: OFF
			{
				g_rotaryencorder_status=RIGHTLEFT_MOVE;
				TIM4->ARR = FRQ_15KHz;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0); // pwm gen off
				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

			}
			else if(SW_ROE_Y == 0)//SWITCH 0: ON, 1: OFF
			{
				g_rotaryencorder_status=UPDOWN_MOVE;
				TIM4->ARR = FRQ_5KHz;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0); // pwm gen off
				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
			}	
			else if(SW_ROE_Z == 0)//SWITCH 0: ON, 1: OFF
				g_rotaryencorder_status=JOG_MOVE;
			break;
		case RIGHTLEFT_MOVE:
			if(g_prev_rorate!=g_rotaryencorder_rorate)
			{
				g_rotaryencorder_count=0;	
				g_move_count=0;
			}
			printf("[%d]:[%d]\n",g_move_count,g_rotaryencorder_count);
			if(g_move_count<(((float)g_rotaryencorder_count*1)/ONE_PULSE_MV))
			{
				if(g_rotaryencorder_rorate==FORWORD)
					SV3_DIR_CCW;
				else
					SV3_DIR_CW;
				g_prev_rorate=g_rotaryencorder_rorate;
				if(SEN_MOVING_HOME == 0 && g_rotaryencorder_rorate==BACKWORD)// 1: ON, 0: OFF
					TIM4->CCR3 = (FRQ_15KHz+1)/2;
				else if(g_rotaryencorder_rorate==FORWORD)
					TIM4->CCR3 = (FRQ_15KHz+1)/2;
				else
					TIM4->CCR3 = 0;	
			}
			else
			{
				TIM4->CCR3 = 0;	
			}
			break;
		case UPDOWN_MOVE:
			if(g_prev_rorate!=g_rotaryencorder_rorate)
			{
				g_rotaryencorder_count=0;	
				g_lift_count=0;
			}
			printf("[%d]:[%d]\n",g_move_count,g_rotaryencorder_count);
			if(g_lift_count<g_rotaryencorder_count*16)
			{
				if(g_rotaryencorder_rorate==FORWORD)
					SV4_DIR_CCW;
				else
					SV4_DIR_CW;
				g_prev_rorate=g_rotaryencorder_rorate;
				if(SEN_LIFTING_UP_LIM == 0 && g_rotaryencorder_rorate==FORWORD)// 1: ON, 0: OFF
					TIM4->CCR4 = (FRQ_5KHz+1)/2;
				else if(SEN_LIFTING_DN_LIM == 0 && g_rotaryencorder_rorate==BACKWORD)// 1: ON, 0: OFF
					TIM4->CCR4 = (FRQ_5KHz+1)/2;
				else 
					TIM4->CCR4 = 0;
			}
			else
			{
				TIM4->CCR4 = 0;
			}
			break;
		case JOG_MOVE:
			g_prev_rotaryencorder_z_on=SW_ROE_Z;//SWITCH 0:ON  1:OFF
			exeTestSewing(2000,FRQ_500Hz);
			break;
		default : 
			break;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin==GPIO_PIN_0)//needle servor 
	{
		if(g_sewing_servor_status == ON  || g_sewing_servor_status == MOVE_END)
		{
			g_needle_puls_count++;
			if(g_sewing_servor_status==MOVE_END)
				TIM4->ARR = g_init_speed;   
			if(g_needle_puls_count >= SEW_1CYCLE_PULSE)
			{	
				if(g_sewing_servor_status == MOVE_END)
				{
					servorStop(TIM_CHANNEL_1);
					servorStop(TIM_CHANNEL_2);
					g_needle_puls_count=0;
					g_sewing_servor_status=COMPLATE;
				}
				g_needle_puls_count=0;
			}
			else if(g_sewing_servor_status == ON && g_needle_puls_count == SEW_OFF_FABRIC_PULSE)
			{
				
				unsigned int speed=g_auto_sewing_speed;
				if(g_manual_status==INIT_TEST_PUSH)
					speed=g_test_sewing_speed;
				TIM4->CCR3 = (speed+1)/2;
			}
		}
	}

	if(GPIO_Pin==GPIO_PIN_1)//looper servor 
	{
		if(g_autosewing_status == AUTO_LOOPER_BACK||g_autosewing_status ==AUTO_LOOPER_FRONT)
			g_looper_puls_count++;
	}


	if(GPIO_Pin==GPIO_PIN_2)//moving servor 
	{
		if(g_manual_status==INIT_MOVING_PUSH||g_autosewing_status==AUTO_MOVE_100MM||g_autosewing_status== AUTO_SEWINGMOVE_100MM||g_autosewing_status== AUTO_MOVE_BEFOREHOME|| g_rotaryencorder_status==RIGHTLEFT_MOVE)//
			g_move_count++;				//g_manual_status==INIT_MOVING_PUSH==>for rotaryencorder
		else if(g_sewing_servor_status == ON || g_sewing_servor_status == MOVE_END)
		{
			g_move_count++;
			g_move_total_count++;
			if(g_move_total_count >= g_move_target_count)	// max sewing moving done -> all stop
			{
				TIM4->CCR3 = 0;
				g_move_total_count=0;
				g_move_count=0;
				g_sewing_servor_status=MOVE_END;
			}
			else
			{
				if(g_move_count == MOV_1CYCLE_PULSE)	// moving motor 1 cycle running
				{
					TIM4->CCR3 = 0;	// moving stop
					g_move_count = 0;
				}
			}
		}
	}

	if(GPIO_Pin==GPIO_PIN_3)//lifting servor   AUTO_LIFTUP_10MM_2
	{
		if(g_autosewing_status == AUTO_LIFTUP_10MM_UP||g_autosewing_status ==AUTO_LIFTUP_10MM_DOWN||g_autosewing_status ==AUTO_LIFTUP_10MM_2||g_autosewing_status ==AUTO_LIFTUP_10MM_3||g_rotaryencorder_status==UPDOWN_MOVE)
			g_lift_count++;
	}

	if(GPIO_Pin==GPIO_PIN_5)
	{
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == 0) // CW
			g_rotaryencorder_rorate = FORWORD;
		else	// ccw
			g_rotaryencorder_rorate = BACKWORD;
		g_rotaryencorder_count++; 
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		if(g_timer_cut_delay)
			g_timer_cut_count++;
		if(g_timer_rc_delay)
			g_timer_rc_count++;
	}
	
}



void servorStart(uint32_t Channel,unsigned int Arr,unsigned char Rotate)
{
	HAL_TIM_PWM_Stop(&htim4,Channel);
	if(Channel==TIM_CHANNEL_1)
	{
		outportSignal(needle_servo_on,RESET);
		HAL_GPIO_WritePin(GPIOD,SV_SIGN1,Rotate);  //SV1_DIR_CCW; // needle ac servo ccw
	}
	else if(Channel==TIM_CHANNEL_2)
	{
		outportSignal(looper_servo_on,RESET);
		HAL_GPIO_WritePin(GPIOD,SV_SIGN2,Rotate);
	}
	else if(Channel==TIM_CHANNEL_3)
	{
		outportSignal(moving_servo_on,RESET);
		HAL_GPIO_WritePin(GPIOD,SV_SIGN3,Rotate);
	}
	else if(Channel==TIM_CHANNEL_4)
	{
		outportSignal(updown_servo_on,RESET);
		HAL_GPIO_WritePin(GPIOD,SV_SIGN4,Rotate);
	}
	if(Arr!=0)
		TIM4->ARR = Arr;   
	__HAL_TIM_SET_COMPARE(&htim4,Channel,(Arr+1)/2);
	HAL_TIM_PWM_Start(&htim4, Channel);
}
void servorStop(uint32_t Channel)
{
	__HAL_TIM_SET_COMPARE(&htim4,Channel,0);
	HAL_TIM_PWM_Stop(&htim4, Channel);	// needle servo stop
}

void exeClamp1(unsigned char outportName, unsigned char Mode)
{
	outportSignal(outportName,Mode);
}

void exeVaccum1(unsigned char outportName, unsigned char Mode)
{
	outportSignal(outportName,Mode);
}

void exelHeat1(unsigned char outportName, unsigned char Mode)
{
	outportSignal(outportName,Mode);
}


void exeLifting1( int Move_length, unsigned char Rotate)
{
	g_lift_target_count = Move_length*160;	// 10mm lifting 5mm:800 => 10mm:1600 => 1mm:160
	if(g_lift_count >= g_lift_target_count)	// max sewing moving done -> all stop
	{
		servorStop(TIM_CHANNEL_4);	
		g_lift_count=0;
		g_lifting_servor_status=OFF;
	}
	else 
	{
		switch (g_lifting_servor_status)
		{
			case OFF:
				if(Rotate==ON_UP)
				{
					servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CW);
					g_lifting_servor_status = ON_UP;
				}
				else
				{
					servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CCW);
					g_lifting_servor_status = ON_DOWN;
				}
				break;
			case ON_UP:
				break;
			case ON_DOWN:
				break;
			default : 
				break;
		}
	}
}



void exeMoving1( int Move_length,unsigned int Speed,unsigned char Rotate)
{
	g_move_target_count = (unsigned long)((float)Move_length/ONE_PULSE_MV);
	if(g_move_count >= g_move_target_count)	// max sewing moving done -> all stop
	{
		servorStop(TIM_CHANNEL_3);	
		g_move_count=0;
		g_moving_servor_status=OFF;
	}
	else // needle home sensor off 
	{
		switch (g_lifting_servor_status)
		{
			case OFF://SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
				if(Rotate==ON_RIGHT)
				{
					servorStart(TIM_CHANNEL_3,Speed,ROTATE_TYPE_CW);
					g_moving_servor_status = ON_RIGHT;
				}
				else
				{
					servorStart(TIM_CHANNEL_3,Speed,ROTATE_TYPE_CCW);
					g_moving_servor_status = ON_LEFT;
				}
				break;
			case ON_RIGHT:
			case ON_LEFT:
				if(g_auto_wait==true)
				{
					TIM4->CCR3 = (Speed+1)/2;
					g_auto_wait=false;
				}
				break;
			default : 
				break;
		}
	}
}

void exeAutoSewing1(void)
{
	if(g_sewing_servor_status==COMPLATE) // needle home sensor on -> stop
	{
		g_sewing_servor_status = OFF;
	}
	else
	{
		switch (g_sewing_servor_status)
		{
			case OFF:
				g_move_target_count=(unsigned long)((float)g_auto_sewing_length/ONE_PULSE_MV);
				g_needle_puls_count=0;
				g_move_count=0;
				g_move_total_count=0;
				g_sewing_servor_status = ON;
				servorStart(TIM_CHANNEL_1,g_auto_sewing_speed,ROTATE_TYPE_CCW);
				servorStart(TIM_CHANNEL_2,g_auto_sewing_speed,ROTATE_TYPE_CCW);
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
				break;
			case ON:
				if(g_auto_wait==true)
				{
					TIM4->CCR1 = (g_auto_sewing_speed+1)/2;	
					TIM4->CCR2 = (g_auto_sewing_speed+1)/2;	
					g_auto_wait=false;
				}
				break;
			case MOVE_END:
				//needle looper init
				break;
			default : 
				break;
		}	
	}
}


void exeLooper1(unsigned int Move_Pulse,unsigned char Rotate)
{
	g_move_target_count = Move_Pulse;
	if(g_looper_puls_count >= g_move_target_count)	// max sewing moving done -> all stop
	{
		servorStop(TIM_CHANNEL_2);	
		g_looper_puls_count=0;
		g_looper_servor_status=OFF;
	}
	else 
	{
		switch (g_looper_servor_status)
		{
			case OFF:
				
				if(Rotate==FORWORD)
				{
					servorStart(TIM_CHANNEL_2,g_lifting_speed,ROTATE_TYPE_CCW);
					g_looper_servor_status = ON_FORWORD;
				}
				else
				{
					servorStart(TIM_CHANNEL_2,g_lifting_speed,ROTATE_TYPE_CW);
					g_looper_servor_status = ON_BACKWORD;
				}
				break;
			case ON_FORWORD:
			case ON_BACKWORD:
			default : 
				break;
		}
	}
}


void exeSewingMoving1(unsigned int Move_length,unsigned int Speed)
{
	g_move_target_count = (unsigned long)((float)Move_length/ONE_PULSE_MV);
	if(g_move_count >= g_move_target_count)	// max sewing moving done -> all stop
	{
		TIM4->CCR3 = 0;	
		TIM4->CCR1 = 0;	
		g_move_count=0;
		g_moving_servor_status=OFF;
	}
	else // needle home sensor off 
	{
		switch (g_moving_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_1,Speed,ROTATE_TYPE_CCW);
				servorStart(TIM_CHANNEL_3,Speed,ROTATE_TYPE_CW);
				g_moving_servor_status=ON;
				break;
			case ON:
				break;
			default : 
				break;
		}
	}
}


void exeDelay1(unsigned int Delay_time,unsigned char Mode)
{
	if(Mode==TIMECHECK_CUT)
	{
		g_timer_cut_delay=1;
		if(g_timer_cut_count>=Delay_time*1000)
		{
			g_timer_cut_count=0;
			g_timer_cut_delay=false;
		}
	}
}

void exeRC1(unsigned char Mode)
{
	switch (g_rc_servor_status)
	{
		case OFF://SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
			if(Mode == RC_INCREASE)
			{
				g_rcDuty[0] = RC_DUTY_MIN;
				g_rcDuty[1] = RC_DUTY_MAX;
				g_timer_rc_delay=ON;
			}
			else
			{
				g_rcDuty[0] = RC_DUTY_MAX;
				g_rcDuty[1] = RC_DUTY_MIN;
			}
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,g_rcDuty[0]);	// duty set rc servo1
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,g_rcDuty[1]);
	        HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);    // rc servo1 pwm start
	        HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);    // rc servo2 pwm start
			g_rc_servor_status=Mode;
			break;
		case RC_INCREASE:
			if(g_timer_rc_count>=g_rc_delay_time)
			{
				g_rcDuty[0] += RC_DUTY_STEP;
				g_rcDuty[1] -= RC_DUTY_STEP;
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,g_rcDuty[0]);	// duty set rc servo1
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,g_rcDuty[1]);	// duty set rc servo2
				if((g_rcDuty[0] > RC_DUTY_MAX) && (g_rcDuty[1] < RC_DUTY_MIN)) // moving top end
					g_rc_servor_status = OFF;
				
				g_timer_rc_count=0;
			}
			break;
		case RC_DECREASE:
			g_rcDuty[0] -= RC_DUTY_STEP;
            g_rcDuty[1] += RC_DUTY_STEP;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,g_rcDuty[0]);  // duty set rc servo1
        	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,g_rcDuty[1]);
			if((g_rcDuty[0] < RC_DUTY_MIN) && (g_rcDuty[1] > RC_DUTY_MAX)) // end
                g_rc_servor_status = OFF;
			break;
		default : 
			break;
	}
}




void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time */
	//sysClk = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	sDate.Month = RTC_MONTH_DECEMBER;
	sDate.Date = 0x1;
	sDate.Year = 0x21;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	//hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	//hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	//hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	//hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	//hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	//hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
	    Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
	/* USER CODE BEGIN TIM2_Init 0 */
    // 1msec system tick timer
	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
	/* USER CODE BEGIN TIM3_Init 0 */
    // RC SERVO1 PWM
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = FRQ_RC_SV;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
	/* USER CODE BEGIN TIM4_Init 0 */
    // AC SERVO1,2,3,4 PWM
	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	//sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{
	/* USER CODE BEGIN TIM8_Init 0 */
    // RC SERVO2 PWM
	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 71;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = FRQ_RC_SV;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	//sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	
	/*PORTA -------------------------------------------------------------------------------------*/
    #if 0
	/*Configure GPIO pins : UART2_RW, SYS_RUN, EEP_WP */
	GPIO_InitStruct.Pin = UART2_RW|SYS_RUN|EEP_WP;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, UART2_RW|SYS_RUN|EEP_WP, GPIO_PIN_RESET);
    #else
    /*Configure GPIO pins : UART2_RW, SYS_RUN, EEP_WP */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, UART2_RW|SYS_RUN|EEP_WP, GPIO_PIN_RESET);
    #endif

	/*PORTB -------------------------------------------------------------------------------------*/
	#if 0
    /*Configure GPIO pins : OUT_CLK1, OUT_CLK2, OUT_CLK3, OUT_CLK4, OUT_CLK5, OUT_CLK7, OUT_CLK8, OUT_CLK6 */
	GPIO_InitStruct.Pin = OUT_CLK1|OUT_CLK2|OUT_CLK3|OUT_CLK4|OUT_CLK5|OUT_CLK7|OUT_CLK8|OUT_CLK6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ROE_B */
	GPIO_InitStruct.Pin = ROE_B;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	//HAL_GPIO_WritePin(GPIOB, OUT_CLK1|OUT_CLK2|OUT_CLK3|OUT_CLK4|OUT_CLK5|OUT_CLK7|OUT_CLK8|OUT_CLK6, GPIO_PIN_RESET);
    #else
    /*Configure GPIO pins : OUT_CLK1, OUT_CLK2, OUT_CLK3, OUT_CLK4, OUT_CLK5, OUT_CLK7, OUT_CLK8, OUT_CLK6 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   
	/*Configure GPIO pin : ROE_B */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);
    #endif

	/*PORTC -------------------------------------------------------------------------------------*/
    #if 0
	/*Configure GPIO pins : IN_EN1, IN_EN2, IN_EN3, IN_EN4, IN_EN5, IN_EN7, IN_EN8, UART1_RW */
	GPIO_InitStruct.Pin = IN_EN1|IN_EN2|IN_EN3|IN_EN4|IN_EN5|IN_EN7|IN_EN8|UART1_RW;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_EN6 */
	GPIO_InitStruct.Pin = IN_EN6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, IN_EN1|IN_EN2|IN_EN3|IN_EN4|IN_EN5|IN_EN6|IN_EN7|IN_EN8|UART1_RW, GPIO_PIN_RESET);
    #else
    /*Configure GPIO pins : IN_EN1, IN_EN2, IN_EN3, IN_EN4, IN_EN5, IN_EN7, IN_EN8, UART1_RW */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_EN6 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    #endif

	/*PORTD -------------------------------------------------------------------------------------*/
    #if 0
	/*Configure GPIO pins : SV_SIGN1_Pin SV_SIGN2_Pin SV_SIGN3_Pin SV_SIGN4_Pin */
	GPIO_InitStruct.Pin = SV_SIGN1|SV_SIGN2|SV_SIGN3|SV_SIGN4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : ID_D0, ID_D1, ID_D2, ID_D3 */
	GPIO_InitStruct.Pin = ID_D0|ID_D1|ID_D2|ID_D3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, SV_SIGN1|SV_SIGN2|SV_SIGN3|SV_SIGN4, GPIO_PIN_RESET);
    #else
    /*Configure GPIO pins : SV_SIGN1_Pin SV_SIGN2_Pin SV_SIGN3_Pin SV_SIGN4_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : SV_SIGN1_Pin SV_SIGN2_Pin SV_SIGN3_Pin SV_SIGN4_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : ID_D0, ID_D1, ID_D2, ID_D3 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
    #endif

	/*PORTE -------------------------------------------------------------------------------------*/
    #if 0
	/*Configure GPIO pins : IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7  */
	GPIO_InitStruct.Pin = IN0|IN1|IN2|IN3|IN4|IN5|IN6|IN7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : OUT0, OUT1, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7 */
	GPIO_InitStruct.Pin = OUT0|OUT1|OUT2|OUT3|OUT4|OUT5|OUT6|OUT7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, OUT0|OUT1|OUT2|OUT3|OUT4|OUT5|OUT6|OUT7, GPIO_PIN_RESET);
    #else
    /*Configure GPIO pins : IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7  */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : OUT0, OUT1, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
    #endif

	/*EXTERNAL TI -------------------------------------------------------------------------------*/
    #if 0
	/*Configure GPIO pin : ROE_A */
	GPIO_InitStruct.Pin = ROE_A;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SV_PULSE1_IN, SV_PULSE2_IN, SV_PULSE3_IN, SV_PULSE4_IN */
	GPIO_InitStruct.Pin = SV_PULSE1_IN|SV_PULSE2_IN|SV_PULSE3_IN|SV_PULSE4_IN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    #else
    /*Configure GPIO pin : ROE_A */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SV_PULSE1_IN, SV_PULSE2_IN, SV_PULSE3_IN, SV_PULSE4_IN */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    #endif

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief Variables Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

