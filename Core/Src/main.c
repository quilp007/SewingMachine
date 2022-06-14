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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "vfdn_io_cntl.h"
#include "vfdn_adc.h"
#include "vfdn_spi.h"
#include "dwt_stm32_delay.h"

#include <stdio.h>
#include <stdbool.h>

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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void _write(int file, uint8_t* p, int len)
{
	HAL_UART_Transmit(&huart4, p, len, 500);
	//HAL_UART_Transmit(&huart3, p, len, 500);
}


void errInitChangeSwitch(void)				//manual모드나  auto모드 에서 SWITCH변경시  이전 모드의 동작이 진행 중일경우   에러처리...
{
	g_lamp_mode=RED_ERROR;
	g_sewing_err=222;
	servorStop(TIM_CHANNEL_1);
	servorStop(TIM_CHANNEL_2);
	servorStop(TIM_CHANNEL_3);
	servorStop(TIM_CHANNEL_4);
	exeOutportSignal(heating_on,SET);
	exeRC(RC_DECREASE);
	exeOutportSignal(vaccum_on,SET);
	g_manual_status=MANUAL_READY;
	g_auto_status=AUTO_READY;
	g_change_sw_auto_manual=false;
	g_autosewing_status=AUTO_INIT;
	g_auto_wait = OFF;					//????????????????????

	g_needle_servor_status=OFF;
	g_looper_servor_status=OFF;
	g_moving_servor_status=OFF;
	g_sewing_servor_status=OFF;
	g_rotaryencorder_status=OFF;

	g_lenth_encode_count=0;						//무창기계관련 동작 초기화.....제어할수 있는 입력버튼이 없어서 메뉴얼모드에서 조작불가하기때문에....
	g_etc_machine_status=ETC_MACHINE_READY;
	g_fablic_cut_status=OFF;
	exeEtcMachine(etc_machine_on,RESET);	
	exeOutportSignal(fablic_heating_on,RESET);
	exeOutportSignal(cut_cylinder_on,SET);
	exeOutportSignal(loader_updown_on,SET);
	exeOutportSignal(loader_pushpull_on,SET);


	g_etc_slow_count=0;
	g_prev_etcmachine_home=OFF;


	g_topstop_flag=false;
}

uint8_t checkPrevRunStatus(void)					//manual모드나  auto모드 에서 SWITCH변경시  이전 모드의 동작이 진행 중인지를  판단하여    결과 리턴
{
	if(g_change_sw_auto_manual)						//false : sw automanual변경시 이전 동작이 완료 되었음   true:이전  동작이 진행 중임==>이경우  진행 중인 동작을 정지시킴 
		return g_change_sw_auto_manual;
	uint8_t curValue=SW_AUTO_MANUAL;
	if(g_prev_sw_auto_manual==2)					//최초 g_prev_sw_auto_manual 값은 2.......initVariables()는 readSignalProcess()전에 호출되므로 초기값이 필요
		g_prev_sw_auto_manual=curValue;

	if(g_prev_sw_auto_manual!=curValue)
	{
		switch(g_prev_sw_auto_manual)				//[SWITCH] Mode select  1: MANUAL MODE, 0: AUTO MODE
		{
			case 1:									//이전동작이 manual이므로 g_manual_status상태를 체크
				if(g_manual_status!=MANUAL_READY)
					g_change_sw_auto_manual=true;
				else
				{
					g_change_sw_auto_manual=false;
					g_auto_wait=OFF;				//SW_AUTO_STARTf ON시에   if(g_auto_wait==OFF)에 의해 초기화		
					g_prev_lamp_mode=YELLOW_RUN;
				}
				//메뉴얼모드에서 오토모드로 변경시 최초 loader를 초기화
				exeOutportSignal(loader_updown_on,RESET);
				exeOutportSignal(loader_pushpull_on,RESET);
				break;
			case 0:
				if(g_auto_status==AUTO_READY&&g_autosewing_status==AUTO_INIT&&g_etc_machine_status==ETC_MACHINE_READY)				//auto모드에서 아무 동작을 하지 않은 경우...
				{
					g_change_sw_auto_manual=false;
					g_prev_lamp_mode=YELLOW_RUN;
				}
				else																												//auto에서 한번 동작하게 되면 오토메뉴얼 스의치 변경시 무조건 에러....
					g_change_sw_auto_manual=true;
				break;
			default : 
				break;
		}
	}
	g_prev_sw_auto_manual=curValue;
	return g_change_sw_auto_manual;
}

void initVariables(void)
{
	// initialize OUTPUT PORT!!!!!!!!!!!!!!
	OUT_PORT_DATA[OUT_CH0].data=0b00000010;		//outportSignal(needle_servo_on,RESET);    outportSignal(needle_alram_reset,SET);
	OUT_PORT_DATA[OUT_CH1].data=0b00000010;		//outportSignal(looper_servo_on,RESET);    outportSignal(looper_alram_reset,SET);
	OUT_PORT_DATA[OUT_CH2].data=0b00000010;		//outportSignal(moving_servo_on,RESET);    outportSignal(moving_alram_reset,SET);
	OUT_PORT_DATA[OUT_CH3].data=0b00000010;		//outportSignal(updown_servo_on,RESET);    outportSignal(updown_alram_reset,SET);
	OUT_PORT_DATA[OUT_CH4].data=0b11111111;
	OUT_PORT_DATA[OUT_CH5].data=0b11111111;
	OUT_PORT_DATA[OUT_CH6].data=0b11111111;
	OUT_PORT_DATA[OUT_CH7].data=0b11111111;
	for(int ch =0; ch<8; ch++)
		outSignal(ch, OUT_PORT_DATA[ch].data);

	DWT_Delay_us(500);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);    				// rc servo1 pwm start
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);    				// rc servo2 pwm start
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,RC_DUTY_MIN);  	// duty set rc servo1
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,RC_DUTY_MIN);	
	exeOutportSignal(etc_machine_on,RESET);						//무창기계 스탑
	exeOutportSignal(etc_machine_decrease,RESET);				//무창기계 감속 정지
	exeOutportSignal(fablic_heating_on,RESET);					//절단용 열선  OFF
	exeOutportSignal(loader_updown_on,RESET);
	exeOutportSignal(loader_pushpull_on,RESET);

	DWT_Delay_us(500);

	g_auto_sewingmove_length	= 170;					//스윙무브 기본길이
	g_attachband_length  		= 300;					//달기밴드 길이
	g_auto_vertical_length		= 1200;					//무창기계 수직봉제길이
	g_fablic_cut_position		= 830;					//원단절단 시작시점(길이)

	
	g_starting_position			= 160;					//자동스윙시작시 최초 재봉시작위치...
	g_auto_sewing_length 		= 1080;					//테스트원단 실제 너비 1065
	g_test_sewing_length 		= 300;//1000mm

	g_auto_sewing_speed = FRQ_15KHz;			//자동모드 속도
	g_test_sewing_speed = FRQ_15KHz;			//테스트모드 기본속도
	g_lifting_speed 	= FRQ_2KHz;				//lifting 기본속도
	g_moving_speed 		= FRQ_2KHz;				//moving 기본속도  FRQ_5KHz
	g_init_speed 		= FRQ_200Hz;				//needle,looper초기화속도


	g_rotaryencorder_status=OFF;
	g_manual_status 		=	MANUAL_READY;	//메뉴얼모드 초기상태
	g_auto_status			=	AUTO_READY;		//오토모드 초기상태
	g_autosewing_status 	=	AUTO_INIT;		//자동봉제 초기상태
	g_needle_servor_status  =	OFF;
	g_looper_servor_status  =	OFF;
	g_moving_servor_status  =	OFF;
	g_sewing_servor_status  =	OFF;
	g_rotaryencorder_status =	OFF;
	g_sewingmoving_servor_status=OFF;			//자동모드에서 봉제후, 커팅전에 sewingmoving상태처리를 위해
	g_auto_wait = OFF;							//자동봉제중에 정지가 된 경우==>재시작시 이전봉제길이 계산하여 남은 봉제길이 작업

	g_prev_lamp_mode	=GREEN_READY;
	g_lamp_mode			=GREEN_READY;
	g_error_count		=0;						//에러시 램프의 점멸시간 설정
	
	g_com_wakeup_flag=false;					//PC연결확인......PC와 연결이 안되면............
	g_prev_sw_auto_manual=2;					//[SWITCH] Mode select  0: MANUAL MODE, 1: AUTO MODE      2:INIT   manual/auto변경여부를 확인하기 위해 이전 모드값 셋팅
	g_change_sw_auto_manual=false;				//manual/auto변경여부

	
	g_etc_machine_status=ETC_MACHINE_READY;
	g_etc_machine_run=OFF;						//PC최초 연결시 해당정보 받아서 셋팅:g_lenth_encode_count...g_auto_wait...g_sewingmoving_servor_status...g_etc_machine_status....g_prev_needle_puls_count
	g_fablic_cut_status=OFF;
	g_loader_status=LOADER_READY;
	g_encode_count_step=ENCODE_COUNT_READY;
	g_prev_encode_count_step=ENCODE_COUNT_READY;
	g_lenth_encode_count=0;
	g_etc_slow_count=0;
	g_topstop_flag=false;
	
}

void manualMode(void)
{
	if(g_manual_status==MANUAL_READY)
	{
		g_lamp_mode=GREEN_READY;						//메뉴얼버튼 off시 램프 대기
		if(SW_NEEDLE_HOME == 0) 											// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_NEEDLE_PUSH;
		else if(SW_LOOPER_HOME == 0)										// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_LOOFER_PUSH;
		if(SW_MOVING_HOME == 0)										// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_MOVING_PUSH;							
		else if(SW_LIFTING_HOME == 0)
			g_manual_status=INIT_LIFTING_PUSH;
		else if(SW_TEST_SEWING == 0)										// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_TEST_PUSH;
		else if( (SW_ROE_X == 0) || (SW_ROE_Y == 0) || (SW_ROE_Z == 0) )	// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=RUN_ROTARYENCORDER_PUSH;
		else if(SW_ETCMACHINE_RUN == 0)										// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_ETCRUN_PUSH;
		else if(SW_PUSHPULL == 0)											// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_PUSHPULL_PUSH;
		else if(SW_CUT_AIRCYLINDER_ON == 0)										// [PUSH BT] 0: ON, 1: OFF
			g_manual_status=INIT_CUTFABLICRUN_PUSH;
		else if(SW_LOADER_UPDOWN==0)	
			g_manual_status=INIT_LOADER_UPDOWN;


		if(SW_CLAMP3_OPEN_CLOSE==0)											// [SWITCH] 0: ON, 1: OFF
		 	exeOutportSignal(band_clamp_3_on,RESET);
		else
			exeOutportSignal(band_clamp_3_on,SET);
		if(SW_CLAMP4_OPEN_CLOSE==0)											// [SWITCH] 0: ON, 1: OFF
		 	exeOutportSignal(band_clamp_4_on,RESET);
		else
			exeOutportSignal(band_clamp_4_on,SET);
		if(SW_VACCUM_OPEN_CLOSE==0)											// [SWITCH] 0: ON, 1: OFF
		 	exeOutportSignal(vaccum_on,RESET);
		else
			exeOutportSignal(vaccum_on,SET);

		
		if(SW_ETCMACHINE_JOG == 0)
		{
			exeOutportSignal(etc_machine_on,SET);
			exeOutportSignal(etc_machine_decrease,SET);
		}
		else
		{
			exeOutportSignal(etc_machine_on,RESET);
			exeOutportSignal(etc_machine_decrease,RESET);
		}

		g_sewing_err=CHECK_READY;			//메뉴얼모드에서는  err 초기화(에러 발견시  통신하고 초기화)
											// [PUSH BT] 0: ON, 1: OFF
	}
	else
	{
		g_lamp_mode=YELLOW_RUN;				//메뉴얼버튼 off시 램프 대기
	}

	
	if(SW_MANUAL_STOP==0)					//[PUSH BT] 0: ON, 1: OFF
		stopManual();
	switch(g_manual_status)
	{
		case INIT_NEEDLE_PUSH:
			initNeedle();
			break;
		case INIT_LOOFER_PUSH:
			initLooper();
			break;
		case INIT_LIFTING_PUSH:
			initLifting();
			break;
		case INIT_MOVING_PUSH:
			initMoving(g_moving_speed);
			break;
		case INIT_TEST_PUSH:
			exeTestSewing(g_test_sewing_length,g_test_sewing_speed);		
			break;
		case RUN_ROTARYENCORDER_PUSH:
			exeRotaryEncorder();
			break;
		case INIT_ETCRUN_PUSH:
			exeEtcMachine(etc_machine_on,SET);
			break;
		case INIT_PUSHPULL_PUSH:
			exeLoaderPushPull();
			break;
		case INIT_CUTFABLICRUN_PUSH:
			exeCuttingFablic();
			break;
		case INIT_LOADER_UPDOWN:
			exeLoaderUpdown();
			break;
		default : 
			break;
	}							
}

void autoMode(void)
{
	if(g_auto_status==AUTO_READY||g_auto_wait!=OFF)			//최초 AUTO_READY상태에서 사용자가 푸쉬버튼 누르면(g_auto_status==AUTO_READY) g_auto_status=AUTO_SEWING_PUSH로 오토스윙시작==>오토스윙끝나면 g_auto_status=AUTO_ETCMACHINE_PUSH==>무창기계 종료되면 status=AUTO_SEWING_PUSH
	{														//정지후 자동시작 버튼을 누르면(g_auto_wait!=OFF) 오토스윙에서 재시작할경우...무창기계에서 재시작할경우로 분기...
		g_lamp_mode=GREEN_READY;							//오토버튼 off시 램프 대기
		if(SW_AUTO_START == 0)								// [PUSH BT] 0: ON, 1: OFF
		{
			if(g_auto_wait==OFF)							//정상적으로 SW_AUTO_START를 ON할 경우...
			{
				g_auto_status=AUTO_SEWING_PUSH;				//오토상태를 시작으로....
				g_autosewing_status=AUTO_INIT;				//오토스윙 상태플 초기화
				g_etc_machine_status=ETC_MACHINE_READY;		//무창기계 상태를 초기화
				g_sewing_servor_status=OFF;					//니들,루퍼,무빙 한셋의 상태(오토스윙시 한꺼번에 동작하므로 독립적인 상태로 관리)
				g_needle_puls_count=0;	
				g_move_count=0;	
				g_lenth_encode_count=0;															
			}
			else if(g_auto_wait==SWEING_WAIT)								//오토스윙에서 스탑후 재시작할 경우...
			{
				g_needle_puls_count=g_prev_needle_puls_count; 				//이전까지 봉제진행한  g_needle_puls_count값을 입력
				g_sewing_servor_status=OFF;
				g_auto_status=AUTO_SEWING_PUSH;
			}
			else											//ETCMACHINE_WAIT 무창기계 동장에서 스탑후 재시작할 경우.....g_auto_wait==OFF상태이므로 checkSweingLength()를 계속 수행하여 길이가 되면 무창기계 on
			{												
				g_auto_status=AUTO_ETCMACHINE_PUSH;			//g_auto_wait가 OFF상태면 자동으로 무창기계 관련 checkSweingLength()함수 수행 해당 루틴 없어도 무방....
			}
			g_sewing_err=CHECK_READY;						//오토버튼 클릭시 에러 초기화
			g_auto_wait=OFF;
		}
	}
	else
		g_lamp_mode=YELLOW_RUN;								//오토버튼 ON시 램프 런

	
	if(SW_AUTO_STOP == 0)									// [PUSH BT] 0: ON, 1: OFF
		stopAutoSewing();
	else if(g_auto_status==AUTO_SEWING_PUSH)
		autoSewing();
	
	checkSweingLength();									//동작과 별개로 무장기계 제어.....
}

void checkSweingLength(void)
{
	if(g_sewing_err)				
	{
		printf("checkSweingLength error code : %d\n",g_sewing_err);
		exeEtcMachine(etc_machine_on,RESET);
		exeOutportSignal(fablic_heating_on,SET);
		exeOutportSignal(cut_cylinder_on,SET);
		exeOutportSignal(loader_updown_on,SET);
		exeOutportSignal(loader_pushpull_on,SET);	
		return;
		
	}
	if(g_auto_wait!=OFF)					//스탑버튼이 눌릴경우 return....
		return;
	//printf(" %d \n",g_lenth_encode_count);
														
	if(g_prev_encode_count_step!=g_encode_count_step&&g_encode_count_step==ENCODE_COUNT_END_POS)							//g_auto_sewing_length까지 봉제 완료한 경우
	{
		printf("step 3 ON  %d \n",g_lenth_encode_count);
		g_etc_machine_status=ETC_MACHINE_FINISH_RUN;
	}
	else if(g_prev_encode_count_step!=g_encode_count_step&&g_encode_count_step==ENCODE_COUNT_INSERT_POS)					//원단절단후 달기밴드 길이까지 봉제완료 한경우
	{
		printf("step 2 ON   %d \n",g_lenth_encode_count);
		g_etc_machine_status=ETC_MACHINE_STOP1;
	}
	else if(g_prev_encode_count_step!=g_encode_count_step&&g_encode_count_step==ENCODE_COUNT_CUT_POS)						//원단절단 포지션까지 이동했을 경우
	{
		printf("step 1 ON   %d \n",g_lenth_encode_count);
		g_etc_machine_status=ETC_MACHINE_STOP;
	}
	g_prev_encode_count_step=g_encode_count_step;

	switch (g_etc_machine_status)
	{
		case ETC_MACHINE_RUN:												//무창기계 런:컷팅포지션까지 가기위해
			exeEtcMachine(etc_machine_on,SET);	
			printf("ETC_MACHINE_RUN %d\n",g_lenth_encode_count);
			break;
		case ETC_MACHINE_STOP:												//무창기계 스탑:컷팅위치까지 도착
			exeEtcMachine(etc_machine_on,RESET);	
			printf("ETC_MACHINE_STOP %d\n",g_lenth_encode_count);
			g_etc_machine_status = ETC_LOADER_DOWN;
			break;
		case ETC_LOADER_DOWN:												//로우더머신 다운:절단할 원단 고정하기 위해
			printf("ETC_LOADER_DOWN\n");
			exeOutportSignal(loader_updown_on,SET);
			exeDelay(2000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_etc_machine_status = ETC_MACHINE_CUTING_RUN;
			break;
		case ETC_MACHINE_CUTING_RUN:										//원단절단
			printf("ETC_MACHINE_CUTING_RUN\n");
			exeCuttingFablic();
			if(g_fablic_cut_status==OFF)	//[SENSOR] 0: ON, 1: OFF
				g_etc_machine_status = ETC_MACHINE_RUN1;
			else 
				;
			break;
		case ETC_MACHINE_RUN1:												//무창기계 런:달기로프 재봉을 위해
			exeEtcMachine(etc_machine_on,SET);	
			printf("ETC_MACHINE_RUN1 %d\n",g_lenth_encode_count);
			break;
		case ETC_MACHINE_STOP1:												//무창기계 스탑:달기로프 봉제 완료==>정지시 최상단 센서 위치에 정지하도록....
			printf("ETC_MACHINE_STOP1 %d\n",g_lenth_encode_count);
			topStopEtcMachine();	
			if(g_topstop_flag==true)
			{
				g_etc_machine_status = ETC_LOADER_PUSH;
				g_topstop_flag=false;
			}
			break;
		case ETC_LOADER_PUSH:												//로우더머신 푸쉬:절단후 대기중인 원단 무창기계 안단까지 밀어넣기
			printf("ETC_LOADER_PUSH\n");
			exeOutportSignal(loader_pushpull_on,SET);
			exeDelay(3500,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_etc_machine_status = ETC_MACHINE_SLOWRUN;
			break;
		case ETC_MACHINE_SLOWRUN:											//조깅런:밀어넣은 원단을 조깅으로 봉제
			printf("ETC_MACHINE_SLOWRUN\n");
			slowRunEtcMachine(2);	
			if(g_topstop_flag==true)
			{
				g_etc_machine_status = ETC_LOADER_PULL;
				g_topstop_flag=false;
			}
			break;
		case ETC_LOADER_PULL:												//로우더머신 풀:로우더 머신 원위치
			exeOutportSignal(loader_pushpull_on,RESET);
			printf("ETC_LOADER_PULL\n");
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_etc_machine_status = ETC_LOADER_UP;
			break;
		case ETC_LOADER_UP:												    //로우더머신 업:로우더 머신 원위치
			exeOutportSignal(loader_updown_on,RESET);
			printf("ETC_LOADER_UP\n");
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_etc_machine_status = ETC_MACHINE_RUN2;
			break;
		case ETC_MACHINE_RUN2:												//무창기계 런:세로봉제 완료를 위해
			printf("ETC_MACHINE_RUN2 %d\n",g_lenth_encode_count);
			exeEtcMachine(etc_machine_on,SET);
			break;
		case ETC_MACHINE_FINISH_RUN:										//무창기계 스탑:새로봉제 완료==>오토스윙시작   
			printf("ETC_MACHINE_FINISH_RUN     %d\n",g_lenth_encode_count);
			exeEtcMachine(etc_machine_on,RESET);
			g_lenth_encode_count=0;
			g_etc_machine_status=ETC_MACHINE_READY;
			startAutoSewing();
			break;
	}
	
}

void topStopEtcMachine(void)
{
	exeOutportSignal(etc_machine_decrease,SET);
	if(SEN_ETCMACHINE_HOME== 1&&g_prev_etcmachine_home!=SEN_ETCMACHINE_HOME)	//최상단 센서 ON시 정지  [SENSOR] 							0: ON, 1: OFF
		g_etc_slow_count++;
	if( g_etc_slow_count > 1 )													//최상단 센서 ON시 정지
	{
		if(SEN_ETCMACHINE_HOME == 1)											//최상단 센서 ON시 정지  [SENSOR] 							1: ON,    0:OFF
		{
			printf("SEN_ETCMACHINE_HOME\n");
			exeEtcMachine(etc_machine_on,RESET);								//무창기계 RUN off
			exeOutportSignal(etc_machine_decrease,RESET);						//무창기계 Decrease off
			g_topstop_flag=true;
			g_etc_slow_count=0;
		}
		else
		{
			g_topstop_flag=false;
		}
	}
	g_prev_etcmachine_home=SEN_ETCMACHINE_HOME;

	
}

void slowRunEtcMachine(uint8_t count)
{
	if(SEN_ETCMACHINE_HOME== 1&&g_prev_etcmachine_home!=SEN_ETCMACHINE_HOME)	//최상단 센서 ON시 정지  [SENSOR] 							0: ON, 1: OFF
		g_etc_slow_count++;
	if( g_etc_slow_count == count )												//최상단 센서 ON시 정지
	{
		exeEtcMachine(etc_machine_on,RESET);									//무창기계 RUN off
		exeOutportSignal(etc_machine_decrease,RESET);							//무창기계 Decrease off
		g_topstop_flag=true;
		g_etc_slow_count=0;
	}
	else
	{
		exeOutportSignal(etc_machine_on,SET);
		exeOutportSignal(etc_machine_decrease,SET);
		g_topstop_flag=false;
	}
	g_prev_etcmachine_home=SEN_ETCMACHINE_HOME;
}


void exeOutportSignal(unsigned char outportName, unsigned char Mode)
{
	outportSignal(outportName,Mode);
}


void startAutoSewing(void)
{
	g_lamp_mode=YELLOW_RUN;			
	g_autosewing_status=AUTO_INIT;
	g_needle_puls_count=0;		
	g_auto_status=AUTO_SEWING_PUSH;
	g_sewing_servor_status=OFF;
	g_sewing_err=CHECK_READY;		//오토버튼 클릭시 에러 초기화
	g_auto_wait=OFF;
	autoSewing();
}

void autoSewing(void)
{
	if(g_sewing_err)				
	{
		g_lamp_mode=RED_ERROR;//머신에러상태
		g_auto_status=AUTO_READY;

		printf("autoSewing error code : %d\n",g_sewing_err);
		//return;
	}
	send2PCautoSewingStatus();			//PC에 현재 상태정보를 줌
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
			
			g_encode_count_step		=ENCODE_COUNT_READY;
			g_prev_encode_count_step=ENCODE_COUNT_READY;
			g_lenth_encode_count	=0;
			break;
		case AUTO_MOVING_HOME_CHECK:
			g_autosewing_status = AUTO_NEDDLE_HOME_CHECK;
			if(SEN_MOVING_HOME == 1)								//[SENSOR] 1: ON, 0: OFF
				g_autosewing_status = AUTO_NEDDLE_HOME_CHECK;
			else
				g_sewing_err = MOVING_INIT_FAIL;
			break;
		case AUTO_NEDDLE_HOME_CHECK:
			initNeedle();
			if(g_needle_servor_status==OFF&&SEN_NEEDLE_HOME == 0)	//[SENSOR] 0: ON, 1: OFF
				g_autosewing_status = AUTO_LOOPER_HOME_CHECK;
			else if(g_needle_servor_status==ON)
				g_sewing_err = checkRunTime(50000,g_autosewing_status,NEEDLE_INIT_FAIL);			//니들초기화 time error울 위해 셋팅  run=>time over=>g_sewing_err  on=>g_auto_status=AUTO_READY;
			else
				g_sewing_err = NEEDLE_INIT_FAIL;
			break;
		case AUTO_LOOPER_HOME_CHECK:
			initLooper();
			if(g_looper_servor_status==OFF&&SEN_LOOPER_HOME == 0)	//[SENSOR] 0: ON, 1: OFF
				g_autosewing_status = AUTO_LIFTING_HOME_CHECK;
			else if(g_looper_servor_status==ON)
				;
			else
				g_sewing_err = LOOPER_INIT_FAIL;
			break;
		case AUTO_LIFTING_HOME_CHECK:
			initLifting();
			if(g_lifting_servor_status==OFF&&SEN_LIFTING_HOME == 1)	//[SENSOR] 1: ON, 0: OFF
				g_autosewing_status = AUTO_BANDCLAMP_CLOSE_CHECK;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = LIFTING_INIT_FAIL;
			break;
		case AUTO_BANDCLAMP_CLOSE_CHECK:									//밴드 삽입센서가 둘다 온되었느지 체크...온시 밴드를 잡아줌
			if(SEN_BANDINSERT_CHECK1 == 1&&SEN_BANDINSERT_CHECK2 == 1)		//[SENSOR] 1: ON, 0: OFF    
			{
				exeOutportSignal(band_clamp_3_on,RESET);
				exeOutportSignal(band_clamp_4_on,RESET);
				g_autosewing_status = AUTO_LIFTUP_10MM_UP;
			}
			else
				g_sewing_err = BANDCLAMP_INIT_FAIL;
			break;
		case AUTO_LIFTUP_10MM_UP:
			exeLifting(25,ON_UP);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVE_100MM;
			else if(g_lifting_servor_status==ON_UP||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVE_100MM:
			exeMoving(g_starting_position,g_moving_speed,ON_RIGHT);
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_LIFTUP_10MM_DOWN;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_MOVE_100MM_FAIL;//time check
			break;
		case AUTO_LIFTUP_10MM_DOWN:
			exeLifting(25,ON_DOWN);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_SEWING_RUN;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_SEWING_RUN:
			exeAutoSewing();
			if(g_sewing_servor_status==OFF)
				g_autosewing_status = AUTO_SEWINGMOVDELAY;
			else if(g_sewing_servor_status==ON || g_sewing_servor_status==MOVE_END )
				;
			else
				g_sewing_err = AUTO_SEWING_RUN_FAIL;//time check
			break;		
		case AUTO_SEWINGMOVDELAY:															//exeSewingMoving() 원할한 동작을 위해 delay
			exeDelay(500,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_autosewing_status = AUTO_HEATING_ON;
			break;
		case AUTO_HEATING_ON:
			exeOutportSignal(heating_on,RESET);
			exeOutportSignal(fablic_heating_on,SET);
			g_autosewing_status = AUTO_SEWINGMOVE_100MM;
			break;
		case AUTO_SEWINGMOVE_100MM:
			exeSewingMoving(g_auto_sewingmove_length,FRQ_10KHz);
			if(g_sewingmoving_servor_status==OFF)	
				g_autosewing_status = AUTO_VOCCUM_ON;
			else if(g_sewingmoving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_SEWINGMOVE_100MM_FAIL;//time check
			break;
		case AUTO_VOCCUM_ON:
			exeOutportSignal(vaccum_on,RESET);
			exeOutportSignal(heating_on,RESET);												//정지후 해당 구역에서 재시작시 히팅온.......???
			g_autosewing_status = AUTO_RC_TOP;
			break;
		case AUTO_RC_TOP:
			exeRC(RC_INCREASE);
			g_autosewing_status = AUTO_CUTDELAY;
			break;
		case AUTO_CUTDELAY:																	//열절단 되는 밴드 두께를 고려하여 delay설정
			exeDelay(7000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_autosewing_status = AUTO_HEATING_OFF;
			break;
		case AUTO_HEATING_OFF:
			exeOutportSignal(heating_on,SET);
			g_autosewing_status = AUTO_MOVE_10MM;
			break;
		case AUTO_MOVE_10MM:
			exeMoving(50,FRQ_10KHz,ON_RIGHT);												//컷팅완료후 뒤로 이동한 후, RC모터 복귀
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_RC_HOME;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_MOVE_100MM_FAIL;//time check
			break;
		case AUTO_RC_HOME:
			exeRC(RC_DECREASE);
			g_autosewing_status = AUTO_BANDCLAMP_OPEN_CHECK;
			break;
		case AUTO_BANDCLAMP_OPEN_CHECK:
			exeOutportSignal(band_clamp_3_on,SET);
			exeOutportSignal(band_clamp_4_on,SET);
			g_autosewing_status = AUTO_BANDCLAMP_OPEN_DELAY;
			break;
		case AUTO_BANDCLAMP_OPEN_DELAY:															//band_clamp가 정상적으로 OPEN되었는지 체크하기위해 클램프 동작시간 딜레이를 줌.....
			exeDelay(800,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF && SEN_BANDCLAMP_CLOSE==1)
				g_autosewing_status = AUTO_LIFTUP_10MM_2;
			else if(g_timer_delay_on==OFF && SEN_BANDCLAMP_CLOSE==0)
				g_sewing_err = AUTO_RC_HOME_FAIL;//time check
			break;
		case AUTO_LIFTUP_10MM_2:
			exeLifting(50,ON_UP);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVE_BEFOREHOME;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVE_BEFOREHOME:
			g_etc_machine_status=ETC_MACHINE_RUN;												//g_etc_machine_status=ETC_MACHINE_RUN하여 무창기계 동작시작
			exeMoving(g_starting_position+g_auto_sewing_length+g_auto_sewingmove_length+50-80,FRQ_8KHz,ON_LEFT);								//HOME의 80전까지
			exeOutportSignal(vaccum_on,RESET);															//정지 후, 재동작시 바큠 온....체크
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_LIFTUP_10MM_3;										//g_starting_position+g_auto_sewing_length+g_auto_sewingmove_length+50
			else if(g_moving_servor_status==ON_RIGHT||g_moving_servor_status==ON_LEFT)
				;
			else
				g_sewing_err = AUTO_MOVE_100MM_FAIL;//time check
			break;
		case AUTO_LIFTUP_10MM_3:
			exeLifting(50,ON_DOWN);
			if(g_lifting_servor_status==OFF)
				g_autosewing_status = AUTO_MOVING_HOME_CHECK2;
			else if(g_lifting_servor_status==ON_UP ||g_lifting_servor_status==ON_DOWN)
				;
			else
				g_sewing_err = AUTO_LIFTUP_10MM_FAIL;//time check
			break;
		case AUTO_MOVING_HOME_CHECK2:
			initMoving(FRQ_2KHz);
			exeOutportSignal(vaccum_on,RESET);															//정지 후, 재동작시 바큠 온....체크
			if(g_moving_servor_status==OFF)
				g_autosewing_status = AUTO_BANDCLAMP_CLOSE_CHECK_2;
			else if(g_moving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_MOVING_HOME_CHECK2_FAIL;
			break;
		case AUTO_BANDCLAMP_CLOSE_CHECK_2:
			if( (SEN_BANDINSERT_CHECK1 == 1&&SEN_BANDINSERT_CHECK2 == 1) )	
			{
				//exeOutportSignal(band_clamp_3_on,RESET);
				//exeOutportSignal(band_clamp_4_on,RESET);
				g_autosewing_status = AUTO_VOCCUM_OFF;
			}
			else
			{
				//g_sewing_err = BANDCLAMP_INIT_FAIL;		//???????????에러가 나더라도 일단 다음 동작 수행....
				//exeOutportSignal(band_clamp_3_on,RESET);
				//exeOutportSignal(band_clamp_4_on,RESET);
				g_autosewing_status = AUTO_VOCCUM_OFF;
			}
			break;
		case AUTO_VOCCUM_OFF:
			exeOutportSignal(vaccum_on,SET);
			//g_auto_status = AUTO_READY;
			g_auto_status=AUTO_ETCMACHINE_PUSH;													//AUTO_MOVE_BEFOREHOME에서 AUTO_ETCMACHINE_PUSH상태이지만 AUTO_SEWING_PUSH상태를 유지해야지만 마지막동작까지 완료
			break;
	}
}


void emergnecyMode(void)
{

}

void stopManual(void)
{
	g_timer_delay_on=OFF;				//delay는 무조건 초기화.....스탑시 exeDelay()함수 구분하지 않고 플래그값을 초기화시켜버림
	g_timer_delay_count=0;
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
			break;
		case INIT_MOVING_PUSH:
			servorStop(TIM_CHANNEL_3);	
			g_moving_servor_status=OFF;
			g_move_count=0;
			break;
		case INIT_TEST_PUSH:
			g_needle_puls_target_count=(g_needle_puls_count/SEW_1CYCLE_PULSE+1)*SEW_1CYCLE_PULSE;
			break;
		case INIT_ETCRUN_PUSH:
			exeEtcMachine(etc_machine_on,RESET);		
			break;
		case INIT_PUSHPULL_PUSH:
			exeOutportSignal(loader_updown_on,SET);
			exeOutportSignal(loader_pushpull_on,SET);
			break;
		case INIT_CUTFABLICRUN_PUSH:
			exeOutportSignal(fablic_heating_on,RESET);
			exeOutportSignal(cut_cylinder_on,SET);
			g_fablic_cut_status=OFF;
			break;
		default : 
			break;
	}
	g_manual_status=MANUAL_READY;
}


void stopAutoSewing(void)
{
	if(g_auto_wait!=OFF)				//Do it for the first time....오토모드의 오토스윙상태에서 정지버튼을 클릭시  한번만 동작(여러번 반복적으로 호출되면 g_prev_move_total_count이 g_move_target_count됨)
		return;

	g_timer_delay_on=OFF;				//delay는 무조건 초기화.....스탑시 exeDelay()함수 구분하지 않고 플래그값을 초기화시켜버림
	g_timer_delay_count=0;
	switch (g_etc_machine_status)		//먼저 무창기계에 대한 stop수행(오토스윙과 무창기계가 같이 수행되는 구간이 있음)		//다믕 autosweing에 대한 stop수행
	{
		case ETC_MACHINE_RUN:
			exeEtcMachine(etc_machine_on,RESET);
			g_auto_wait=ETCMACHINE_WAIT;
			printf("stop 0\n");
			break;
		case ETC_MACHINE_CUTING_RUN: 				//컷팅전에 중지명령이 발생하면 다시 시작시 컷팅부터 다시 수행
			exeOutportSignal(fablic_heating_on,SET);
			exeOutportSignal(cut_cylinder_on,RESET);
			g_auto_wait=ETCMACHINE_WAIT;
			g_fablic_cut_status=OFF;
			printf("stop 1\n");
			break;
		case ETC_MACHINE_RUN1:						//달기밴드 제작시에 중지명령이 발생할경우 무창기계 스탑
			exeEtcMachine(etc_machine_on,RESET);
			g_auto_wait=ETCMACHINE_WAIT;
			printf("stop 2\n");
			break;
		case ETC_MACHINE_RUN2:
			exeEtcMachine(etc_machine_on,RESET);
			g_auto_wait=ETCMACHINE_WAIT;
			printf("stop 4\n");
			break;
		default : 
			g_auto_wait=ETCMACHINE_WAIT;
			break;
	}
	switch (g_autosewing_status)
	{
		case AUTO_MOVE_100MM:
			servorStop(TIM_CHANNEL_3);	
			g_moving_servor_status=OFF;
			
			g_auto_wait=SWEING_WAIT;
			g_auto_status=AUTO_READY;
			break;
		case AUTO_SEWING_RUN:
			g_prev_needle_puls_count=(g_needle_puls_count/SEW_1CYCLE_PULSE+1)*SEW_1CYCLE_PULSE;
			g_needle_puls_target_count=g_prev_needle_puls_count;

			g_auto_wait=SWEING_WAIT;
			g_auto_status=AUTO_READY;
			break;
		case AUTO_VOCCUM_ON:
		case AUTO_RC_TOP:
		case AUTO_CUTDELAY:
		case AUTO_HEATING_OFF:
		case AUTO_RC_HOME:
			exeOutportSignal(vaccum_on,SET);
			exeOutportSignal(heating_on,SET);
			exeRC(RC_DECREASE);
			g_autosewing_status=AUTO_VOCCUM_ON;

			g_auto_wait=SWEING_WAIT;
			g_auto_status=AUTO_READY;
			break;
		case AUTO_MOVE_BEFOREHOME:
			exeOutportSignal(vaccum_on,SET);
			servorStop(TIM_CHANNEL_3);	
			g_moving_servor_status=OFF;
			
			g_auto_wait=SWEING_WAIT;
			g_auto_status=AUTO_READY;
			break;
		case AUTO_MOVING_HOME_CHECK2:
			exeOutportSignal(vaccum_on,SET);
			servorStop(TIM_CHANNEL_3);	
			g_moving_servor_status=OFF;
			
			g_auto_wait=SWEING_WAIT;
			g_auto_status=AUTO_READY;
			break;
		default : 
			break;
	}
}

uint8_t checkRunTime(unsigned int checkTime, uint8_t machineStatus, uint8_t errCode )
{
	uint8_t resultErr=CHECK_READY;
	if(g_check_prevstatus!=machineStatus)
	{
		g_check_time_count=0;
		g_check_prevstatus=machineStatus;
	}
	else
	{
		if(checkTime==g_check_time_count)
		{
			resultErr=errCode;
			g_check_time_count=0;
			g_check_prevstatus=CHECK_READY;
		}
	}
	return resultErr;
}

void initNeedle(void)
{
	if(SEN_NEEDLE_HOME == 0 || g_sewing_err==NEEDLE_INIT_FAIL)			//[SENSOR] 0: ON, 1: OFF			오토모드에서의 time over에러 체크하여 에러시, needle서보 stop
	{
		servorStop(TIM_CHANNEL_1);	
		g_manual_status=MANUAL_READY;
		g_needle_servor_status=OFF;
	}
	else 
	{
		switch (g_needle_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_1,g_init_speed,ROTATE_TYPE_CW);
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

void initLooper(void)
{
	if(SEN_LOOPER_HOME == 0) 						//[SENSOR] 0: ON, 1: OFF
	{
		servorStop(TIM_CHANNEL_2);	
		g_manual_status=MANUAL_READY;
		g_looper_servor_status=OFF;
	}
	else // needle home sensor off
	{
		switch (g_looper_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_2,g_init_speed,ROTATE_TYPE_CW);
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

void initLifting(void)
{
	if( g_lifting_servor_status!=ON_UP && SEN_LIFTING_HOME == 1) 			//[SENSOR] 1: ON, 0: OFF
	{
		servorStop(TIM_CHANNEL_4);	
		g_manual_status=MANUAL_READY;
		g_lifting_servor_status=OFF;
		g_lift_count=0;
	}
	else // needle home sensor off
	{
		switch (g_lifting_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CW);
				g_lifting_servor_status = ON_UP;
				break;
			case ON_UP:
				if(SEN_LIFTING_UP_LIM == 1)									//[SENSOR] 1: ON, 0: OFF
				{
					servorStop(TIM_CHANNEL_4);	
					servorStart(TIM_CHANNEL_4,g_lifting_speed,ROTATE_TYPE_CCW);
					g_lifting_servor_status=ON_DOWN;
				}
				break;
			case ON_DOWN:
				if(SEN_LIFTING_DN_LIM == 1)									//[SENSOR] 1: ON, 0: OFF
					servorStop(TIM_CHANNEL_4);//error
				break;
			default : 
				break;
		}
	}
}

void initMoving(unsigned int Speed)
{
	
	if(SEN_MOVING_HOME == 1)												//[SENSOR] 1: ON, 0: OFF 
	{
		servorStop(TIM_CHANNEL_3);	
		g_manual_status=MANUAL_READY;
		g_moving_servor_status=OFF;
	}
	else 
	{
		switch (g_moving_servor_status)
		{
			case OFF:
				servorStart(TIM_CHANNEL_3,Speed,ROTATE_TYPE_CCW);//FRQ_200Hz g_moving_speed
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

void bandCutting(void)
{
	if(g_sewing_err)				
	{
		g_lamp_mode=RED_ERROR;//머신에러상태
		g_manual_status=MANUAL_READY;
		//return;
	}
	switch (g_test_status)
	{
		
		case AUTO_VOCCUM_ON:
			exeOutportSignal(vaccum_on,RESET);
			g_test_status = AUTO_HEATING_ON;		//g_test_status=AUTO_VOCCUM_ON;
			break;
		
		case AUTO_HEATING_ON:
			exeOutportSignal(heating_on,RESET);
			g_test_status = AUTO_SEWINGMOVDELAY;
			break;
		case AUTO_SEWINGMOVDELAY:
			exeDelay(5000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_test_status = AUTO_RC_TOP;
			break;
		case AUTO_RC_TOP:
			exeRC(RC_INCREASE);
			if(g_rc_servor_status==OFF)
				g_test_status = AUTO_CUTDELAY;
			else if(g_rc_servor_status==RC_INCREASE)
				;
			else
				g_sewing_err = AUTO_RC_TOP_FAIL;//time check
			break;
		case AUTO_CUTDELAY:
			exeDelay(9000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_test_status = AUTO_HEATING_OFF;
			break;
		case AUTO_HEATING_OFF:
			exeOutportSignal(heating_on,SET);
			g_test_status = AUTO_RC_HOME;
			break;
		case AUTO_RC_HOME:
			exeRC(RC_DECREASE);
			if(g_rc_servor_status==OFF)
				g_test_status = AUTO_VOCCUM_OFF;
			else if(g_rc_servor_status==RC_DECREASE)
				;
			else
				g_sewing_err = AUTO_RC_HOME_FAIL;//time check
			break;
		case AUTO_VOCCUM_OFF:
			exeOutportSignal(vaccum_on,SET);
			g_manual_status=MANUAL_READY;
			break;
	}
}

void testSweingMoving(void)
{
	if(g_sewing_err)				
	{
		g_lamp_mode=RED_ERROR;//머신에러상태
		g_manual_status=MANUAL_READY;
		//return;
	}
	switch (g_test_status)
	{
		case AUTO_SEWINGMOVE_100MM:
			exeSewingMoving(100,FRQ_15KHz);//FRQ_200Hz  g_moving_speed  FRQ_10KHz 160
			if(g_sewingmoving_servor_status==OFF)
				g_manual_status=MANUAL_READY;
			else if(g_sewingmoving_servor_status==ON)
				;
			else
				g_sewing_err = AUTO_SEWINGMOVE_100MM_FAIL;//time check
			break;
	}
}

void exeTestSewing(unsigned int sewingLength,unsigned int Speed)
{
	if(g_sewing_servor_status==OFF && (SEN_NEEDLE_HOME == 1 || SEN_LOOPER_HOME == 1)) 		//[SENSOR] 0: ON, 1: OFF
	{
		g_manual_status=MANUAL_READY;
		g_sewing_servor_status=OFF;
		g_lamp_mode=RED_ERROR;
		g_sewing_err = LOOPER_INIT_FAIL;
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
				g_needle_puls_target_count=SEW_1CYCLE_PULSE*(sewingLength/SEWING_TICK+1);     //SEW_1CYCLE_PULSE*sewingLength;//(unsigned long)((float)sewingLength/ONE_PULSE_MV);// g_test_sewing_length g_test_sewing_speed
				g_needle_puls_count=0;
				g_move_count=0;
				g_target_speed=Speed;
				servorStart(TIM_CHANNEL_1,g_target_speed,ROTATE_TYPE_CW);
				servorStart(TIM_CHANNEL_2,g_target_speed,ROTATE_TYPE_CW);
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
				g_sewing_servor_status = ON;
				break;
			case ON:
				//time over err
				break;
			default : 
				break;
		}	
	}
}

void exeJogSewing(unsigned int Speed)
{
	switch (g_sewing_servor_status)
	{
		case OFF:
			g_needle_puls_target_count=SEW_1CYCLE_PULSE*(3000/SEWING_TICK+1);     //SEW_1CYCLE_PULSE*sewingLength;//(unsigned long)((float)sewingLength/ONE_PULSE_MV);// g_test_sewing_length g_test_sewing_speed
			g_target_speed=Speed;
			servorStart(TIM_CHANNEL_1,g_target_speed,ROTATE_TYPE_CW);
			servorStart(TIM_CHANNEL_2,g_target_speed,ROTATE_TYPE_CW);
			if(g_move_count!=0)
				servorStart(TIM_CHANNEL_3,g_target_speed,ROTATE_TYPE_CW);
			else
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
			g_sewing_servor_status = ON;
			break;
		case ON:
			//time over err
			break;
		default : 
			break;
	}	
}

void exeLoaderPushPull(void)
{
	if(g_loader_status == LOADER_COMPLATE)												//[SENSOR] 1: ON, 0: OFF 
	{
		g_manual_status=MANUAL_READY;
		g_loader_status=LOADER_READY;
	}

	switch (g_loader_status)
	{
		case LOADER_READY:
			if(OUT_PORT_DATA[7].bit0==SET)//if(OUT_PORT_DATA[6].bit7==SET&&OUT_PORT_DATA[7].bit0==SET)
			{
				g_loader_direct=RESET;
				g_loader_status = LOADER_PULL;
			}
			else
			{
				g_loader_direct=SET;
				g_loader_status = LOADER_DOWN;
			}
			break;
		case LOADER_DOWN:												//로우더머신 다운:절단할 원단 고정하기 위해
			exeOutportSignal(loader_updown_on,g_loader_direct);
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_PUSH;
			break;
		case LOADER_PUSH:												//로우더머신 푸쉬:절단후 대기중인 원단 무창기계 안단까지 밀어넣기
			exeOutportSignal(loader_pushpull_on,g_loader_direct);
			exeDelay(1500,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_COMPLATE;
			break;
		case LOADER_PULL:												//로우더머신 푸쉬:절단후 대기중인 원단 무창기계 안단까지 밀어넣기
			exeOutportSignal(loader_pushpull_on,g_loader_direct);
			exeDelay(3000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_UP;
			break;
		case LOADER_UP:												//로우더머신 다운:절단할 원단 고정하기 위해
			exeOutportSignal(loader_updown_on,g_loader_direct);
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_COMPLATE;
			break;
	}

}

void exeLoaderUpdown(void)
{
	if(g_loader_status == LOADER_COMPLATE)												//[SENSOR] 1: ON, 0: OFF 
	{
		g_manual_status=MANUAL_READY;
		g_loader_status=LOADER_READY;
	}

	switch (g_loader_status)
	{
		case LOADER_READY:
			if(OUT_PORT_DATA[7].bit0==SET)
				g_loader_status = LOADER_DOWN;
			else
				g_loader_status = LOADER_UP;
			break;
		case LOADER_DOWN:												//로우더머신 다운:절단할 원단 고정하기 위해
			exeOutportSignal(loader_updown_on,RESET);
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_COMPLATE;
			break;
		case LOADER_UP:												//로우더머신 다운:절단할 원단 고정하기 위해
			exeOutportSignal(loader_updown_on,SET);
			exeDelay(1000,TIMECHECK_DELAY);
			if(g_timer_delay_on==OFF)
				g_loader_status = LOADER_COMPLATE;
			break;
	}
}




void exeCuttingFablic(void)
{
	if(g_fablic_cut_status==ETC_CUT_COMPLATE) 
	{
		g_fablic_cut_status = OFF;
	}
	else
	{
		switch (g_fablic_cut_status)
		{
			case OFF:									
				g_fablic_cut_status = ETC_MACHINE_HEATING_ON;
				break;
			case ETC_MACHINE_HEATING_ON:										//원단 절단용 히터 온
				exeOutportSignal(fablic_heating_on,SET);
				printf("ETC_MACHINE_HEATING_ON\n");
				g_fablic_cut_status = ETC_MACHINE_HEATTINGDELAY;
				break;
			case ETC_MACHINE_HEATTINGDELAY:
				exeDelay(4000,TIMECHECK_DELAY);
				if(g_timer_delay_on==OFF)
					g_fablic_cut_status = ETC_MACHINE_CUTTING_TOP;
				break;
			/*case ETC_MACHINE_CUTTING_TOP:
				printf("ETC_MACHINE_CUTTING_TOP\n");
				exeOutportSignal(cut_cylinder_on,RESET);
				if(SEN_WELDING_CYLINDER_CLOSE==1)
					g_fablic_cut_status = ETC_MACHINE_CUTDELAY;
				else
					;
				break;*/
			case ETC_MACHINE_CUTTING_TOP:
				printf("ETC_MACHINE_CUTTING_TOP\n");
				exeOutportSignal(cut_cylinder_on,RESET);
				exeDelay(3000,TIMECHECK_DELAY);
				if(g_timer_delay_on==OFF)
					g_fablic_cut_status = ETC_MACHINE_CUTDELAY;
				break;
			case ETC_MACHINE_CUTDELAY:
				exeDelay(500,TIMECHECK_DELAY);
				if(g_timer_delay_on==OFF)
					g_fablic_cut_status = ETC_MACHINE_HEATING_OFF;
				break;
			case ETC_MACHINE_HEATING_OFF:
				printf("ETC_MACHINE_HEATING_OFF\n");
				exeOutportSignal(fablic_heating_on,RESET);
				g_fablic_cut_status = ETC_MACHINE_CUTTING_HOME;
				break;
			case ETC_MACHINE_CUTTING_HOME:
				printf("cut_cylinder_on\n");
				exeOutportSignal(cut_cylinder_on,SET);
				g_fablic_cut_status=ETC_CUT_COMPLATE;
				if(g_manual_status==INIT_CUTFABLICRUN_PUSH)
					g_manual_status=MANUAL_READY;
				break;
		}
	}
}


void exeRotaryEncorder(void)
{
	//https://m.blog.naver.com/newdname/221176596423
	//https://www.youtube.com/watch?v=xqzWQgpqHmI
	if(g_prev_rotaryencorder_z_on==0 && SW_ROE_Z==1)//Z-push button rising    					// [PUSH BT] 0: ON, 1: OFF   버튼이 온에서 오프로 변경시 한번만....서보모터 정지...니들 루퍼 무빙
	{																							
		TIM4->CCR1 = 0;	
		TIM4->CCR2 = 0;	
		TIM4->CCR3 = 0;	
		g_sewing_servor_status=OFF;	
	}
	else if(g_prev_rotaryencorder_z_on==1 && SW_ROE_Z==0)									// [PUSH BT] 0: ON, 1: OFF   버튼이 오프에에서 온으로 변경시 한번만....로터리엔코더 원활한 사용을 위해서 g_sewing_servor_status==off
	{																												
		g_prev_rotaryencorder_z_on=0;
	}
	

	if( (SW_ROE_X == 1) && (SW_ROE_Y == 1) && (SW_ROE_Z == 1) )					// [PUSH BT] 0: ON, 1: OFF
	{
		g_manual_status=MANUAL_READY;
		g_rotaryencorder_count=0;
		g_prev_rotaryencorder_rorate=0;
		g_rotaryencorder_status=OFF;
		g_prev_rotaryencorder_z_on=1;//off IN_PORT_DATA[3].bit5

		TIM4->CCR3 = 0;	
		TIM4->CCR4 = 0;
		return;
	}
	
	switch (g_rotaryencorder_status)
	{
		case OFF:
			g_prev_rotaryencorder_rorate=FORWORD;
			if(SW_ROE_X == 0)									// [PUSH BT] 0: ON, 1: OFF
			{
				g_re_runtime = true;
				g_rotaryencorder_runtime=0;
				g_re_move_count=0;
				g_rotaryencorder_status=RIGHTLEFT_MOVE;
				TIM4->ARR = FRQ_10KHz;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0); 
				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
			}
			else if(SW_ROE_Y == 0)								// [PUSH BT] 0: ON, 1: OFF
			{
				g_lift_count=0;
				g_rotaryencorder_status=UPDOWN_MOVE;
				TIM4->ARR = FRQ_5KHz;
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0); 
				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
			}	
			else if(SW_ROE_Z == 0)								// [PUSH BT] 0: ON, 1: OFF
				g_rotaryencorder_status=JOG_MOVE;
			break;
		case RIGHTLEFT_MOVE:
			if(g_prev_rotaryencorder_rorate!=g_rotaryencorder_rorate)
			{
				g_rotaryencorder_count=0;	
				g_re_move_count=0;
			}
			//printf("[%d]:[%d]\n",g_move_count,g_rotaryencorder_count);
			DWT_Delay_us(10);
			if(g_re_move_count<(((float)g_rotaryencorder_count*1)/ONE_PULSE_MV))
			{
				if(g_rotaryencorder_rorate==FORWORD)
					SV3_DIR_CCW;
				else
					SV3_DIR_CW;
				g_prev_rotaryencorder_rorate=g_rotaryencorder_rorate;
				if(SEN_MOVING_HOME == 0 && g_rotaryencorder_rorate==BACKWORD)
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_15KHz+1)/2);
				else if(g_rotaryencorder_rorate==FORWORD)
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_15KHz+1)/2);
				else
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			}
			else
			{
				TIM4->CCR3 = 0;	
			}
			break;
			//로터리엔코더의 
			/*if(g_prev_rotaryencorder_count!=g_rotaryencorder_count)
			{
				g_rotaryencorder_runtime=0;
			}
			else
			{
				if(g_rotaryencorder_runtime>10)
				{
					TIM4->CCR3 = 0;	
					g_rotaryencorder_count=0;	
					g_re_move_count=0;	
					g_rotaryencorder_runtime=0;
				}
			}
			//if(g_re_move_count<(((float)g_rotaryencorder_count*1)/ONE_PULSE_MV))
			if(g_re_move_count<g_rotaryencorder_count*16)
			{
				if(g_rotaryencorder_rorate==FORWORD)
					SV3_DIR_CCW;
				else
					SV3_DIR_CW;
				if(SEN_MOVING_HOME == 0 && g_rotaryencorder_rorate==BACKWORD)
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_15KHz+1)/2);
				else if(g_rotaryencorder_rorate==FORWORD)
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_15KHz+1)/2);
				else
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			}
			else
			{
				TIM4->CCR3 = 0;	
				g_rotaryencorder_count=0;	
				g_re_move_count=0;
			}
			g_prev_rotaryencorder_count=g_rotaryencorder_count;
			

			break;*/
		case UPDOWN_MOVE:
			if(g_prev_rotaryencorder_rorate!=g_rotaryencorder_rorate)
			{
				g_rotaryencorder_count=0;	
				g_lift_count=0;
			}
			//printf("[%d]:[%d]\n",g_move_count,g_rotaryencorder_count);
			DWT_Delay_us(10);
			if(g_lift_count<g_rotaryencorder_count*16)
			{
				if(g_rotaryencorder_rorate==FORWORD)
					SV4_DIR_CCW;
				else
					SV4_DIR_CW;
				g_prev_rotaryencorder_rorate=g_rotaryencorder_rorate;
				if(SEN_LIFTING_UP_LIM == 0 && g_rotaryencorder_rorate==FORWORD)				// [SENSOR] 1: ON, 0: OFF
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_15KHz+1)/2);
				else if(SEN_LIFTING_DN_LIM == 0 && g_rotaryencorder_rorate==BACKWORD)		// [SENSOR] 1: ON, 0: OFF
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_15KHz+1)/2);
				else 
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
			}
			else
			{
				TIM4->CCR4 = 0;
			}
			break;
		case JOG_MOVE:
			g_prev_rotaryencorder_z_on=0;				//on   IN_PORT_DATA[3].bit5;
			exeJogSewing(FRQ_200Hz);
			break;
		default : 
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	
	if(GPIO_Pin==GPIO_PIN_0)			//needle servor 
	{
		if(g_sewing_servor_status == ON )				//테스트스윙이나 오토스윙의 경우
		{
			g_needle_puls_count++;
			if(g_needle_puls_count==g_needle_puls_target_count)
			{
				//g_needle_puls_count=0;
				g_sewing_servor_status=COMPLATE;

				TIM4->CCR1 = 0;	
				TIM4->CCR2 = 0;	
				TIM4->CCR3 = 0;	
			}
			else if((g_needle_puls_count+(SEW_1CYCLE_PULSE-SEW_OFF_FABRIC_PULSE))%SEW_1CYCLE_PULSE==0)				
				TIM4->CCR3 = (g_target_speed+1)/2;
		}
		else if(g_sewingmoving_servor_status == ON)		//exeSewingMoving()용......TIM4->CCR2 루퍼동작 안함,SEW_OFF_FABRIC_PULSE가 고정(1300)
		{
			g_needle_puls_count++;
			if(g_needle_puls_count==g_needle_puls_target_count)
			{
				g_needle_puls_count=0;
				g_sewingmoving_servor_status=COMPLATE;

				TIM4->CCR1 = 0;	
				TIM4->CCR3 = 0;	
			}
			else if((g_needle_puls_count+(SEW_1CYCLE_PULSE-SEWMOVING_OFF_FABRIC_PULSE))%SEW_1CYCLE_PULSE==0)	//exeSewingMoving()==>빈스윙을 이용하여 무빙	  SEW_1CYCLE_PULSE가  1300에서  무무빙을 시작
				TIM4->CCR3 = (g_target_speed+1)/2;
		}
	}

	/*if(GPIO_Pin==GPIO_PIN_1)			//looper servor 인터럽터 동작없음 
	{
	;
	}*/

	if(GPIO_Pin==GPIO_PIN_2)			//moving servor 
	{	
		if(g_autosewing_status==AUTO_MOVE_100MM||g_autosewing_status== AUTO_MOVE_BEFOREHOME||g_autosewing_status==AUTO_MOVE_10MM)
			g_move_count++;				
		else if(g_sewing_servor_status == ON)		//테스트스윙이나 오토스윙의 경우
		{
			g_move_count++;
			if(g_move_count == MOV_1CYCLE_PULSE)	
			{
				TIM4->CCR3 = 0;	
				g_move_count = 0;
			}
		}
		else if(g_sewingmoving_servor_status== ON)	//exeSewingMoving()용......MOV_1CYCLE_PULSE가 4mm으로 고정
		{
			g_move_count++;
			if(g_move_count == MOV_5MM_CLOCK)	    // exeSewingMoving()==>빈스윙을 이용하여 무빙   4mm 무빙
			{
				TIM4->CCR3 = 0;
				g_move_count = 0;
			}
		}
		else if(g_rotaryencorder_status==RIGHTLEFT_MOVE)	//로터리엔코더의 무빙과 조깅테스트가 충돌.....로터리엔코더용 무빙카운터 추가
			g_re_move_count++;								//로터리엔코더로 좌우 무빙후 로터리엔코더 조그모드 동작하면 g_move_count가 좌우 무빙후의 카운트로 셋팅==>오동작
	}

	if(GPIO_Pin==GPIO_PIN_3)//lifting servor  
		g_lift_count++;

	if(GPIO_Pin==GPIO_PIN_5)//RotaryEncorder
	{
		if(g_rotaryencorder_status==RIGHTLEFT_MOVE||g_rotaryencorder_status==UPDOWN_MOVE)
		{
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == 0) // check RotaryEncorder Rotate
				g_rotaryencorder_rorate = FORWORD;
			else	
				g_rotaryencorder_rorate = BACKWORD;
			g_rotaryencorder_count++; 
		}
	}

	if(GPIO_Pin==GPIO_PIN_15)                           //RotaryEncorder
	{
		if(g_etc_machine_run==ON)		                //인터럽터의 카운트와 메인의 카운트가 시간차이로 인해 메인에서 정확한 카운트 불가능==>인터럽트 안에서 정확한 카운트를 구하여 플래그값 변경을 통해 메인에서 동작 구현
		{
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == 0) // check RotaryEncorder Rotate
				g_lenth_encode_count++; 


			if(g_lenth_encode_count>=g_fablic_cut_position+g_attachband_length+(g_auto_vertical_length-g_fablic_cut_position))		//g_auto_vertical_length까지 봉제 완료한 경우	1200+300							
				g_encode_count_step=ENCODE_COUNT_END_POS; 																		//840+300+(1200-840)
			else if(g_lenth_encode_count>=g_fablic_cut_position+g_attachband_length)	//원단절단후 달기밴드 길이까지 봉제완료 한경우 				840+300   870
				g_encode_count_step=ENCODE_COUNT_INSERT_POS; 
			else if(g_lenth_encode_count>=g_fablic_cut_position)						//원단절단 포지션까지 이동했을 경우    		        840      
				g_encode_count_step=ENCODE_COUNT_CUT_POS; 
		}

		//g_attachband_length  		= 300;					//달기밴드 길이
		//g_auto_vertical_length		= 1200;					//무창기계 수직봉제길이
		//g_fablic_cut_position		= 840;					//원단절단 시작시점(길이)
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		if(g_timer_delay_on)			//히팅절단용 카운터 on==>오토스윙의 delay동작용
			g_timer_delay_count++;

		if(g_lamp_mode==RED_ERROR||g_lamp_mode==RED_COM_ERROR||g_lamp_mode==YELLOW_RUN_WAIT)		//Lamp RED 반전용 카운터 on
			g_error_count++;

		if(g_check_prevstatus)			//auto스윙의 개별적인 함수 시간을  체크하여   타임오버시 에러를   표시  ==>카운터 on
			g_check_time_count++;
		
		g_com_wakeup_count++;
		if(g_com_wakeup_count>=WAKWUP_MAX)		//PC와의 통신확인용.....2s
		{
			g_com_wakeup_count=0;
			g_cycle_send_flag=true;
			
		}

		if(g_re_runtime)
			g_rotaryencorder_runtime++;

	}
	
}

void sendCycle2PC(void)
{
	if(g_cycle_send_flag==true)
	{
		g_cycle_send_flag=false;
		send2PC(COMMAND_WAKEUP,0,0);
	}
}	

void receiveFromPC(void)
{
	unsigned int temp=0;
	unsigned char bError=false;
	unsigned char bComplate = false;
	if(readBufferCount != insertBufferCount)	//0 1
	{
		temp = readBuffer[readBufferCount++];

		if(readBufferCount == COM_BUF_MAX)	
			readBufferCount = 0;
		switch(DataCount)
		{
			case 0:
				if(temp!=COM_STX1)
					bError=true;
				break;
			case 1:
				if(temp!=COM_STX2)
					bError=true;
				break;
			case 2:
			case 3:
			case 4:
				break;
			case 5:
				if(temp!=COM_ETX1)
					bError=true;
				break;
			case 6:
				if(temp!=COM_ETX2)
					bError=true;
				else 
					bComplate=true;
				break;
			
		}

		if(bError==true)
			DataCount=0;
		else
			readData[DataCount++]=temp;


		if(bComplate==true)
		{
			printf("0x%2x  0x%2x  0x%2x  0x%2x  0x%2x   0x%2x  0x%2x\n",readData[0],readData[1],readData[2],readData[3],readData[4],readData[5],readData[6]);
			DataCount=0;
			unsigned char command=readData[2];//g_com_wakeup_flag
			switch(command)							
			{
				case COMMAND_LENGTH:				//PC에서 보낸 봉제길이
					temp = (unsigned int)readData[3];
					temp <<= 8;
					temp &= 0xff00;
					temp |= ((unsigned int)readData[4])&0x00ff;
					break;
				case COMMAND_SPEED:					//PC에서 보낸 속도
					break;
				case COMMAND_TICK:					//PC에서 보낸 땀띠
					break;
				case COMMAND_AUTOSEWING_STOP:		//비젼 센서에서 오류감지하여 정지 지령
					stopAutoSewing();
					break;
				case COMMAND_WAKEUP:				//PC Wake_up신호  최초 한번 점검 (최초 COMMAND_LENGTH,COMMAND_SPEED,COMMAND_TICK정보를 다 받아야지만 True)
					g_com_wakeup_flag=true;
					break;
				default:
					break;
			}
			g_receive_flag=false;
		}
	}
}

void runTowerLamp(void)										//g_sewing_err를 확인하여 PC에 전송
{
	if(g_prev_lamp_mode==RED_ERROR&&g_lamp_mode==GREEN_READY)		//RED_ERROR상태에서 GREEN_READY변경없음 RED_ERROR확인 후, 구동시켜서 YELLOW_RUN, 그리고 GREEN_READY
	{																//이전이 RED_ERROR, 현재 GREEN_READY라면 현재상태를 RED_ERROR로 변경
		g_lamp_mode=RED_ERROR;										//오토스윙시 에러발생시 g_lamp_mode==RED_ERROR가 되고 동작정지되면서 다시 g_lamp_mode==GREEN_READY가 되므로 에러를 표현할수 없음
	}
	else if(g_auto_wait!=OFF)
	{
		g_lamp_mode=YELLOW_RUN_WAIT;	
		
	}
	
	switch (g_lamp_mode)
	{
		case RED_ERROR:
			outportSignal(towerlamp_y_on,SET);			//[LAMP] 0:ON  1:OFF
			outportSignal(towerlamp_g_on,SET);			//[LAMP] 0:ON  1:OFF
			if(g_error_count>=RED_LAMP_FAST_TOGGLE)
			{
				g_error_count=0;
			    outportSignal(towerlamp_r_on, EXCLUSIVE);
				
			}
			break;
		case RED_COM_ERROR:
			outportSignal(towerlamp_y_on,SET);			//[LAMP] 0:ON  1:OFF
			outportSignal(towerlamp_g_on,SET);			//[LAMP] 0:ON  1:OFF
			if(g_error_count>=RED_LAMP_SLOW_TOGGLE)
			{
				g_error_count=0;
			    outportSignal(towerlamp_r_on, EXCLUSIVE);
				
			}
			break;
		case YELLOW_RUN:
			outportSignal(towerlamp_r_on,SET);
			outportSignal(towerlamp_g_on,SET);
			outportSignal(towerlamp_y_on, RESET);
			break;
		case YELLOW_RUN_WAIT:
			outportSignal(towerlamp_r_on,SET);			//[LAMP] 0:ON  1:OFF
			outportSignal(towerlamp_g_on,SET);			//[LAMP] 0:ON  1:OFF
			if(g_error_count>=RED_LAMP_FAST_TOGGLE)
			{
				g_error_count=0;
			    outportSignal(towerlamp_y_on, EXCLUSIVE);
			}
			break;
		case GREEN_READY:
			outportSignal(towerlamp_r_on,SET);
			outportSignal(towerlamp_y_on,SET);
			outportSignal(towerlamp_g_on, RESET);
			break;
		default : 
			break;
	}

	

	send2PCerrorStatus();
}

void send2PCerrorStatus(void)								//현재의 에러상태를 PC로 전송   STX1+STX2+COMMAND_ERR+g_lamp_mode+g_prev_sewing_err+ETX1+ETX2
{
	if( g_prev_sewing_err!=g_sewing_err || g_prev_lamp_mode!=g_lamp_mode )		//동일한 에러는 걸러냄 
		send2PC(COMMAND_ERR,g_lamp_mode,g_prev_sewing_err);
	g_prev_sewing_err=g_sewing_err;
	g_prev_lamp_mode=g_lamp_mode;
}

void send2PCautoSewingStatus(void)							//현재의 autosweing상태를 PC로 전송   STX1+STX2+COMMAND_ERR+g_lamp_mode+g_prev_sewing_err+ETX1+ETX2
{
	if(g_prev_autosewing_status!=g_autosewing_status)
		send2PC(COMMAND_AUTOSWING_STATUS,0,g_autosewing_status);
	g_prev_autosewing_status=g_autosewing_status;
}

void servorStart(uint32_t Channel,unsigned int Arr,unsigned char Rotate)
{
	HAL_TIM_PWM_Stop(&htim4,Channel);
	if(Channel==TIM_CHANNEL_1)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,Rotate);  //SV1_DIR_CCW; // needle ac servo ccw
	else if(Channel==TIM_CHANNEL_2)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,Rotate);
	else if(Channel==TIM_CHANNEL_3)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,Rotate);
	else if(Channel==TIM_CHANNEL_4)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,Rotate);

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

void exeEtcMachine(unsigned char outportName, unsigned char Mode)
{
	outportSignal(outportName,Mode);
	if(Mode==SET)
		g_etc_machine_run=ON;
	else
		g_etc_machine_run=OFF;
}



void exeLifting( int Move_length, unsigned char Rotate)
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
				g_lift_count=0;
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

void exeMoving( int Move_length,unsigned int Speed,unsigned char Rotate)
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
		switch (g_moving_servor_status)
		{
			case OFF:
				//g_move_count=0;
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
				if(g_auto_wait==SWEING_WAIT)
				{
					TIM4->CCR3 = (Speed+1)/2;
					g_auto_wait=OFF;
				}
				break;
			default : 
				break;
		}
	}
}

void exeAutoSewing(void)
{
	if(g_sewing_servor_status==COMPLATE) 
	{
		g_sewing_servor_status = OFF;
		g_needle_puls_count=0;
		
	}
	else
	{
		switch (g_sewing_servor_status)
		{
			case OFF:
				g_needle_puls_target_count=SEW_1CYCLE_PULSE*(g_auto_sewing_length/SEWING_TICK+1);
				g_move_count=0;
				g_target_speed=g_auto_sewing_speed;
				servorStart(TIM_CHANNEL_1,g_target_speed,ROTATE_TYPE_CW);
				servorStart(TIM_CHANNEL_2,g_target_speed,ROTATE_TYPE_CW);
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
				g_sewing_servor_status = ON;
				break;
			case ON:
				break;
			default : 
				break;
		}	
	}
}

void exeLooper(unsigned int Move_Pulse,unsigned char Rotate)
{
	g_move_target_count = Move_Pulse;
	if(g_looper_puls_count >= g_move_target_count)
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
				g_looper_puls_count=0;
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

void exeNeedle(unsigned int Move_Pulse,unsigned char Rotate)
{
	g_move_target_count = Move_Pulse;
	if(g_needle_puls_count >= g_move_target_count)
	{
		servorStop(TIM_CHANNEL_1);	
		g_needle_puls_count=0;
		g_needle_servor_status=OFF;
	}
	else 
	{
		
		switch (g_needle_servor_status)
		{
			case OFF:
				g_needle_puls_count=0;
				if(Rotate==FORWORD)
				{
					servorStart(TIM_CHANNEL_1,g_lifting_speed,ROTATE_TYPE_CCW);
					g_needle_servor_status = ON_FORWORD;
				}
				else
				{
					servorStart(TIM_CHANNEL_1,g_lifting_speed,ROTATE_TYPE_CW);
					g_needle_servor_status = ON_BACKWORD;
				}
				break;
			case ON_FORWORD:
			case ON_BACKWORD:
			default : 
				break;
		}
	}
}

void exeSewingMoving(unsigned int Move_length,unsigned int Speed)
{
	if(g_sewingmoving_servor_status==COMPLATE) 
	{
		g_sewingmoving_servor_status = OFF;
	}
	else
	{
		switch (g_sewingmoving_servor_status)
		{
			case OFF:
				
				g_needle_puls_target_count=SEW_1CYCLE_PULSE*(Move_length/SEWINGMOVING_TICK+1);
				g_move_count=0;
				servorStart(TIM_CHANNEL_1,Speed,ROTATE_TYPE_CW);
				servorStart(TIM_CHANNEL_3,0,ROTATE_TYPE_CW);
				g_sewingmoving_servor_status = ON;
				g_target_speed 				 = Speed;
				break;
			case ON:
				break;
			default : 
				break;
		}	
	}
}

void exeDelay(unsigned int Delay_time,unsigned char Mode)
{
	if(Mode==TIMECHECK_DELAY)
	{
		g_timer_delay_on=ON;
		if(g_timer_delay_count>=Delay_time)
		{
			g_timer_delay_count=0;
			g_timer_delay_on=OFF;
		}
	}
}

void exeRC(unsigned char Mode)
{
	if(Mode == RC_INCREASE)
	{
		g_rcDuty[0] = RC_DUTY_MAX-100;		//컷팅을 좀더 빨리시작==>앞쪽에서 컷팅
		//g_rcDuty[1] = RC_DUTY_MIN+100;		//컷팅을 좀더 빨리시작==>앞쪽에서 컷팅
		g_rcDuty[1] = RC_DUTY_MAX-100;
	}
	else
	{
		g_rcDuty[0] = RC_DUTY_MIN;
		g_rcDuty[1] = RC_DUTY_MIN;
		//g_rcDuty[1] = RC_DUTY_MAX;
	}
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,g_rcDuty[0]);  // duty set rc servo1
    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,g_rcDuty[1]);
	g_rc_servor_status = OFF;
}





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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
    DWT_Delay_Init();
    HAL_TIM_Base_Start_IT(&htim2);

    //putChar(&huart4,'%');
    DWT_Delay_us(3000000);

    initVariables();
	printf("System Start!!!!!!!!!!!!!!!!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		readSignalProcess();

		if(g_com_wakeup_flag||g_com_wakeup_flag==false)					//최초 PC와의 상호통신상태를 확인하여 g_com_wakeup_flag==true일 경우만 동작
		{
			if(!checkPrevRunStatus())			//스위치변경시 초기화상태일 경우
			{
				if (SW_AUTO_MANUAL)				//[SWITCH] Mode select  1: MANUAL MODE, 0: AUTO MODE
					manualMode();
				else
					autoMode();
			}
			else								//스위치변경시 이전동작이 초기화상태가 아닐경우, 즉 동작이 진행중일경우 필요한 초기화처리
				errInitChangeSwitch();
		}
		else									//PC와의 상호통신상태를 주기적으로 확인하여 g_com_wakeup_flag==false일 경우 에러처리
		{
			g_lamp_mode=RED_COM_ERROR;				//통신 불능......
		}
		
		if(IN_PORT_DATA[0].bit0 == 1)//비상정이의 경우 미정의
			emergnecyMode();

		runTowerLamp();	
		receiveFromPC();
		sendCycle2PC();
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();


  /*PORTA -------------------------------------------------------------------------------------*/
  /*Configure GPIO pins : UART2_RW_Pin SYS_RUN_Pin EEP_WP_Pin */
  //GPIO_InitStruct.Pin = UART2_RW_Pin|SYS_RUN_Pin|EEP_WP_Pin;
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	

  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
  


  /*PORTB -------------------------------------------------------------------------------------*/
  /*Configure GPIO pins : OUT_CLK1_Pin OUT_CLK2_Pin OUT_CLK3_Pin OUT_CLK6_PinOUT_CLK4_Pin OUT_CLK5_Pin OUT_CLK7_Pin OUT_CLK8_Pin */
  //GPIO_InitStruct.Pin = OUT_CLK1_Pin|OUT_CLK2_Pin|OUT_CLK3_Pin|OUT_CLK6_Pin|OUT_CLK4_Pin|OUT_CLK5_Pin|OUT_CLK7_Pin|OUT_CLK8_Pin;
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROE_B_Pin */
  //GPIO_InitStruct.Pin = ROE_B_Pin;
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);



  /*PORTC -------------------------------------------------------------------------------------*/
  /*Configure GPIO pins : IN_CLK1_Pin IN_CLK2_Pin IN_CLK3_Pin IN_CLK4_PinIN_CLK5_Pin IN_CLK7_Pin IN_CLK8_Pin UART1_RW_Pin */
  //GPIO_InitStruct.Pin = IN_CLK1_Pin|IN_CLK2_Pin|IN_CLK3_Pin|IN_CLK4_Pin|IN_CLK5_Pin|IN_CLK7_Pin|IN_CLK8_Pin|UART1_RW_Pin;
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_CLK6_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC02 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOC, IN_CLK1_Pin|IN_CLK2_Pin|IN_CLK3_Pin|IN_CLK4_Pin|IN_CLK5_Pin|IN_CLK7_Pin|IN_CLK8_Pin|UART1_RW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);



   /*PORTD -------------------------------------------------------------------------------------*/
   /*Configure GPIO pins : SV_SIGN1_Pin SV_SIGN2_Pin SV_SIGN3_Pin SV_SIGN4_Pin */
   //GPIO_InitStruct.Pin = SV_SIGN1_Pin|SV_SIGN2_Pin|SV_SIGN3_Pin|SV_SIGN4_Pin;
   GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   /*Configure GPIO pins : ID_D0_Pin ID_D1_Pin ID_D2_Pin ID_D3_Pin */
   //GPIO_InitStruct.Pin = ID_D0_Pin|ID_D1_Pin|ID_D2_Pin|ID_D3_Pin;
   GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   /*Configure GPIO pin Output Level */
   //HAL_GPIO_WritePin(GPIOD, SV_SIGN1_Pin|SV_SIGN2_Pin|SV_SIGN3_Pin|SV_SIGN4_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);


   /*PORTE -------------------------------------------------------------------------------------*/
   /*Configure GPIO pins : IN2_Pin IN3_Pin IN4_Pin IN5_Pin IN6_Pin IN0_Pin IN1_Pin */
   //GPIO_InitStruct.Pin = IN2_Pin|IN3_Pin|IN4_Pin|IN5_Pin|IN6_Pin|IN0_Pin|IN1_Pin;
   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

   /*Configure GPIO pins : IN7_Pin OUT0_Pin OUT1_Pin OUT2_Pin OUT3_Pin OUT4_Pin OUT5_Pin OUT6_Pin OUT7_Pin */
   //GPIO_InitStruct.Pin = IN7_Pin|OUT0_Pin|OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin|OUT5_Pin|OUT6_Pin|OUT7_Pin;
   GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
   
   /*Configure GPIO pin Output Level */
   //HAL_GPIO_WritePin(GPIOE, IN7_Pin|OUT0_Pin|OUT1_Pin|OUT2_Pin|OUT3_Pin|OUT4_Pin|OUT5_Pin|OUT6_Pin|OUT7_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
  

  
  /*EXTERNAL TI -------------------------------------------------------------------------------*/
  /*Configure GPIO pin : ROE_A_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


   /*Configure GPIO pin : ROE_B_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  

  /*Configure GPIO pins : SV_PULSE1_IN_Pin SV_PULSE2_IN_Pin SV_PULSE3_IN_Pin SV_PULSE4_IN_Pin */
  //GPIO_InitStruct.Pin = SV_PULSE1_IN_Pin|SV_PULSE2_IN_Pin|SV_PULSE3_IN_Pin|SV_PULSE4_IN_Pin;
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  

  
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

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

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
