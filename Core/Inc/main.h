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
#define SYS_LED_TCNT                    499
unsigned int	sysLedCnt;

#define RC_DUTY_MIN                     1000
#define RC_DUTY_MAX                     1950
#define RC_DUTY_STEP                    1





/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern	ADC_HandleTypeDef hadc1;

extern	I2C_HandleTypeDef hi2c1;

extern	RTC_HandleTypeDef hrtc;


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

/* USER CODE BEGIN Private defines */
/* PORTA */
#define	LED_RUN_ON					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define	LED_RUN_OFF					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define	LED_RUN_TOG					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11)
/* PORT D */
#define	SV1_DIR_CW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET)
#define	SV1_DIR_CCW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET)
#define	SV2_DIR_CW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET)
#define	SV2_DIR_CCW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET)
#define	SV3_DIR_CW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET)
#define	SV3_DIR_CCW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET)	
#define	SV4_DIR_CW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET)
#define	SV4_DIR_CCW					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET)





struct PORT_DEF {
	union {
		struct {
			uint8_t bit0:1; // LSB
			uint8_t bit1:1;
			uint8_t bit2:1;
			uint8_t bit3:1;
			uint8_t bit4:1;
			uint8_t bit5:1;
			uint8_t bit6:1;
			uint8_t bit7:1; //MSB
		};
		uint8_t data;
	};
} IN_PORT_DATA[8];

struct OUT_PORT_DEF {
	union {
		struct {
			uint8_t bit0:1; // LSB
			uint8_t bit1:1;
			uint8_t bit2:1;
			uint8_t bit3:1;
			uint8_t bit4:1;
			uint8_t bit5:1;
			uint8_t bit6:1;
			uint8_t bit7:1; //MSB
		};
		uint8_t data;
	};
} OUT_PORT_DATA[8];

enum outport_name {
	// OUTPUT CH0
	needle_servo_on = 0,		
	needle_alram_reset,		//out_port1,
	out_port2,
	out_port3,
	out_port4,
	out_port5,
	out_port6,
	out_port7,
	// OUTPUT CH1
	looper_servo_on,		
	looper_alram_reset,
	out_port10,
	out_port11,
	out_port12,
	out_port13,
	out_port14,
	out_port15,
	// OUTPUT CH2
	moving_servo_on,		
	moving_alram_reset,		//out_port17,
	out_port18,
	out_port19,
	out_port20,
	out_port21,
	out_port22,
	out_port23,
	// OUTPUT CH3
	updown_servo_on,	
	updown_alram_reset,		// out_port25
	out_port26,
	out_port27,
	out_port28,
	out_port29,
	out_port30,
	out_port31,
	// OUTPUT CH4		
	out_port32,		
	out_port33,				//welding_cylinder_on
	out_port34,
	cut_cylinder_on,		//out_port35,
	out_port36,
	band_clamp_3_on, 		//out_port37
	out_port38,	
	band_clamp_4_on,		//out_port39
	// OUTPUT CH5
	out_port40,		
	vaccum_on, 				//out_port41,
	out_port42,
	heating_on,				//out_port43,
	out_port44,			
	out_port45,				//out_port45,
	out_port46,
	towerlamp_r_on,			//out_port47
	// OUTPUT CH6
	towerlamp_y_on,			//out_port48
	towerlamp_g_on,			//out_port49   
	out_port50,			
	out_port51,
	etc_machine_on,			//out_port52   21
	etc_machine_decrease,	//out_port53   22
	fablic_heating_on,		//out_port54   23
	loader_pushpull_on,		//out_port55   24   loader_updown_on
	// OUTPUT CH7
	loader_updown_on,		//out_port56   25	
	out_port57,		
	out_port58,		
	out_port59,
	out_port60,
	out_port61,
	out_port62,
	out_port63,
    num_of_out_port
};

#define SET   1
#define RESET 0
#define EXCLUSIVE 2


// -------------------------------------------------------------------------------------------------------------
// INPUT CH 0
#define SW_VACCUM_OPEN_CLOSE        IN_PORT_DATA[IN_CH0].bit5   // [SWITCH] 0: ON, 1: OFF




// INPUT CH 1
#define SEN_BANDINSERT_CHECK1       IN_PORT_DATA[IN_CH1].bit6   // [SWITCH] 1: ON, 0: OFF or another
#define SEN_BANDINSERT_CHECK2       IN_PORT_DATA[IN_CH1].bit7   // [SWITCH] 1: ON, 0: OFF or another

// INPUT CH 2
#define SEN_ETCMACHINE_HOME			IN_PORT_DATA[IN_CH2].bit5   // [SENSOR] 							0: ON, 1: OFF or another
#define SW_ROE_X                    IN_PORT_DATA[IN_CH2].bit6   // [SWITCH] 0: ON, 1: OFF
#define SW_ROE_Y                    IN_PORT_DATA[IN_CH2].bit7   // [SWITCH] 0: ON, 1: OFF

// INPUT CH 3
#define SW_ROE_Z                    IN_PORT_DATA[IN_CH3].bit5   // [SWITCH] 0: ON, 1: OFF
#define SW_ETCMACHINE_JOG           IN_PORT_DATA[IN_CH3].bit6   // [SWITCH] 0: ON, 1: OFF
#define SW_PUSHPULL                 IN_PORT_DATA[IN_CH3].bit7   // [SWITCH] 0: ON, 1: OFF

// INPUT CH 4
#define SEN_LIFTING_DN_LIM          IN_PORT_DATA[IN_CH4].bit0   // [SENSOR] LIFTING DOWN LIMITE         1: ON, 0: OFF
#define SEN_LIFTING_HOME            IN_PORT_DATA[IN_CH4].bit1   // [SENSOR] LIFTING HOME                1: ON, 0: OFF
#define SEN_BAND_EMPTY1             IN_PORT_DATA[IN_CH4].bit2   // [SENSOR] BAND1 EMPTY                 1: ON, 0: OFF
#define SEN_MOVING_HOME             IN_PORT_DATA[IN_CH4].bit3   // [SENSOR] MOVING HOME                 1: ON, 0: OFF
#define SEN_BAND_LOST1              IN_PORT_DATA[IN_CH4].bit4   // [SENSOR] BAND1 LOST                  1: ON, 0: OFF
#define SEN_BAND_LOST2              IN_PORT_DATA[IN_CH4].bit5   // [SENSOR] BAND2 LOST                  1: ON, 0: OFF
#define SEN_UPPER_THREAD_CUT_OFF    IN_PORT_DATA[IN_CH4].bit6   // [SENSOR] UPPER SEWING THREAD         1: CUT OFF DETECT, 0: NORMAL


// INPUT CH 5
#define SEN_LOOPER_HOME             IN_PORT_DATA[IN_CH5].bit0   // [SENSOR] LOOPER HOME                 0: ON, 1: OFF
#define SEN_WELDING_CYLINDER_CLOSE  IN_PORT_DATA[IN_CH5].bit4   // [SENSOR] LOOPER HOME                 0: CLOSE, 1: OPEN
#define SEN_BANDCLAMP_CLOSE         IN_PORT_DATA[IN_CH5].bit5   // [SENSOR] LOOPER HOME                 0: CLOSE, 1: OPEN
#define SEN_WELDING_CYLINDER_OPEN   IN_PORT_DATA[IN_CH5].bit6   // [SENSOR] LOOPER HOME                 0: CLOSE, 1: OPEN
#define SEN_BANDCLAMP_OPEN          IN_PORT_DATA[IN_CH5].bit7   // [SENSOR] LOOPER HOME                 0: CLOSE, 1: OPEN

// INPUT CH 6
#define SEN_NEEDLE_HOME             IN_PORT_DATA[IN_CH6].bit0   // [SENSOR] NEEDLE HOME                 0: ON, 1: OFF
#define SEN_LIFTING_UP_LIM          IN_PORT_DATA[IN_CH6].bit1   // [SENSOR] LIFTING UP LIMITE           1: ON, 0: OFF
#define SW_ETCMACHINE_RUN			IN_PORT_DATA[IN_CH6].bit2   // [PUSH BT] 0: ON, 1: OFF
#define SW_AUTO_MANUAL              IN_PORT_DATA[IN_CH6].bit3   // [SWITCH] Mode select  1: MANUAL MODE, 0: AUTO MODE                            ==>22
#define SW_AUTO_START               IN_PORT_DATA[IN_CH6].bit4   // [PUSH BT] 0: ON, 1: OFF
#define SW_AUTO_STOP                IN_PORT_DATA[IN_CH6].bit5   // [PUSH BT] 0: ON, 1: OFF
#define SW_MANUAL_STOP		 		IN_PORT_DATA[IN_CH6].bit6	// [PUSH BT] 0: ON, 1: OFF
#define SW_TEST_SEWING		 		IN_PORT_DATA[IN_CH6].bit7	// [PUSH BT] 0: ON, 1: OFF



// INPUT CH 7
#define SW_NEEDLE_HOME 				IN_PORT_DATA[IN_CH7].bit0	// [PUSH BT] 0: ON, 1: OFF
#define SW_LOOPER_HOME 				IN_PORT_DATA[IN_CH7].bit1	// [PUSH BT] 0: ON, 1: OFF
#define SW_MOVING_HOME 				IN_PORT_DATA[IN_CH7].bit2	// [PUSH BT] 0: ON, 1: OFF
#define SW_LIFTING_HOME 			IN_PORT_DATA[IN_CH7].bit3	// [PUSH BT] 0: ON, 1: OFF
#define SW_LOADER_UPDOWN	    	IN_PORT_DATA[IN_CH7].bit4   // [SWITCH] 0: ON, 1: OFF	
#define SW_CUT_AIRCYLINDER_ON	    IN_PORT_DATA[IN_CH7].bit5   // [SWITCH] 0: ON, 1: OFF
#define SW_CLAMP3_OPEN_CLOSE        IN_PORT_DATA[IN_CH7].bit6   // [SWITCH] 0: ON, 1: OFF
#define SW_CLAMP4_OPEN_CLOSE        IN_PORT_DATA[IN_CH7].bit7   // [SWITCH] 0: ON, 1: OFF





//jw0829 add
#define	ONE_PULSE_MV                (float)0.016421367
#define	ONE_PULSE_LF				(float)0.00625
#define	ONE_PULSE_NL                (float)0.007657632

#define	SEW_1CYCLE_PULSE            1600//1600
#define	SEW_OFF_FABRIC_PULSE        1350// 1400 short needle, 1200 long needle
#define	SEWMOVING_OFF_FABRIC_PULSE  1300

// AC SERVO
#define	FRQ_25KHz					39		// very fast
#define	FRQ_20KHz					49
#define	FRQ_15KHz					74				// narmal
#define	FRQ_10KHz					99	// 10KHz
#define	FRQ_8KHz					124
#define	FRQ_5KHz					199
#define	FRQ_4KHz					249
#define	FRQ_2KHz					499
#define	FRQ_1KHz					999
#define	FRQ_500Hz					1999
#define	FRQ_200Hz					4999			// low
#define	FRQ_100Hz					9999	// 100Hz
#define	FRQ_62Hz					15999	// 62Hz

#define	FRQ_RC_SV                   FRQ_62Hz

#define	MOV_1MM_CLOCK				60
#define	MOV_2MM_CLOCK				121
#define	MOV_3MM_CLOCK				182
#define	MOV_4MM_CLOCK				243
#define	MOV_5MM_CLOCK				304
#define	MOV_6MM_CLOCK				365
#define	MOV_7MM_CLOCK				426
#define	MOV_8MM_CLOCK				487
#define	MOV_9MM_CLOCK				548
#define	MOV_10MM_CLOCK				608

#define	MOV_100MM_CLOCK				6089 // 100.0061226
#define	MOV_150MM_CLOCK				9134 // 150.0091839
#define	MOV_200MM_CLOCK				12179 // 200.0122452
#define	MOV_250MM_CLOCK				15199 // 250.0153065
#define	MOV_300MM_CLOCK				18268 // 200.0122452
#define	MOV_350MM_CLOCK				21313 // 250.0153065
#define	MOV_400MM_CLOCK				24358 // 200.0122452
#define	MOV_450MM_CLOCK				27403 // 250.0153065
#define	MOV_500MM_CLOCK				30448 // 500.0141916
#define	MOV_1000MM_CLOCK			60896 // 1000.011962
#define	MOV_1200MM_CLOCK			73075 // 1200.007786
#define	MOV_1500MM_CLOCK			91344 // 1500.009732
#define	MOV_2000MM_CLOCK			121792 // 2000.007502
#define	MOV_2500MM_CLOCK			152240 // 2500.005273
#define	MOV_2600MM_CLOCK			158330 // 2600.011395
#define	MOV_2700MM_CLOCK			164419 // 2700.001096
#define	MOV_2800MM_CLOCK			170509 // 2800.007219

#define SEWING_TICK                 7
#define SEWINGMOVING_TICK           5

#define	MOV_1CYCLE_PULSE	    	MOV_7MM_CLOCK	// ddam length
#define	MOV_MAX_PULSE	    		MOV_1500MM_CLOCK	// total meterial length
#define MOV_ROE_MAX_PULSE           MOV_10MM_CLOCK // roe input run

#define INIT_SERVORSTOP_SPEED		FRQ_200Hz

#define FORWORD							0
#define BACKWORD						1

#define ROTATE_TYPE_FORWORD				1
#define ROTATE_TYPE_BACKWORD			2

#define UPDOWN_TYPE_UP					1
#define UPDOWN_TYPE_DOWN 				2

#define ROTATE_TYPE_CCW 				1
#define ROTATE_TYPE_CW 					0

#define VACCUM_ON 						1
#define VACCUM_OFF 						0

#define HEAT_ON 						1
#define HEAT_OFF 						0

#define OFF 							0
#define ON 								1
#define MOVE_END 						2
#define COMPLATE 						3
#define ON_DOWN 						2
#define ON_UP 							3
#define RC_INCREASE  					1
#define RC_DECREASE  					2
#define ON_FORWORD   					1
#define ON_BACKWORD  					2
#define ON_RIGHT  	 					1
#define ON_LEFT  	 					2
#define SWEING_WAIT       				1
#define ETCMACHINE_WAIT   				2

#define RIGHTLEFT_MOVE  				1
#define UPDOWN_MOVE  					2
#define JOG_MOVE  						3

#define TIMECHECK_DELAY 				1

#define MANUAL_READY					0
#define INIT_NEEDLE_PUSH 				1
#define INIT_LOOFER_PUSH 				2
#define INIT_MOVING_PUSH 				3
#define INIT_LIFTING_PUSH 				4
#define INIT_TEST_PUSH 					5
#define INIT_ETCRUN_PUSH				6
#define INIT_PUSHPULL_PUSH				7
#define INIT_CUTFABLICRUN_PUSH			8
#define RUN_ROTARYENCORDER_PUSH	 		9
#define INIT_LOADER_UPDOWN				10



#define AUTO_READY 						0
#define AUTO_SEWING_PUSH 				1
#define AUTO_ETCMACHINE_PUSH 			2

#define CHECK_READY 					0

#define AUTO_INIT 						0
#define AUTO_MOVING_HOME_CHECK			1
#define AUTO_NEDDLE_HOME_CHECK			2
#define AUTO_LOOPER_HOME_CHECK			3
#define AUTO_LIFTING_HOME_CHECK			4
#define AUTO_BANDCLAMP_CLOSE_CHECK		5
#define AUTO_LIFTUP_10MM_UP				6
#define AUTO_MOVE_100MM					7
#define AUTO_LIFTUP_10MM_DOWN			8
#define AUTO_SEWING_RUN					9
#define AUTO_SEWINGMOVDELAY				10
#define AUTO_HEATING_ON					11
#define AUTO_SEWINGMOVE_100MM			12
#define AUTO_VOCCUM_ON					13
#define AUTO_RC_TOP						14
#define AUTO_CUTDELAY					15
#define AUTO_HEATING_OFF				16
#define AUTO_MOVE_10MM					17
#define AUTO_RC_HOME					18
#define AUTO_BANDCLAMP_OPEN_CHECK		19
#define AUTO_BANDCLAMP_OPEN_DELAY		20
#define AUTO_LIFTUP_10MM_2				21
#define AUTO_MOVE_BEFOREHOME			22
#define AUTO_LIFTUP_10MM_3				23
#define AUTO_MOVING_HOME_CHECK2			24
#define AUTO_BANDCLAMP_CLOSE_CHECK_2	25
#define AUTO_VOCCUM_OFF					26

#define MOVING_INIT_FAIL				1
#define NEEDLE_INIT_FAIL				2
#define LOOPER_INIT_FAIL				3
#define LIFTING_INIT_FAIL				4
#define BANDCLAMP_INIT_FAIL				5
#define AUTO_LIFTUP_10MM_FAIL			6
#define AUTO_MOVE_100MM_FAIL			7
#define AUTO_SEWING_RUN_FAIL			8
#define AUTO_LOOPER_BACK_FAIL			9
#define AUTO_SEWINGMOVE_100MM_FAIL		10
#define AUTO_LOOPER_FRONT_FAIL			11
#define AUTO_RC_TOP_FAIL				12
#define AUTO_RC_HOME_FAIL				13
#define AUTO_MOVING_HOME_CHECK2_FAIL	14

#define ETC_MACHINE_READY 				0
#define ETC_MACHINE_RUN					1
#define ETC_MACHINE_STOP				2
#define ETC_LOADER_DOWN					3
#define ETC_MACHINE_CUTING_RUN          4
#define ETC_MACHINE_RUN1				5
#define ETC_MACHINE_STOP1				6
#define ETC_LOADER_PUSH					7
#define ETC_MACHINE_SLOWRUN				8
#define ETC_LOADER_PULL					9
#define ETC_LOADER_UP					10
#define ETC_MACHINE_RUN2				12
#define ETC_MACHINE_FINISH_RUN			13


#define ETC_MACHINE_HEATING_ON			1
#define ETC_MACHINE_HEATTINGDELAY		2
#define ETC_MACHINE_CUTTING_TOP			3
#define ETC_MACHINE_CUTDELAY			4
#define ETC_MACHINE_HEATING_OFF			5
#define ETC_MACHINE_CUTTING_HOME		6
#define ETC_CUT_COMPLATE				7

#define ETC_MACHINE_WELDING_DOWN		1
#define ETC_MACHINE_WELDINGDELAY		2
#define ETC_MACHINE_WELDING_HOME		3
#define ETC_WELDING_COMPLATE			4




#define RED_ERROR						0
#define YELLOW_RUN						1
#define GREEN_READY						2
#define RED_COM_ERROR					3
#define YELLOW_RUN_WAIT					4


#define	COM_STX1						0x02
#define	COM_STX2						0x0b
#define	COM_ETX1						0x0d
#define	COM_ETX2						0x0a

#define	COMMAND_IN						0x01
#define	COMMAND_OUT						0x02
#define	COMMAND_ERR						0x03
#define	COMMAND_AUTOSWING_STATUS		0x04
#define	COMMAND_WAKEUP					0x05
#define	COMMAND_LENGTH					0x06
#define	COMMAND_SPEED					0x07
#define	COMMAND_TICK					0x08
#define	COMMAND_AUTOSEWING_STOP			0x09

#define WAKWUP_MAX        				2000

#define	COM_BUF_MAX						50
#define	RECEIVE_DATA_MAX				7

#define RED_LAMP_FAST_TOGGLE			250
#define RED_LAMP_SLOW_TOGGLE			500

#define ENCODE_COUNT_READY 				0
#define ENCODE_COUNT_CUT_POS			1   
#define ENCODE_COUNT_INSERT_POS			2   
#define ENCODE_COUNT_END_POS			3	

#define LOADER_READY	 				0
#define LOADER_DOWN						1
#define LOADER_PUSH						2
#define LOADER_PULL						3
#define LOADER_UP						4
#define LOADER_COMPLATE					5










extern unsigned char	readBufferCount;			//PC????????? ?????? ???????????? ??? ??????
extern unsigned char	insertBufferCount;	
extern unsigned char	readBuffer[COM_BUF_MAX];
static unsigned char	readData[RECEIVE_DATA_MAX];
static unsigned char	DataCount;

static	unsigned int	g_auto_sewingmove_length;
static	unsigned int	g_auto_vertical_length;
static  unsigned int	g_starting_position;
static	unsigned int	g_auto_sewing_length;
static	unsigned int	g_test_sewing_length;
static	unsigned int	g_init_speed;
static	unsigned int	g_auto_sewing_speed;
static	unsigned int	g_test_sewing_speed;
static	unsigned int	g_lifting_speed;
static	unsigned int	g_moving_speed;
static	unsigned int	g_test_sewingspeed;
static  unsigned int 	g_target_speed;				//????????????????????? ??????????????? ???????????? HAL_GPIO_EXTI_Callback()?????? ???????????? ??????
static  uint16_t 		g_attachband_length;		//??????????????? ????????? ?????????
static  uint16_t 		g_fablic_cut_position;		//?????? ????????? ?????? ?????????

static	unsigned int	g_manual_status;			//manual-mode button/switch status
static	unsigned int	g_auto_status;  			//auto-mode button/switch status
static	unsigned int	g_autosewing_status;		//auto-mod operate status
static	unsigned int	g_auto_wait;				//???????????? ???????????? ????????? STOP          0:OFF   1:SWEING_WAIT   2:ETCMACHINE_WAIT
static	unsigned int	g_timer_delay_on;			//????????? ?????? ?????????

static	unsigned int	g_move_target_count;  		//interrupe plus count
static	unsigned int	g_lift_target_count;
static	unsigned int	g_needle_puls_count;
static	unsigned int	g_looper_puls_count;
static	unsigned int	g_move_count;
static	unsigned int	g_re_move_count;			//????????????????????? ???????????????
static	unsigned int	g_lift_count;
static	unsigned int	g_timer_delay_count;		//??????????????? ?????? ?????????
static  uint32_t  		g_needle_puls_target_count; //autosweing or testsweing??? ???????????????
static  uint32_t  		g_prev_needle_puls_count;   //autosweing or testsweing??? stop??? ???????????? ??????

static	unsigned int	g_sewing_servor_status; 	//servo moter operate status
static	unsigned int	g_needle_servor_status;
static	unsigned int	g_looper_servor_status;
static	unsigned int	g_moving_servor_status;
static	unsigned int	g_lifting_servor_status;
static	unsigned int	g_rc_servor_status;
static	unsigned int	g_rotaryencorder_status;
static	unsigned int 	g_sewingmoving_servor_status;
static  uint8_t 		g_check_prevstatus;			//g_check_prevstatus() ??????????????? ????????? ???????????? ????????????????????? ??????????????? ?????? 
static	unsigned int	g_prev_autosewing_status;	//send2PCautoSewingStatus()?????? ????????? ????????? ???????????? ?????? ??????...
static unsigned char	g_etc_machine_status;

static	unsigned int	g_rotaryencorder_count;			//?????????????????? ??????????????????
static	unsigned int	g_prev_rotaryencorder_count;	
static  uint8_t 		g_re_runtime;
static	unsigned int	g_rotaryencorder_runtime;


static	unsigned int	g_rotaryencorder_rorate;		//????????????????????? ????????????
static	unsigned int	g_prev_rotaryencorder_rorate;	//????????????????????? ??????????????????==>????????? ??????????????? ???????????? ??????
static	unsigned int	g_prev_rotaryencorder_z_on;		//????????????????????? z????????? ?????????==>?????? ????????? ???????????? ??????

static	unsigned int	g_error_count;				//??????????????? ??????????????? ?????? ?????????....RED_COM_ERROR:250(????????? ??????)         RED_ERROR:500(????????? ??????)     ?????? ??? ???????????? ??????
static  uint8_t 		g_lamp_mode;
static  uint8_t 		g_prev_lamp_mode;			//g_lamp_mode??? ???????????? ???????????? ???????????? ??????????????? ??????.....?????? RED_ERROR???????????? ??????GREEN_READY???????????? RED_ERROR?????? ???, ??????????????? YELLOW_RUN, ????????? GREEN_READY

static  unsigned int 	g_check_time_count;			//checkRunTime()?????? ??????
static  uint8_t 		g_prev_sw_auto_manual;		//checkPrevRunStatus() ?????????????????? ????????????????????? ????????????????????? ??????...
static  uint8_t 		g_change_sw_auto_manual;	//checkPrevRunStatus()  true:??????   false:?????????

static	uint8_t    		g_prev_sewing_err;			//PC???????????????.....????????? ????????? ??????
static	uint8_t    		g_sewing_err;				//??????????????????
static  unsigned int    g_rcDuty[2];				//rc ????????? ?????? MIN,MAX

static	uint8_t    		g_receive_flag;				//????????? ????????? ??? ??????
static	unsigned int 	g_com_wakeup_count;
static	uint8_t  		g_com_wakeup_flag;
static	uint8_t  		g_cycle_send_flag;

static	uint32_t  		g_lenth_encode_count;		//???????????? ????????? ?????? ?????????????????????
static	uint32_t  		g_prev_lenth_encode_count;	//wait??? ?????? ????????? ??????
static 	uint8_t  		g_etc_machine_run;			//???????????? ????????????...g_lenth_encode_count??? ??????(??????????????? ON??? ????????? g_lenth_encode_count??????)

static unsigned char	g_test_status;


static unsigned char	g_fablic_cut_status;
static unsigned char	g_loader_status;
static unsigned char	g_loader_direct;









static unsigned char	g_encode_count_step;		//??????????????? ???????????? ????????? ???????????? ??????????????? ?????? ???????????? ????????? ????????? ?????????==>???????????? ????????? ????????? ???????????? ????????? ???????????? ????????? ?????? ???????????? ?????? ??????
static unsigned char	g_prev_encode_count_step;

static 	uint8_t  		g_topstop_flag;

static 	uint8_t 		g_etc_slow_count;
static 	uint8_t 		g_prev_etcmachine_home;









void errInitChangeSwitch(void);
void send2PCerrorStatus(void);
void receiveFromPC(void);
void send2PCautoSewingStatus(void);
void sendCycle2PC(void);
void manualMode(void);


void exeLifting( int Move_length, unsigned char Rotate);
void exeMoving( int Move_length,unsigned int Speed,unsigned char Rotate);
void exeAutoSewing(void);
void exeLooper(unsigned int Move_Pulse, unsigned char Rotate);
void exeSewingMoving(unsigned int Move_length,unsigned int Speed);
void exeDelay(unsigned int Delay_time, unsigned char Mode);
void exeRC(unsigned char Mode);
void exeTestSewing(unsigned int sewingLength,unsigned int Speed);
void emergnecyMode(void);
void servorStart(uint32_t Channel,unsigned int Arr,unsigned char Rotate);
void servorStop(uint32_t Channel);
void initNeedle(void);
void initLooper(void);
void initLifting(void);
void initMoving(unsigned int Speed);
void bandCutting(void);
void testSweingMoving(void);
void exeNeedle(unsigned int Move_Pulse,unsigned char Rotate);
void stopManual(void);
void autoMode(void);
void stopAutoSewing(void);
void autoSewing(void);
void exeRotaryEncorder(void);
void initVariables(void);
void runTowerLamp(void);
uint8_t checkRunTime(unsigned int checkTime, uint8_t machineStatus, uint8_t errCode );
uint8_t checkPrevRunStatus(void);
void exeJogSewing(unsigned int Speed);
void startAutoSewing(void);
void checkSweingLength(void);
void exeEtcMachine(unsigned char outportName, unsigned char Mode);
void exeAircylinder(unsigned char outportName, unsigned char Mode);
void exeCuttingFablic(void);
void topStopEtcMachine(void);
void exeOutportSignal(unsigned char outportName, unsigned char Mode);
void slowRunEtcMachine(uint8_t count);
void exeLoaderPushPull(void);










/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
