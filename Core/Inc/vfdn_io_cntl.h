/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_sub.h
  * @brief          : Header for vfdn_sysInit.c file.
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
#ifndef __VFDN_IO_CNTL_H
#define __VFDN_IO_CNTL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
//#define	__DEBUG_IO__

#define	IN_CH_MAX						8

#define READ_US_DLY                     5

#define	READ_SIG_TCNT					2	// read signal timer counter per READ_SIGNAL_TCNT msec
#define	READ_SIG_TSTART					1
#define	READ_SIG_NOW					2

#if 0
#define	SW_SCAN_TCNT					10	// status change check timer counter SW_SCAN_TCNT msec for switch scan
#define	SW_SCAN_TSTART					1
#define	SW_SCAN_NOW						2
#endif

#define	MODE_AUTO						0x01
#define	MODE_MANUAL						0x02
#define	MODE_EMS						0x04
#define	MODE_MTEST						0x08

#define	IN_CH0							0
#define	IN_CH1							1
#define	IN_CH2							2
#define	IN_CH3							3
#define	IN_CH4							4
#define	IN_CH5							5
#define	IN_CH6							6
#define	IN_CH7							7

#define	OUT_CH1							0
#define	OUT_CH2							1
#define	OUT_CH3							2
#define	OUT_CH4							3
#define	OUT_CH5							4
#define	OUT_CH6							5
#define	OUT_CH7							6
#define	OUT_CH8							7

// SWITCH INPUT VARIABLE BIT DEFINE
#define	ST_SW_EMS						0x00000001	// PUSH LOCK SWITCH 1 : EMERGENCY STOP, 0 : NORMAL
#define	ST_SW_AUTO_MANUAL				0x00000002	// 2SELECT SWITCH1  1 : MANUAL, 0 : AUTO
#define	ST_SW_MOVING_FB					0x00000004	// 3SELECT SWITCH1  1 : MOVING FORWARD/BACKWARD
#define	ST_SW_MOVING_UD					0x00000008	// 3SELECT SWITCH1  1 : MOVING UP/DOWN
#define	ST_SW_AUTO_START				0x00000010	// PUSH BUTTON1     1 : AUTO MODE START
#define	ST_SW_AUTO_STOP					0x00000020	// PUSH BUTTON2     1 : AUTO MODE STOP
#define	ST_SW_AUTO_RESET				0x00000040	// PUSH BUTTON3     1 : AUTO MODE RESET - all position �ʱ�ȭ, 0 : NO ACTION
#define	ST_SW_TEST_SEWING				0x00000080	// PUSH BUTTON4     1 : TEST SEWING, 0 : NO ACTION
#define	ST_SW_NEEDLE_HOME				0x00000100	// PUSH BUTTON5     1 : UPPER SEWING MACHINE NEEDLE MOVING for HOME POSITION
#define	ST_SW_LOOPER_HOME				0x00000200	// PUSH BUTTON6     1 : UNDER SEWING MACHINE LOOPER MOVING for HOME POSITION
#define	ST_SW_MOVING_HOME				0x00000400	// PUSH BUTTON7     1 : MOVING FORWARD-REVERSE for HOME POSITION
#define	ST_SW_UD_HOME					0x00000800	// PUSH BUTTON8     1 : MOVING UP-DOWN for HOME POSITION
#define	ST_SW_CLAMP1_OPEN_CLOSE			0x00001000	// 2SELECT SWITCH2  1 : CLAMP1 CLOSE, 0 : CLAMP1 OPEN
#define	ST_SW_CLAMP2_OPEN_CLOSE			0x00002000	// 2SELECT SWITCH3  1 : CLAMP2 CLOSE, 0 : CLAMP2 OPEN
#define	ST_SW_CLAMP3_OPEN_CLOSE			0x00004000	// 2SELECT SWITCH4  1 : CLAMP3 CLOSE, 0 : CLAMP2 OPEN
#define	ST_SW_CLAMP4_OPEN_CLOSE			0x00008000	// 2SELECT SWITCH5  1 : CLAMP4 CLOSE, 0 : CLAMP2 OPEN
#define	ST_SW_VACCUM_OPEN_CLOSE			0x00010000	// 2SELECT SWITCH6  1 : VACCUM OPEN, 0 : VACCUM CLOSE
#define	ST_SW_BAND_CUT					0x00020000	// PUSH BUTTON9     1 : BAND CUT(with HEAT THREAD) 0 : NO ACTION
#define	ST_SW_ROE_X                     0x00040000	// 5SELECT SWITCH1  1 : ROE X(FB) 0 : off or another
#define	ST_SW_ROE_Y                     0x00080000	// 5SELECT SWITCH1  1 : ROE Y(UD) 0 : off or another
#define	ST_SW_ROE_Z                     0x00100000	// 5SELECT SWITCH1  1 : ROE Z(not specified) 0 : off or another
#define ST_SW_ROE_X1                    0x00200000  // 5SELECT SWITCH2  1 : 1pulse/mov len = 1mm 0 : off or another
#define ST_SW_ROE_X10                   0x00400000  // 5SELECT SWITCH2  1 : 1pulse/mov len = 10mm 0 : off or another

// SENSOR INPUT VARIABLE BIT DEFINE
#define	ST_SEN_LIFTING_UP_LIM			0x00000001	// SENSOR  1 : LIFTING UP LIMITE SENSOR ON, 0 : LIFTING UP LIMITE SENSRO OFF
#define	ST_SEN_LIFTING_DN_LIM			0x00000002	// SENSOR  1 : LIFTING DOWN LIMITE SENSOR ON, 0 : LIFTING DOWN LIMITE SENSOR OFF
#define	ST_SEN_LIFTING_HOME				0x00000004	// SENSOR  1 : LIFTING HOME SENSOR ON, 0 : LIFTING HOME SENSOR OFF
#define	ST_SEN_NEEDLE_HOME				0x00000008	// SENSOR  1 : NIDDLE HOME SENSOR ON, 0 : NEDDLE HOME SENSOR OFF
#define	ST_SEN_BAND_EMPTY1				0x00000010	// SENSOR  1 : BAND1 EMPTY SENSOR ON(������), 0 : BAND1 EMPTY SENSOR OFF(��峲��)
#define	ST_SEN_BAND_EMPTY2				0x00000020	// SENSOR  1 : BAND2 EMPTY SENSOR ON(������), 0 : BAND2 EMPTY SENSOR OFF(��峲��)
#define	ST_SEN_BAND_LOST1				0x00000040	// SENSOR  1 : BAND1 LOST SENSOR ON(��� cut off), 0 : BAND1 INSERT SENSOR OFF(���OK)
#define	ST_SEN_BAND_LOST2				0x00000080	// SENSOR  1 : BAND2 LOST SENSOR ON(��� cut off), 0 : BAND2 INSERT SENSOR OFF(���OK)
#define	ST_SEN_UPPER_THREAD_CUT_OFF		0x00000100	// SENSOR  1 : UPPER SEWING THREAD CUT OFF DETECT, 0 : UPPER SEWING THREAD NORMAL
#define	ST_SEN_LOOPER_HOME				0x00000200	// SENSOR  1 : LOOPER HOME SENSOR ON, 0 : LOOPER HOME SENSOR OFF
#define	ST_SEN_UNDER_THREAD_CUT_OFF		0x00000400	// SENSOR  1 : UNDER SEWING THREAD CUT OFF DETECT, 0 : UNDER SEWING THREAD NORMAL
#define	ST_SEN_BAND_INSERT1				0x00000800	// SENSOR  1 : BAND INSERT OK, 0 : NOT INSERT BAND
#define	ST_SEN_BAND_INSERT2				0x00001000	// SENSOR  1 : BAND INSERT OK, 0 : NOT INSERT BAND
#define	ST_SEN_MOVING_BACKWARD_LIM		0x00002000	// SENSOR  1 : MOVING BACKWARD LIMITE SENSOR ON, 0 : MOVING BACKWARD LIMITE SENSOR OFF
#define	ST_SEN_MOVING_FORWARD_LIM		0x00004000	// SENSOR  1 : MOVING FORWARD LIMITE SENSOR ON, 0 : MOVING FORWARD LIMITE SENSOR OFF
#define	ST_SEN_MOVING_HOME				0x00008000	// SENSOR  1 : MOVING HOME SENSOR ON, 0 : MOVING HOME SENSOR OFF
#define	ST_SEN_FABRIC_DETECT1			0x00010000	// SENSOR  1 : FABRIC NOT DETECT, 0 : FABRIC DETECT
#define	ST_SEN_FABRIC_DETECT2			0x00020000	// SENSOR  1 : FABRIC NOT DETECT, 0 : FABRIC DETECT
#define	ST_SEN_CLAMP12_OPEN				0x00040000	// SENSOR  CLAMP12_OPEN-1,CLAMP12_CLOSE-0 : OPEN
#define	ST_SEN_CLAMP12_CLOSE			0x00080000	// SENSOR  CLAMP12_OPEN-0,CLAMP12_CLOSE-1 : CLOSE
#define	ST_SEN_CLAMP34_OPEN				0x00100000	// SENSOR  CLAMP34_OPEN-1,CLAMP34_CLOSE-0 : OPEN
#define	ST_SEN_CLAMP34_CLOSE			0x00200000	// SENSOR  CLAMP34_OPEN-0,CLAMP34_CLOSE-1 : CLOSE

// AC SERVO INPUT VARIABLE BIT DEFINE
#define	ST_SV_ALRM						0x01		// SERVO ALARM
#define	ST_SV_HOME						0x02		// SERVO HOME
#define	ST_SV_RDY						0x04		// SERVO READY
#define	ST_ZERO_SPD						0x08		// SERVO ZERO SPEED
#define	ST_T_POS						0x10		// SERVO TERMINATE POSITION

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

// AC SERVO OUTPUT BIT DEFINE
#define	ST_SV_ON						0x01		// SERVO ON
#define	ST_ALRM_RST						0x02		// SERVO ALARM RESET
#define	ST_EMG_STOP						0x04		// SERVO EMERGENCY STOP
#define	ST_CNT_CLR						0x08		// SERVO COUNTER CLEAR
#define	ST_MODE_SEL0					0x10		// SERVO TORQUE COMMNAD LSB
#define	ST_MODE_SEL1					0x20		// SERVO TORQUE COMMNAD MSB
#define	ST_CW_LIM						0x40		// SERVO CLOCK WISE LIMITE(������ ȸ�� ���� limite)
#define	ST_CCW_LIM						0x80		// SERVO COUNTER CLOCK WISE LIMITE(������ ȸ�� ���� limite)

// OUT_CH5
#define	OUT_CLAMP1_CNTL					0x02		// 2 out2
#define	OUT_CLAMP2_CNTL					0x08		// 4 out4
#define	OUT_CLAMP3_CNTL					0x20		// 6 out6
#define	OUT_CLAMP4_CNTL					0x80		// 8 out8

//#define	OUT_RES1_CNT					0x20		// 6
//#define	OUT_RC_PWM_5V					0x40		// 7
#define	OUT_HEAT_CNTL					0x01		// 8


// OUT_CH6
//#define	OUT_RES2_CNTL					0x01		// 9
//#define	OUT_RES3_CNTL					0x02		// 10
#define	OUT_TL_R_CNTL					0x04		// 11
#define	OUT_TL_G_CNTL					0x01		// 12
#define	OUT_VACCUM_CNTL 				0x0a		// a out2,4
#define	OUT_TL_Y_CNTL					0x10		// 13
//#define	OUT_RES4_CNTL					0x20		// 14
//#define	OUT_RES5_CNTL					0x40		// 15
//#define	OUT_RES6_CNTL					0x80		// 16

// OUT_CH7
//#define	OUT_RES7_CNTL					0x01		// 17
//#define	OUT_RES8_CNTL					0x02		// 18
//#define	OUT_RES9_CNTL					0x04		// 19
//#define	OUT_RES10_CNTL					0x08		// 20
//#define	OUT_RES11_CNTL					0x10		// 21
//#define	OUT_RES12_CNTL					0x20		// 22
//#define	OUT_RES13_CNTL					0x40		// 23
//#define	OUT_RES14_CNTL					0x80		// 24

// OUT_CH8
//#define	OUT_RES15_CNTL					0x01		// 25
//#define	OUT_RES16_CNTL					0x02		// 26


/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
extern	unsigned char	scFg;	// input scan complete flag
extern	unsigned char	rsFg;	// read signal timer flag
extern	unsigned int	rsCnt;	// read signal timer counter
extern	unsigned char	inCh;	// input channel
extern	unsigned char	stSVin[4];	// Servo motor input status1~4
//extern	unsigned char	stSVout[4];	// Servo motor output status1~4
extern	unsigned char	stOut[8];	// Servo motor output status1~4 and out signal buffer
extern	unsigned long	stSWin;
extern	unsigned long	stSENin;


/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */
extern	void readID(void);
extern  void outSignal(unsigned char ch, unsigned int sdata);
extern  void readSignalProcess(void);
extern  void chkError(void);

extern  void outTest(void);
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_IO_CNTL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

