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
#ifndef __VFDN_SUB_H
#define __VFDN_SUB_H

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
/* USER CODE BEGIN EM */
#define	__DEBUG_SUB__

#define	INIT_STEP_NEEDLE				0
#define	INIT_STEP_LOOPER				1
#define	INIT_STEP_MOVING				2
#define	INIT_STEP_LIFTING				3
#define	INIT_STEP_CLAMP 				4

#define SYS_LED_TCNT                    499

#define	INIT_TCNT						60000 //60sec
#define	INIT_TSTART						1
#define	INIT_TOVER						2

#define	SW_SCAN_TCNT					10 // switch scan timer MANUAL_TCNT msec
#define	SW_SCAN_TSTART					1
#define	SW_SCAN_NOW  					2

#define	MODE_SEWING						0
#define	MODE_TEST						1

#define	VACCUM_CLOSE					0
#define	VACCUM_OPEN						1

#define	CLAMP_OPEN						0
#define	CLAMP_CLOSE						1

#define	METERIAL_LEN_MIN				500
#define	METERIAL_LEN_MAX				2600

#define AP_KPA_MIN                      100
#define AP_KPA_MAX                      700

#define RC_DUTY_MIN                     1000
#define RC_DUTY_MAX                     2000
#define RC_DUTY_STEP                    1
#define	RC_GEN_TCNT			        	20
#define	RC_GEN_TSTART		        	1
#define	RC_GEN_NOW  		        	2

#define HEATER_CURR_MIN                 20 // 2A
#define HEATER_CURR_MAX                 80 // 8A

#define TLAMP_RUN_TSTART                1
#define TLAMP_RUN_NOW                   2
#define TLAMP_10MSEC_TCNT               10
#define TLAMP_200MSEC_TCNT              200
#define TLAMP_500MSEC_TCNT              500
#define TLAMP_1000MSEC_TCNT             1000

#define SEW_ACT_WAIT                    0x00 // sewing machine action wait
#define SEW_ACT_FR                      0x20 // sewing machine action forward run
#define SEW_ACT_FIN                     0x40 // sewing machine action finish
#define SEW_ACT_BR                      0x60 // sewing machine action backward run
#define SEW_ACT_AS                      0x80 // sewing machine action auto stop
#define SEW_ACT_ALL                     0xe0 // sewing machine action bit all

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern	unsigned char	bdID;	// Board ID
extern	unsigned char	heatCurr;	//Heater current 0 ~ 255(10배 데이터)
extern	unsigned int	apkPa;  // air pressure kPa 0 ~ 255(1/10배 데이터)
extern	unsigned int	metLen;	// 미싱길이 데이터.....

extern  unsigned int	sysLedCnt;
extern  unsigned char	sysMode;	// AUTO-MANUAL mode and EMS상태...0:EMS, 1:
extern	unsigned char   roeMode;    // Rotay encoder mode

extern  unsigned char	swFg; // switch scan timer flag
extern  unsigned int	swCnt; // switch scan timer counter

extern  unsigned char   bcFg; // band cut timer flag
extern  unsigned int    bcCnt; // band cut timer counter

extern	unsigned char	sewAct;	// sewing machine status

extern  unsigned long   sewRunCnt; // sewing machine run counter

extern  unsigned long   sewMovMaxCnt; // sewing machine moving max counter
extern  unsigned long   sewMovCnt; // sewing machine moving counter

//extern  unsigned long   sewMovMaxPulse; // 통신으로 받은 전체 미싱 길이 연산 결과 값

extern	unsigned char	errorCode[4];  // error code
extern  unsigned char   sysStartFg;

extern	unsigned char	initFg[5]; // init timer flag, [0] -> lifting, [1] -> needle, [2] -> looper, [3] -> moving
extern	unsigned int	initCnt[5]; // init timer counter, [0] -> lifting, [1] -> needle, [2] -> looper, [3] -> moving

extern	unsigned char	stCkFg; // status check timer flag
extern	unsigned int	stCkCnt; // status check timer counter

extern  unsigned char   tlFg; // tower lamp timer flag
extern  unsigned int    tlCnt; // tower lamp timer counter
extern  unsigned char   initFin; // location initial flag

extern  unsigned char   svRdy; // 0x01: servo1 ready, 0x02: servo2 ready, 0x04: servo3 ready, 0x08: servo4 ready

extern  unsigned int    testCnt;
extern  unsigned char   testFg;

extern  unsigned int    testROECnt;

extern	unsigned int	tSewCnt;
extern	unsigned int	tMovCnt;
extern	unsigned int	tMovMaxCnt;

extern	unsigned int	looperCnt;
extern	unsigned char	looperFg;

extern	unsigned long	movCnt;
extern	unsigned char	movFg;

extern	unsigned int	lfCnt;
extern	unsigned char	lfFg;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
extern  void cntlTLamp(void);
extern  void scanSw(void);
extern  void runningProcess(void);
extern  void testSV(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t ttime);
extern  void testROE(void);

extern	void cntlBandCut(void);
extern	void autoMode(void);

extern	unsigned char   sewRunFg; // sewing machine run flag
/* USER CODE END EFP */
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_SUB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

