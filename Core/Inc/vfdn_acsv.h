/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_acsv.h
  * @brief          : Header for vfdn_acsv.c file.
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
#ifndef __VFDN_ACSV_H
#define __VFDN_ACSV_H

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
//#define	__DEBUG_ACSV__

#define	SEW_MOTOR_1CYCLE_CLOCK		100
#define	MOV_MOTOR_1CYCLE_CLOCK		100

// 200(N1):1(M)
// calculat : want move length(mm)/ONE_PULSE_MOV
#define	ONE_PULSE_MV                (float)0.016421367
#define	ONE_PULSE_LF				(float)0.00625
#define	ONE_PULSE_NL                (float)0.007657632

#define NL_1MM_CLOCK                130

#define	MOV_1MM_CLOCK				60
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

#define	SEW_1CYCLE_PULSE            1600//1600
#define	SEW_OFF_FABRIC_PULSE        1250// 1400 short needle, 1200 long needle
	
#define	MOV_1CYCLE_PULSE	    	MOV_7MM_CLOCK	// ddam length
#define	MOV_MAX_PULSE	    		MOV_1500MM_CLOCK	// total meterial length
#define MOV_ROE_MAX_PULSE           MOV_10MM_CLOCK // roe input run

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern  unsigned long   svEtiCnt; // moving servo pulse count for external interrupt
extern  unsigned long   svRoeCnt; // moving servo pulse count ROE for external interrupt
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_ACSV_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

