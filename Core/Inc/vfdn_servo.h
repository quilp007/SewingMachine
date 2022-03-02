/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_servo.h
  * @brief          : Header for pwm.
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
#ifndef __VFDN_SERVO_H
#define __VFDN_SERVO_H

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
// RC SERVO
#define	RC_SV_PERIOD				10000		// 72MHz/72 = 1MHz, 1MHz/10000 = 100Hz = 10msec pulse
#define	RC_SV_A0					1500		// 1.5msec high duty angle 0dgree
#define	RC_SV_AP45					2000		// 2.0msec high duty angle +90dgree
#define	RC_SV_AN45					1000		// 1.0msec high duty angle -90dgree

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

#define	VSLOW_SEWING				FRQ_100Hz
#define	SLOW_SEWING					FRQ_500Hz
#define	FAST_SEWING					FRQ_10KHz
#define	VFAST_SEWING				FRQ_25KHz

#define	FRQ_AC_SV_NOR_OP            FRQ_25KHz	// 미싱 바느질 속도 관련....
#define	FRQ_RC_SV                   FRQ_62Hz
#define FRQ_AC_SV_TEST_OP           FRQ_500Hz
	

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */

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

#endif /* __VFDN_SERVO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

