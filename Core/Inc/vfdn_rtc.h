/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_rtc.h
  * @brief          : Header for vfdn_rtc.c file.
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
#ifndef __VFDN_RTC_H
#define __VFDN_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
#define	__DEBUG_RTC__

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern	unsigned char	rYear;	// year data
extern	unsigned char	rMonth;	// month data
extern	unsigned char	rDate;	// date data
extern	unsigned char	rHour;	// hour data
extern	unsigned char	rMinute;	// minute data
extern	unsigned char	rSecond;	// second data
extern	unsigned char	rWeekDay;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
extern	void getRTCTime(void);
extern	void setRTCTime(void);
/* USER CODE END EFP */
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_RTC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

