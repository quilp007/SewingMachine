/**
  ******************************************************************************
  * @file           : vfdn_rtc.c
  * @brief          : real time clock write and read
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
#include "vfdn_rtc.h"

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
/* USER CODE BEGIN PV */
unsigned char	rYear = 21;	// year data
unsigned char	rMonth = 12;	// month data
unsigned char	rDate = 1;	// date data
unsigned char	rHour = 0;	// hour data
unsigned char	rMinute = 0;	// minute data
unsigned char	rSecond = 0;	// second data
unsigned char	rWeekDay = RTC_WEEKDAY_WEDNESDAY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void getRTCTime(void);
void setRTCTime(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief get rtc time
  * @param None
  * @retval void
  */
void getRTCTime(void)
{
	//RTC_TimeTypeDef sTime;
	//RTC_DateTypeDef sDate;

	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);

	rWeekDay = sDate.WeekDay;
	rYear = sDate.Year;
	rMonth = sDate.Month;
	rDate = sDate.Date;
	rHour = sTime.Hours;
	rMinute = sTime.Minutes;
	rSecond = sTime.Seconds;
}

/**
  * @brief set rtc time
  * @param None
  * @retval void
  */
void setRTCTime(void)
{
	//RTC_TimeTypeDef sTime;
	//RTC_DateTypeDef sDate;

	sDate.WeekDay = rWeekDay;
	sDate.Year = rYear;
	sDate.Month = rMonth;
	sDate.Date = rDate;
	sTime.Hours = rHour;
	sTime.Minutes = rMinute;
	sTime.Seconds = rSecond;

	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

