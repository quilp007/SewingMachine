/**
  ******************************************************************************
  * @file           : roe.c
  * @brief          : rotary encoder calcualtion
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
#include "vfdn_roe.h"

#ifdef	__DEBUG_ROE__
#include "vfdn_com.h"
#endif


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
#if 0
unsigned char	roeNowFg = 0;	// rotary encoder cw/ccw flag
unsigned char	roePrevFg = 0;	// rotary encoder cw/ccw flag
unsigned long   roeCWNowCnt = 0;
unsigned long   roeCCWNowCnt = 0;
unsigned long   roePrevCnt = 0;
#else
signed long		roeNowCnt = 0;
signed long		roePrevCnt = 0;
unsigned char	roeEvent = 0;	// rotary encoder cw/ccw now event flag
unsigned char	roeDirFg = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#if 0
/**
  * @brief rotary encoder check
  * @param None
  * @retval void
  */
void roeCheck(void)
{
	if(roeEvent)
	{
	    
		roeEvent = 0;
	}
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

