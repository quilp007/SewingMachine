/**
  ******************************************************************************
  * @file           : vfdn_spi.c
  * @brief          : VFDN auto making controller spi communication
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
#include "vfdn_spi.h"

#ifdef	__DEBUG_SPI__
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
unsigned char	apFg = AP_SCAN_TSTART;	// Air Pressure adc timer flag
unsigned int	apCnt = AP_SCAN_TCNT;	// Air Pressure adc timer counter
unsigned char	apBufCnt = 0;	// Air pressure adc data buffer counter
unsigned int	apBuf[AP_CNT_MAX];	// adc buffer
//unsigned char	apBuf[AP_CNT_MAX];	// adc buffer
unsigned int	apkPa = 0;	// result
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void apCheck(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief air pressure ADC check(SPI)
  * @param None
  * @retval void
  */
void apCheck(void)
{
	unsigned char i=0,j=0, buf[2];
	unsigned long avr=0,sum=0;
	float dat=0,kpa=0;
	
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

