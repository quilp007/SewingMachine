/**
  ******************************************************************************
  * @file           : vfdn_acsv.c
  * @brief          : analog servo control
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
#include "vfdn_acsv.h"
#include "vfdn_sub.h"

#ifdef	__DEBUG_ACSV__
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
unsigned long   svMovMLP = 0; // servo pulse count for meterial length
unsigned int    svMovDLP = 0; // servo pulse count for ddam length


unsigned int    svSewEtiCnt = 0; // upper & under sewing servo pulse count for external interrupt

unsigned long   svEtiCnt = 0; // moving servo pulse count for external interrupt
unsigned long   svRoeCnt = 0; // moving servo pulse count ROE for external interrupt

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void svMovLenCaluate(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief servo moving  total length calcuate
  * @param None
  * @retval void
  */
void svMovLenCaluate(void)
{
    float a=0;

    a = ((float)metLen / ONE_PULSE_MV)+1;

    svMovMLP = (unsigned long)a;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

