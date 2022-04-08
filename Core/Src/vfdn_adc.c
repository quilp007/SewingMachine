/**
  ******************************************************************************
  * @file           : vfdn_adc.c
  * @brief          : VFDN auto making controller ADC(Heat current sensor)
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
#include "vfdn_adc.h"

#ifdef	__DEBUG_ADC__
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
unsigned char	adcFg = 0;	// adc timer flag
unsigned int	adcCnt = 0;	// adc timer counter
unsigned char	adcBufCnt = 0;	// adc buffer counter
unsigned int	adcBuf[ADC_CNT_MAX];	// adc buffer
//unsigned int	adcV = 0; //adc volatge

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void adcCheck(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief communication check
  * @param None
  * @retval void
  */
void adcCheck(void)
{
	unsigned char i=0,j=0;
	unsigned int avr=0,sum=0;
	float dat=0;
	
	if(adcFg == ADC_SCAN_NOW)
	{
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);

		adcBuf[adcBufCnt++] = HAL_ADC_GetValue(&hadc1);
			
		HAL_ADC_Stop(&hadc1);
			
		if(adcBufCnt == ADC_CNT_MAX)
		{
			for(i=0; i<ADC_CNT_MAX-1; i++)	// bubble sort
			{
				for(j=0; j<ADC_CNT_MAX-i-1; j++)
				{
					if(adcBuf[j] > adcBuf[j+i])
					{
						avr = adcBuf[j];
						adcBuf[j] = adcBuf[j+1];
						adcBuf[j+1] = avr;
					}
				}
			}

			for(i=1; i<ADC_CNT_MAX-1; i++)
			{
				sum += adcBuf[i];
			}

			avr = sum/(ADC_CNT_MAX-2);	// average, �ִ밪�� �ּҰ��� ������...

			dat = (3300*(float)avr)/4096;	// mV�� ����..

			//adcV = (unsigned int)dat;

			dat /= 50;	// 50mV per A

			//heatCurr = (unsigned char)(dat*10);	// 10�� ������ ����
			
			
			adcBufCnt = 0;
		}

		adcCnt = ADC_SCAN_TCNT;
		adcFg = ADC_SCAN_TSTART;
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

