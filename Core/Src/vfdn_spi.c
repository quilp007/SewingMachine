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
	
	if(apFg == AP_SCAN_NOW)
	{
		//if(HAL_SPI_Receive(&hspi2,(unsigned char *) &apBuf[apBufCnt],1,100) != HAL_OK)  // error
		if(HAL_SPI_Receive(&hspi2,buf,1,100) != HAL_OK)  // error
		{
#ifdef __DEBUG_SPI__
putStr(&huart4,(const unsigned char *)"SPI error!\n");
#endif
		}
		else	// success
		{
			apBuf[apBufCnt] = ((unsigned int)buf[0]&0x00ff)<<8;
			apBuf[apBufCnt] |= ((unsigned int)buf[1]&0x00ff);
			
			apBufCnt++; 
			
			if(apBufCnt == AP_CNT_MAX)
			{
				for(i=0; i<AP_CNT_MAX-1; i++)	// bubble sort
				{
					for(j=0; j<AP_CNT_MAX-i-1; j++)
					{
						if(apBuf[j] > apBuf[j+i])
						{
							avr = apBuf[j];
							apBuf[j] = apBuf[j+1];
							apBuf[j+1] = avr;
						}
					}
				}

				for(i=10; i<AP_CNT_MAX-10; i++)
				{
					sum += apBuf[i];
				}

				avr = sum/(AP_CNT_MAX-20);	// average, 최대값과 최소값을 제외한...
				avr-=1000;

				// ADC data to voltage
				dat = (float)(avr)*0.065/65535;	// now v convert
				dat = ((dat*5)/0.065); // 5v range convert
#ifdef __DEBUG_SPI__
putStr(&huart4,(const unsigned char *)"mV : ");
putChar(&huart4,((signed long)(dat*1000)/10000)+0x30);
putChar(&huart4,(((signed long)(dat*1000)%10000)/1000)+0x30);
putChar(&huart4,(((signed long)(dat*1000)%1000)/100)+0x30);
putChar(&huart4,(((signed long)(dat*1000)%100)/10)+0x30);
putChar(&huart4,((signed long)(dat*1000)%10)+0x30);
putChar(&huart4,'\n');
#endif


				dat -= 1.33;
				kpa = dat*250;

				/*
				// calculate current
				//dat /= 250;  // 250ohm resistor
				//dat -= 4;
								
				// current to kPa
				//kpa = (dat*1000)/16;	// 전체 구간 4 ~ 20mA를 -4하여 0 ~ 16mA구간으로 변경 후 kPa로 변환
				
				//kpa /= 10;
				*/
				
				apkPa = (unsigned int)kpa-76;

#ifdef __DEBUG_SPI__
#if 1
putStr(&huart4,(const unsigned char *)"avr : ");
putChar(&huart4,(avr/10000)+0x30);
putChar(&huart4,((avr%10000)/1000)+0x30);
putChar(&huart4,((avr%1000)/100)+0x30);
putChar(&huart4,((avr%100)/10)+0x30);
putChar(&huart4,(avr%10)+0x30);
putChar(&huart4,'\n');
#endif
#if 1
putStr(&huart4,(const unsigned char *)"apkPa : ");
putChar(&huart4,(apkPa/10000)+0x30);
putChar(&huart4,((apkPa%10000)/1000)+0x30);
putChar(&huart4,((apkPa%1000)/100)+0x30);
putChar(&huart4,((apkPa%100)/10)+0x30);
putChar(&huart4,(apkPa%10)+0x30);
putChar(&huart4,'\n');
#endif
#endif


				apBufCnt = 0;
			}
		}

		apCnt = AP_SCAN_TCNT;
		apFg = AP_SCAN_TSTART;
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

