/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_adc.h
  * @brief          : Header for vfdn_adc.c file.
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
#ifndef __VFDN_ADC_H
#define __VFDN_ADC_H

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
//#define	__DEBUG_ADC__

#define	ADC_SCAN_TCNT					5	// AD Convertor timer counter per ADC_TCNT msec
#define	ADC_SCAN_TSTART					1
#define	ADC_SCAN_NOW					2

#define	ADC_CNT_MAX						40

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern	unsigned char	adcFg;	// adc timer flag
extern	unsigned int	adcCnt;	// adc timer counter
extern	unsigned int	adcV; //adc volatge

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
extern	void adcCheck(void);

/* USER CODE END EFP */
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_ADC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

