/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_roe.h
  * @brief          : Header for vfdn_roe.c file.
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
#ifndef __VFDN_ROE_H
#define __VFDN_ROE_H

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
#define	__DEBUG_ROE__

#define	ROE_CNT_MAX						100

#define	ROE_CLK_REF_CNT					60000

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
#if 0
extern	unsigned char	roeNowFg;	// rotary encoder cw/ccw flag
extern	unsigned char	roePrevFg;	// rotary encoder cw/ccw flag
unsigned long   roeCWNowCnt = 0;
unsigned long   roeCCWNowCnt = 0;
unsigned long   roePrevCnt = 0;
#else
extern	signed long		roeNowCnt;
extern	signed long		roePrevCnt;
extern	unsigned char	roeEvent;	// rotary encoder cw/ccw now event flag
extern	unsigned char	roeDirFg;
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
//extern	void roeCheck(void);

/* USER CODE END EFP */
/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_ROE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

