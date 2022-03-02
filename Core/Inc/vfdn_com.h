/**
  ******************************************************************************
  * @file           : vfdn_com.h
  * @brief          : This file contains the common defines of the application.
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
#ifndef __VFDN_COM_H
#define __VFDN_COM_H

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
/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
//#define	__DEBUG_COM__

#define	COM_BUF_MAX						50

#define	COM_STX1						0x02
#define	COM_STX2						0x0b
#define	COM_ETX1						0x0d
#define	COM_ETX2						0x0a

#define	COM_STEP_STX1					0
#define	COM_STEP_STX2					1
#define	COM_STEP_DATA					2
#define	COM_STEP_ETX1					3
#define	COM_STEP_ETX2					4

#define	COM_DATA						15

#define	COM_PACKET_CYCLE				120	// 120msec안에 통신이 되면 됨....

#define	COM_PACKET_TIM_STOP				0
#define	COM_PACKET_TIM_START			1
#define	COM_PACKET_TIM_OCCUR			2
#define	COM_PACKET_TIM_ERROR			3

#define	COM_ID							0
#define	COM_METERIAL_LEN				9
#define	COM_ERROR_CODE					14
#define	COM_SOFT_STOP					0x80

/* USER CODE END Private defines */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern	unsigned char	comBuf[COM_BUF_MAX];
extern	unsigned char	comIntCnt; // communication interrupt buffer counter
extern	unsigned char	comCycleFg; // communication cycle flag for timer
extern	unsigned int	comCycleCnt; // communication cycle counter for timer
extern  unsigned char	comSoftStop; // software stop from communication
extern	unsigned char	comOKFg;	// communication ok flag
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
extern	void comCheck(void);
extern	void comAnalysis(void);
extern	void putChar(UART_HandleTypeDef * selport, unsigned char sdata);
extern	void putStr(UART_HandleTypeDef * selport, const unsigned char *str);
/* USER CODE END EFP */


#ifdef __cplusplus
}
#endif

#endif /* __VFDN_COM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

