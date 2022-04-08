/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_sub.h
  * @brief          : Header for vfdn_sysInit.c file.
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
#ifndef __VFDN_IO_CNTL_H
#define __VFDN_IO_CNTL_H

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
//#define	__DEBUG_IO__

#define	IN_CH_MAX						8

#define READ_US_DLY                     5



#define	IN_CH0							0
#define	IN_CH1							1
#define	IN_CH2							2
#define	IN_CH3							3
#define	IN_CH4							4
#define	IN_CH5							5
#define	IN_CH6							6
#define	IN_CH7							7

#define	OUT_CH0							0
#define	OUT_CH1							1
#define	OUT_CH2							2
#define	OUT_CH3							3
#define	OUT_CH4							4
#define	OUT_CH5							5
#define	OUT_CH6							6
#define	OUT_CH7							7


/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/



/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */
extern  void outSignal(unsigned char ch, unsigned int sdata);
extern  void readSignalProcess(void);


extern void send2PC(uint8_t command,uint8_t   ch_num, uint8_t read_port_data);

/* Private defines -----------------------------------------------------------*/



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VFDN_IO_CNTL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

