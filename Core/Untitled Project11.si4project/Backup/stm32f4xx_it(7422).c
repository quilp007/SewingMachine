/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"

#include "vfdn_adc.h"
#include "vfdn_spi.h"
#include "vfdn_com.h"
#include "vfdn_sub.h"
#include "vfdn_roe.h"
#include "vfdn_acsv.h"
#include "vfdn_io_cntl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */
    
	if(sysLedCnt == 0)
	{
		LED_RUN_TOG;

		sysLedCnt = SYS_LED_TCNT;
	}
	else
		sysLedCnt--;
	
    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI0_IRQn 0 */
	// Needle PWM counter read
	if(sysMode & MODE_AUTO)    		sewRunCnt++;
	else if(sysMode & MODE_MANUAL)
	{
		if(sysMode & MODE_MTEST)	tSewCnt++;
	}
    
	/* USER CODE END EXTI0_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	/* USER CODE BEGIN EXTI0_IRQn 1 */

	/* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI1_IRQn 0 */
	// Looper PWM counter read
	if(looperFg)	looperCnt++;
    //if(sysMode & MODE_MANUAL)	svEtiCnt++;
	/* USER CODE END EXTI1_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	/* USER CODE BEGIN EXTI1_IRQn 1 */

	/* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI2_IRQn 0 */
	// Moving PWM counter read
    if(sysMode & MODE_MANUAL)
    {
		if(sysMode & MODE_MTEST)
		{
			tMovCnt++;
			tMovMaxCnt++;
		}
		
		if(roeMode & 0x01)	svEtiCnt++;// 바늘 높낮이 위치 초기화를 위한 카운터를 체크해야 한다....
    }
		

    if(sysMode & MODE_AUTO)
    {
    
        sewMovCnt++;
		sewMovMaxCnt++;		

		if(movFg == 1)	movCnt++;
    }
	/* USER CODE END EXTI2_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	/* USER CODE BEGIN EXTI2_IRQn 1 */

	/* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI3_IRQn 0 */
	// Lifting PWM counter read
    if(sysMode & MODE_MANUAL)
	{
	//putChar(&huart4,'A');
		svEtiCnt++;// 바늘 높낮이 위치 초기화를 위한 카운터를 체크해야 한다....
    }

	if(lfFg == 1)	lfCnt++;
	/* USER CODE END EXTI3_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	/* USER CODE BEGIN EXTI3_IRQn 1 */

	/* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != 0x00u)
    {
        if(sysMode & MODE_MANUAL)
        {
        #if 0
            if(HAL_GPIO_ReadPin(GPIOB,ROE_B) == 0) // CW
            {
                //if(roeCWNowCnt == 60000)    roeCWNowCnt = 0;
                //else                        roeCWNowCnt++;
                
                roeCWNowCnt++;
                roeCCWNowCnt = 0;
                roeNowFg = 1;

                //if(testROECnt == 60000) testROECnt = 0;
                //else                    testROECnt++;
            }
            else // CCW
            {
                //if(roeCCWNowCnt == 60000)   roeCCWNowCnt = 0;
                //else                        roeCCWNowCnt++;
                
                roeCCWNowCnt++;
                roeCWNowCnt = 0;
                roeNowFg = 2;

                //if(testROECnt == 0) testROECnt = 60000;
                //else                testROECnt--;
            }
		#else
			//if(HAL_GPIO_ReadPin(GPIOB,ROE_B) == RESET) // CW
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == 0) // CW
			{
				roeEvent = 1;

				roeNowCnt++; // 증가
			}
			else	// ccw
			{
				roeEvent = 2;

				roeNowCnt--;
			}
		#endif
        }
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    }
    
	/* USER CODE END EXTI9_5_IRQn 0 */
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */
    if(testFg == 1)
    {
        if(testCnt == 0)    testFg = 2;
        else                testCnt--;
    }
    
	// check communication to PC per COM_PACKET_CYCLE
    // read signal input timer
	// air pressure data scan timer
	if(apFg == AP_SCAN_TSTART)
	{
		if(apCnt == 0)	apFg = AP_SCAN_NOW;
		else			apCnt--;
	}

	// status check timer
	if(stCkFg == INIT_TSTART)
	{
		if(stCkCnt == 0)	stCkFg = INIT_TOVER;
		else				stCkCnt--;
	}
	
	if(rsFg == READ_SIG_TSTART)
	{
		if(rsCnt == 0)	rsFg = READ_SIG_NOW;
		else			rsCnt--;
	}

	if(swFg == SW_SCAN_TSTART)
	{
		if(swCnt == 0)	swFg = SW_SCAN_NOW;
		else			swCnt--;
	}

	if(bcFg == RC_GEN_TSTART)
	{
		if(bcCnt == 0)	bcFg = RC_GEN_NOW;
		else			bcCnt--;
	}

    // machine lifting home timer
	if(initFg[INIT_STEP_LIFTING] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LIFTING] == 0)	initFg[INIT_STEP_LIFTING] = INIT_TOVER;
		else								initCnt[INIT_STEP_LIFTING]--;
	}

	// machine needle home timer
	if(initFg[INIT_STEP_NEEDLE] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_NEEDLE] == 0)	initFg[INIT_STEP_NEEDLE] = INIT_TOVER;
		else								initCnt[INIT_STEP_NEEDLE]--;
	}

	// machine looper home timer
	if(initFg[INIT_STEP_LOOPER] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LOOPER] == 0)	initFg[INIT_STEP_LOOPER] = INIT_TOVER;
		else								initCnt[INIT_STEP_LOOPER]--;
	}

	// machine moving home timer
	if(initFg[INIT_STEP_MOVING] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_MOVING] == 0)	initFg[INIT_STEP_MOVING] = INIT_TOVER;
		else								initCnt[INIT_STEP_MOVING]--;
	}

    // machine clamp open/close timer
	if(initFg[INIT_STEP_CLAMP] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_CLAMP] == 0)	initFg[INIT_STEP_CLAMP] = INIT_TOVER;
		else								initCnt[INIT_STEP_CLAMP]--;
	}
    
    /*
	if(comCycleFg == COM_PACKET_TIM_START)
	{
		if(comCycleCnt == 0)	comCycleFg = COM_PACKET_TIM_OCCUR;
		else					comCycleCnt--;
	}
	
	// read signal input timer
	if(rsFg == READ_SIG_TSTART)
	{
		if(rsCnt == 0)	rsFg = READ_SIG_NOW;
		else			rsCnt--;
	}

	// adc(band cut heat current) scan timer
	if(adcFg == ADC_SCAN_TSTART)
	{
		if(adcCnt == 0)	adcFg = ADC_SCAN_TCNT;
		else			adcCnt--;
	}

	// air pressure data scan timer
	if(apFg == AP_SCAN_TSTART)
	{
		if(apCnt == 0)	apFg = AP_SCAN_TCNT;
		else			apCnt--;
	}

	// machine lifting home timer
	if(initFg[INIT_STEP_LIFTING] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LIFTING] == 0)	initFg[INIT_STEP_LIFTING] = INIT_TOVER;
		else								initCnt[INIT_STEP_LIFTING]--;
	}

	// machine needle home timer
	if(initFg[INIT_STEP_NEEDLE] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LIFTING] == 0)	initFg[INIT_STEP_NEEDLE] = INIT_TOVER;
		else								initCnt[INIT_STEP_LIFTING]--;
	}

	// machine looper home timer
	if(initFg[INIT_STEP_LOOPER] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LIFTING] == 0)	initFg[INIT_STEP_LOOPER] = INIT_TOVER;
		else								initCnt[INIT_STEP_LIFTING]--;
	}

	// machine moving home timer
	if(initFg[INIT_STEP_MOVING] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_LIFTING] == 0)	initFg[INIT_STEP_MOVING] = INIT_TOVER;
		else								initCnt[INIT_STEP_LIFTING]--;
	}

    // machine clamp open/close timer
	if(initFg[INIT_STEP_CLAMP] == INIT_TSTART)
	{
		if(initCnt[INIT_STEP_CLAMP] == 0)	initFg[INIT_STEP_CLAMP] = INIT_TOVER;
		else								initCnt[INIT_STEP_CLAMP]--;
	}

	// status check timer
	if(stCkFg == INIT_TSTART)
	{
		if(stCkCnt == 0)	stCkFg = INIT_TOVER;
		else				stCkCnt--;
	}

    // tower lamp on-off timer
	if(tlFg == TLAMP_RUN_TSTART)
	{
		if(tlCnt == 0)	tlFg = TLAMP_RUN_NOW;
		else			tlCnt--;
	}
    */
	/*
	if(sysLedCnt == 0)
	{
		LED_RUN_TOG;

		sysLedCnt = SYS_LED_TCNT;
	}
	else
		sysLedCnt--;
        */
    #if 0
    // tower lamp on-off timer
	if(manualFg	== MANUAL_TSTART)
	{
		if(manualCnt == 0)	manualFg = MANUAL_NOW;
		else			    manualCnt--;
	}
	#endif
	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */
	unsigned long tfg=0,titsrc=0;

	tfg = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
	titsrc = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE);

	if((tfg != RESET) && (titsrc != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		
		//comBuf[comIntCnt++] = (unsigned char)(huart1.Instance->DR & 0x00ff);

		//if(comIntCnt == COM_BUF_MAX)	comIntCnt = 0;
	}
	else
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
	}
	/* USER CODE END USART1_IRQn 0 */
	//HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
	/* USER CODE BEGIN USART2_IRQn 0 */
	unsigned long tfg=0,titsrc=0;

	tfg = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE);
	titsrc = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE);

	if((tfg != RESET) && (titsrc != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(&huart2);
		
		//comBuf[comIntCnt++] = (unsigned char)(huart1.Instance->DR & 0x00ff);

		//if(comIntCnt == COM_BUF_MAX)	comIntCnt = 0;
	}
	else
	{
		__HAL_UART_CLEAR_PEFLAG(&huart2);
	}
	/* USER CODE END USART2_IRQn 0 */
	//HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */

	/* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
	/* USER CODE BEGIN USART3_IRQn 0 */
	unsigned long tfg=0,titsrc=0;

	tfg = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE);
	titsrc = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE);

	if((tfg != RESET) && (titsrc != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
		
		comBuf[comIntCnt++] = (unsigned char)(huart1.Instance->DR & 0x00ff);

		if(comIntCnt == COM_BUF_MAX)	comIntCnt = 0;
	}
	else
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	/* USER CODE END USART3_IRQn 0 */
	//HAL_UART_IRQHandler(&huart3);
	/* USER CODE BEGIN USART3_IRQn 1 */

	/* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
	/* USER CODE BEGIN UART4_IRQn 0 */
	unsigned long tfg=0,titsrc=0;

	tfg = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE);
	titsrc = __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_RXNE);

	if((tfg != RESET) && (titsrc != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(&huart4);
		
		//comBuf[comIntCnt++] = (unsigned char)(huart1.Instance->DR & 0x00ff);

		//if(comIntCnt == COM_BUF_MAX)	comIntCnt = 0;
	}
	else
	{
		__HAL_UART_CLEAR_PEFLAG(&huart4);
	}
	/* USER CODE END UART4_IRQn 0 */
	//HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */

	/* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
