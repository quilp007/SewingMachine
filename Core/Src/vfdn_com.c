/**
  ******************************************************************************
  * @file           : vfdn_com.c
  * @brief          : communication to PC 
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
#include "vfdn_com.h"
#include "vfdn_rtc.h"
#include "vfdn_sub.h"
#include "vfdn_io_cntl.h"


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
unsigned char	comStep = COM_STEP_STX1;	// communication
unsigned char	comBuf[COM_BUF_MAX];
unsigned char	comBufCnt = 0;	// communication buffer counter
unsigned char	comIntCnt = 0;	// communication interrupt buffer counter
unsigned char	comEndFg = 0;	// communication 1packet end flag
unsigned char	comDataCnt = 0;	// data buffer counter
unsigned char	comDataBuf[COM_DATA];	// receive data buffer
unsigned char	comCycleFg = 0;	// communication cycle flag for timer
unsigned int	comCycleCnt = 0;	// communication cycle counter for timer
unsigned char	comSoftStop = 0;	// software stop from communication
unsigned char	comOKFg = 0;	// communication ok flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void comCheck(void);
void comAnalysis(void);
static void comDataSort(void);
void putChar(UART_HandleTypeDef * selport, unsigned char sdata);
void putStr(UART_HandleTypeDef * selport, const unsigned char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief communication check
  * @param None
  * @retval void
  */
void comCheck(void)
{
	unsigned char temp=0;

	if((comBufCnt != comIntCnt) && (comEndFg == 0))	// receive buffer counter와 check buffer counter가 다르면...
	{
		temp = comBuf[comBufCnt++];	// 마지막 체크한 데이터의 다음데이터를 확인하기 위하여...
		
		if(comBufCnt == COM_BUF_MAX)	comBufCnt = 0;	// 버퍼 카운터 초기화..

		switch(comStep)
		{
			case COM_STEP_STX1:	// stx1 : 0x02
								if(temp == COM_STX1)	// 현재 데이터 0x02
								{
#ifdef	__DEBUG_COM__
putStr(&huart1,"STX1\n");
#endif
									comStep = COM_STEP_STX2;	// next stx2 : 0x0d
								}
								else
								{
									comStep = COM_STEP_STX1;	// 형식에 맞지 않거나 노이즈...
								}
				break;

			case COM_STEP_STX2:	// stx2 : 0x0d
								if(temp == COM_STX2)	// 현재 데이터 0x0d
								{
#ifdef	__DEBUG_COM__
putStr(&huart1,"STX2\n");
#endif
									comStep = COM_STEP_DATA;	// next data : 14byte
									comDataCnt = 0;
								}
								else
								{
									comStep = COM_STEP_STX1;	// 형식에 맞지 않거나 노이즈...
								}
				break;

			case COM_STEP_DATA:	// 14byte data receive
								comDataBuf[comDataCnt] = temp;	// data restore buffer
#if 0
#ifdef	__DEBUG_COM__
putChar(&huart1,comDataBuf[comDataCnt]);
#endif
#endif
								comDataCnt++;

								if(comDataCnt == COM_DATA)	// 15byte received
								{
#ifdef	__DEBUG_COM__
putChar(&huart1,(comDataCnt/10)+0x30);
putChar(&huart1,(comDataCnt%10)+0x30);
putStr(&huart1,"EA DATA END\n");
#endif
									comStep = COM_STEP_ETX1;	// next etx1 : 0x0b
								}
				break;

			
			case COM_STEP_ETX1:	// etx1 : 0x0b
								if(temp == COM_ETX1)	// 현재 데이터 0x0b
								{
#ifdef	__DEBUG_COM__
putStr(&huart1,"ETX1\n");
#endif
									comStep = COM_STEP_ETX2;	// next etx2 : 0x0a
								}
								else
								{
									comStep = COM_STEP_STX1;	// 형식에 맞지 않거나 노이즈...
								}			
				break;

			case COM_STEP_ETX2:	// etx1 : 0x0b
								if(temp == COM_ETX2)	// 현재 데이터 0x0a
								{
#ifdef	__DEBUG_COM__
putStr(&huart1,"ETX2\n");
#endif
									comEndFg = 1;	// 정상적으로 1packet이 수신됨...

									comCycleCnt = COM_PACKET_CYCLE;
									comCycleFg = COM_PACKET_TIM_START;

                                    comOKFg = 1;
									
								}

								comStep = COM_STEP_STX1;	// 형식에 맞지 않거나 노이즈...
								comDataCnt = 0;
				break;

			default:
				break;
		}
	}
	else
	{
		if(comCycleFg == COM_PACKET_TIM_OCCUR)	// PC로부터 COM_PACKET_CYCLE시간만큼 지나도 1PACKET전송이 되지 않음...
		{
			comCycleFg = COM_PACKET_TIM_ERROR;
			comCycleCnt = 0;

			comEndFg = 0;

			comOKFg = 0;
		}
	}
}

/**
  * @brief communication analysis
  * @param None
  * @retval void
  */
void comAnalysis(void)
{
	unsigned int temp=0;
    //float buf=0;
	
	if(comEndFg == 1)
	{
		if(comDataBuf[0] == bdID)	// 설정된 보드 ID와 PC에서 보낸 목적지 ID가 같으면...
		{
			// meterial length
			temp = (unsigned int)comDataBuf[COM_METERIAL_LEN];

			temp <<= 8;
			temp &= 0xff00;

			temp |= ((unsigned int)comDataBuf[COM_METERIAL_LEN+1])&0x00ff;

			if((temp >= METERIAL_LEN_MIN) && (temp <= METERIAL_LEN_MAX))
			{
			    metLen = temp;
				
				comOKFg = 0x01;	// 통신이 정상 완료 됨.
			}
            else
            {
                //sewMovMaxPulse = 0;
                comOKFg = 0x00;	// 통신이 정상 완료 됨.
            }

			// error code(stop working command from PC)
			if(comDataBuf[COM_ERROR_CODE] & COM_SOFT_STOP)	comSoftStop = 1;
			else											comSoftStop = 0;

			comDataSort();
		}
		else if(comDataBuf[0] == 'T')	// time set
		{
			rWeekDay = comDataBuf[1];
			rYear = comDataBuf[2];
			rMonth = comDataBuf[3];
			rDate = comDataBuf[4];
			rHour = comDataBuf[5];
			rMinute = comDataBuf[6];
			rSecond = comDataBuf[7];

			setRTCTime();
		}

		//comCycleCnt = COM_PACKET_CYCLE;
		//comCycleFg = COM_PACKET_TIM_START;	// next communication timer check start
		
		comEndFg = 0;
	}
}

/**
  * @brief making serial data function
  * @param None
  * @retval None
  */	
static void comDataSort(void)
{
	unsigned char cnt=0,tbuf[19],i=0;

	tbuf[cnt++] = COM_STX1;	// 0x02
	tbuf[cnt++] = COM_STX2;	// 0x0b

	tbuf[cnt++] = bdID;	// 우선 보드ID로 대신...
	tbuf[cnt++] = heatCurr;  // Band cut heater current data(10배 데이터)
	tbuf[cnt++] = apkPa;  // air pressure data(1/10배 데이터)
	tbuf[cnt++] = 0;  // reserved adc data - null
	
	if(stSWin & ST_SW_EMS)	tbuf[cnt] |= 0x01;  // Emergency stop push lock switch on
	else					tbuf[cnt] &= ~0x01;  // Emergency stop push lock switch off

	if(stSWin & ST_SW_AUTO_MANUAL)	tbuf[cnt++] |= 0x10;  // auto/manual switch - auto
	else							tbuf[cnt++] &= ~0x10;  // auto/manual switch - manual

	if(stSENin & ST_SEN_LIFTING_UP_LIM)	tbuf[cnt] |= 0x01;  // lifting up limit snesor on
	else								tbuf[cnt] &= ~0x01;  // lifting up limit snesor off

	if(stSENin & ST_SEN_LIFTING_DN_LIM)	tbuf[cnt] |= 0x02;  // lifting down limit snesor on
	else								tbuf[cnt] &= ~0x02;  // lifting down limit snesor off

	if(stSENin & ST_SEN_LIFTING_HOME)	tbuf[cnt] |= 0x04;  // lifting home snesor on
	else								tbuf[cnt] &= ~0x04;  // lifting home snesor off

	if(stSENin & ST_SEN_NEEDLE_HOME)	tbuf[cnt] |= 0x08;  // needle home sensor on
	else								tbuf[cnt] &= ~0x08;  // needle home snesor off

	if(stSENin & ST_SEN_BAND_EMPTY1)	tbuf[cnt] |= 0x10;  // band empty1 sensor on
	else								tbuf[cnt] &= ~0x10;  // band empty1 snesor off

	if(stSENin & ST_SEN_BAND_EMPTY2)	tbuf[cnt] |= 0x20;  // band empty2 sensor on
	else								tbuf[cnt] &= ~0x20;  // band empty2 snesor off

	if(stSENin & ST_SEN_BAND_LOST1)	tbuf[cnt] |= 0x40;  // band lost1 sensor on
	else							tbuf[cnt] &= ~0x40;  // band lost1 snesor off

	if(stSENin & ST_SEN_BAND_LOST2)	tbuf[cnt++] |= 0x80;  // band lost2 sensor on
	else							tbuf[cnt++] &= ~0x80;  // band lost2 snesor off

	if(stSENin & ST_SEN_UPPER_THREAD_CUT_OFF)	tbuf[cnt] |= 0x01;  // upper thread cut sensor on
	else										tbuf[cnt] &= ~0x01;  // upper thread cut snesor off

	if(stSENin & ST_SEN_LOOPER_HOME)	tbuf[cnt] |= 0x02;  // looper home sensor on
	else								tbuf[cnt] &= ~0x02;  // looper home snesor off

	if(stSENin & ST_SEN_UNDER_THREAD_CUT_OFF)	tbuf[cnt] |= 0x04;  // under thread cut sensor on
	else										tbuf[cnt] &= ~0x04;  // under thread cut snesor off

	if(stSENin & ST_SEN_BAND_INSERT1)	tbuf[cnt] |= 0x08;  // band insert1 sensor on
	else								tbuf[cnt] &= ~0x08;  // band insert1 snesor off

	if(stSENin & ST_SEN_BAND_INSERT2)	tbuf[cnt] |= 0x10;  // band insert2 sensor on
	else								tbuf[cnt] &= ~0x10;  // band insert2 snesor off

	if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)	tbuf[cnt] |= 0x20;  // moving backward limit sensor on
	else										tbuf[cnt] &= ~0x20;  // moving backward limit snesor off

	if(stSENin & ST_SEN_MOVING_FORWARD_LIM)	tbuf[cnt] |= 0x40;  // moving forward limit sensor on
	else									tbuf[cnt] &= ~0x40;  // moving forward limit snesor off

	if(stSENin & ST_SEN_MOVING_HOME)	tbuf[cnt++] |= 0x80;  // moving home sensor on
	else								tbuf[cnt++] &= ~0x80;  // moving home snesor off

	if(stSENin & ST_SEN_FABRIC_DETECT1)	tbuf[cnt] |= 0x01;  // fabric detect1 sensor on
	else								tbuf[cnt] &= ~0x01;  // fabric detect1 snesor off

	if(stSENin & ST_SEN_FABRIC_DETECT2)	tbuf[cnt] |= 0x02;  // fabric detect2 sensor on
	else								tbuf[cnt] &= ~0x02;  // fabric detect2 snesor off

	if(stSENin & ST_SEN_CLAMP12_OPEN)	tbuf[cnt] |= 0x04;  // clamp12 open sensor on
	else								tbuf[cnt] &= ~0x04;  // clamp12 open snesor off

	if(stSENin & ST_SEN_CLAMP12_CLOSE)	tbuf[cnt] |= 0x08;  // clamp12 close sensor on
	else								tbuf[cnt] &= ~0x08;  // clamp12 close snesor off

	if(stSENin & ST_SEN_CLAMP34_OPEN)	tbuf[cnt] |= 0x10;  // clamp34 open sensor on
	else								tbuf[cnt] &= ~0x10;  // clamp34 open snesor off

	if(stSENin & ST_SEN_CLAMP34_CLOSE)	tbuf[cnt++] |= 0x20;  // clamp34 close sensor on
	else								tbuf[cnt++] &= ~0x20;  // clamp34 close snesor off

	if(stSENin & ST_SW_CLAMP1_OPEN_CLOSE)	tbuf[cnt] |= 0x01;  // clamp1 2select switch on - close
	else									tbuf[cnt] &= ~0x01;  // clamp1 2select switch off - open

	if(stSENin & ST_SW_CLAMP2_OPEN_CLOSE)	tbuf[cnt] |= 0x02;  // clamp2 2select switch on - close
	else									tbuf[cnt] &= ~0x02;  // clamp2 2select switch off - open

	if(stSENin & ST_SW_CLAMP3_OPEN_CLOSE)	tbuf[cnt] |= 0x04;  // clamp3 2select switch on - close
	else									tbuf[cnt] &= ~0x04;  // clamp3 2select switch off - open

	if(stSENin & ST_SW_CLAMP4_OPEN_CLOSE)	tbuf[cnt] |= 0x08;  // clamp4 2select switch on - close
	else									tbuf[cnt] &= ~0x08;  // clamp4 2select switch off - open

	if(stSENin & ST_SW_VACCUM_OPEN_CLOSE)	tbuf[cnt] |= 0x10;  // vaccum push switch on - open
	else									tbuf[cnt] &= ~0x10;  // vaccum push switch off - close

	tbuf[cnt++] |= (sewAct&0xe0);  // sewing machine status bit set

	tbuf[cnt] = (unsigned char)(metLen>>8);  // meterial lenght MSB
	tbuf[cnt++] = (unsigned char)metLen;  // meterial lenght LSB

	tbuf[cnt++] = errorCode[0];  // error code
	tbuf[cnt++] = errorCode[1];  // error code
	tbuf[cnt++] = errorCode[2];  // error code
	tbuf[cnt++] = errorCode[3];  // error code
	
	tbuf[cnt++] = COM_ETX1;	// 0x0d
	tbuf[cnt++] = COM_ETX2;	// 0x0a

	for(i=0; i<cnt; i++)
	{
		putChar(&huart1, tbuf[i]);
	}
}

/**
  * @brief one byte serial transfer function
  * @param uart port, transfer data
  * @retval None
  */
void putChar(UART_HandleTypeDef * selport, unsigned char sdata)
{
	HAL_UART_Transmit(selport, (unsigned char *)&sdata,1,100);
}

/**
  * @brief one byte serial transfer function
  * @param uart port, transfer data
  * @retval None
  */
void putStr(UART_HandleTypeDef * selport, const unsigned char *str)
{
	while(*str)
	{
		putChar(selport, *str++);
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

