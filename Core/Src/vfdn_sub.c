/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : vfdn_sub.c
  * @brief          : VFDN auto making sub-process
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
#include "stm32f407xx.h"
#include "main.h"
#include "vfdn_sub.h"
#include "dwt_stm32_delay.h"
#include "vfdn_io_cntl.h"
#include "vfdn_servo.h"
#include "vfdn_acsv.h"
#include "vfdn_roe.h"

#include "vfdn_com.h"

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
unsigned char	bdID = 0;	// Board ID

//unsigned char	sortFg = 0;	// machine 초기 정렬 flag

unsigned char	sysStartFg = 0;	// system start flag...

unsigned int	sysLedCnt = SYS_LED_TCNT;

unsigned char	sysMode = 0;	// AUTO-MANUAL mode and EMS상태...0x01:EMS, 1:
unsigned char   roeMode = 0;    // Rotay encoder mode
unsigned char   roeRunFg = 0; // moving/lifting switch flag
unsigned char   initMode = 0;   // Initalize mode
unsigned char	changeModeFg = 0;
//unsigned char   roeRatio = 0; // x1, x10

unsigned char	swFg = 0; // switch scan timer flag
unsigned int	swCnt = 0; // switch scan timer counter
unsigned char   swChkFg = 0; // switch scan check flag
unsigned long	swCur = 0; // current switch status
unsigned long	swPrev = 0;	// previous switch status
unsigned long   swStFg = 0; // switch toggle status flag   

unsigned char	heatCurr = 0;	//Heater current 0 ~ 255(10배 데이터)
//unsigned int	apkPa = 0;  // air pressure kPa 0 ~ 255(1/10배 데이터)
unsigned int	metLen = 0;	// fabric length

unsigned char	sewAct = SEW_ACT_WAIT;	// sewing machine status

unsigned char   sewRunFg = 0; // sewing machine run flag
unsigned long   sewRunCnt = 0; // sewing machine run counter

unsigned long   sewMovMaxCnt = 0; // sewing machine moving max counter
unsigned long   sewMovMaxPulse = 0; // 통신으로 받은 전체 미싱 길이 연산 결과 값
unsigned char   sewMovFg = 0; // sewing machine moving flag
unsigned long   sewMovCnt = 0; // sewing machine moving counter

unsigned char	errorCode[4];  // error code

unsigned char	initFg[5]; // init timer flag, [0] -> lifting, [1] -> needle, [2] -> looper, [3] -> moving, [4] -> Clamp
unsigned int	initCnt[5]; // init timer counter, [0] -> lifting, [1] -> needle, [2] -> looper, [3] -> moving, [4] -> Clamp
unsigned char	initStep[5]; // initialization step status
unsigned char	liftStep = 0;
unsigned int	liftCnt = 0;

unsigned char	stCkFg = 0; // status check timer flag
unsigned int	stCkCnt = 0; // status check timer counter
unsigned char	stStep = 0;
unsigned char   stPrevStep = 0;

unsigned char   bcFg = 0; // band cut timer flag
unsigned int    bcCnt = 0; // band cut timer counter
unsigned char   bcStep = 0; // band cut operation step flag

unsigned int    rcDuty[2]; // rc duty

unsigned char   rcInDc = 0; // rc servo increse and decrese

unsigned char   heatOnFg = 0; // heater on flag

unsigned char   tlFg = 0; // tower lamp timer flag
unsigned int    tlCnt = 0; // tower lamp timer counter
unsigned char   tlRGYFg = 0; // tower lamp control flag

unsigned char   initFin = 0; // location initial flag

unsigned char   svRdy = 0; // 0x01: servo1 ready, 0x02: servo2 ready, 0x04: servo3 ready, 0x08: servo4 ready

unsigned long	frqBuf = 0;

unsigned char   togFg[2]; // [0]:clamp1,clamp2,clamp3,clamp4, [1]:vaccum

unsigned int    testCnt = 0;
unsigned char   testFg = 0;

unsigned int    testROECnt = 0;
unsigned int    testROEPrev = 0;
unsigned long	testROEData = 0;

unsigned int	tSewCnt = 0;
unsigned int	tMovCnt = 0;
unsigned char	tMovFg = 0;
unsigned char	tSewRunFg = 0;
unsigned int	tMovMaxCnt = 0;

unsigned int	looperCnt = 0;
unsigned char	looperFg = 0;

unsigned long	movCnt = 0;
unsigned char	movFg = 0;
unsigned long	movCLK = 0;

unsigned int	lfCnt = 0;
unsigned char	lfFg = 0;
unsigned int	lfCLK = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void cntlTLamp(void);
void cntlVaccum(unsigned char mode);
void cntlClamp(unsigned char sel, unsigned char mode);
void changeFRQ(unsigned char mode);
//void initLifting(void);
void initLifting(unsigned char mode);
void initNeedle(void);
void initLooper(void);
//void initMoving(void);
void initMoving(unsigned char mode);
void initClamp(unsigned char sel, unsigned char mode);
void initMachine(void);
void scanSw(void);
void runningProcess(void);
void testSV(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t ttime);
void testROE(void);
void mTestMode(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void testSV(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t ttime)
{
    if(testFg == 0)
    {
        if(Channel == TIM_CHANNEL_1)
        {
            stOut[0] &= ~0x01; // servo1 ready -> servo1 on
    		outSignal(OUT_CH1,stOut[0]); //servo1 on 
    		
            SV1_DIR_CCW;
        }
        else if(Channel == TIM_CHANNEL_2)
        {
            stOut[1] &= ~0x01; // servo1 ready -> servo1 on
    		outSignal(OUT_CH2,stOut[1]); //servo1 on 
    		
            SV2_DIR_CCW;
        }
        else if(Channel == TIM_CHANNEL_3)
        {
            stOut[2] &= ~0x01; // servo1 ready -> servo1 on
    		outSignal(OUT_CH3,stOut[2]); //servo1 on 
    		
            SV3_DIR_CW;
        }
        else
        {
            stOut[3] &= ~0x01; // servo1 ready -> servo1 on
    		outSignal(OUT_CH4,stOut[3]); //servo1 on 
    		
            SV4_DIR_CCW;
        }
        HAL_TIM_PWM_Start(htim,Channel);

        testCnt = ttime;
        testFg = 1;
    }
    else if(testFg == 2)
    {
        HAL_TIM_PWM_Stop(htim,Channel);
    }
}

void testROE(void)
{
    if(testROEPrev != testROECnt)
    {
        putChar(&huart4,(testROECnt/10000)+0x30);
        putChar(&huart4,(testROECnt/1000)+0x30);
        putChar(&huart4,(testROECnt/100)+0x30);
        putChar(&huart4,(testROECnt/10)+0x30);
        putChar(&huart4,(testROECnt%10)+0x30);
        putChar(&huart4,'\n');

        testROEPrev = testROECnt;
    }
}

/**
  * @brief Tower Lamp control function
  * @param None
  * @retval None
  */
void cntlTLamp(void)
{
    if(tlFg == TLAMP_RUN_NOW)
    {
        if(errorCode[0] & 0x01) // ems error - R:200->RG:200->RGY:200->off:500
        {
            if(tlRGYFg == 1) // R on
            {
                stOut[OUT_CH6] |= OUT_TL_R_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 2;
                
                tlCnt = TLAMP_200MSEC_TCNT;
            }
            else if(tlRGYFg == 2) // RG on
            {
                stOut[OUT_CH6] |= (OUT_TL_R_CNTL|OUT_TL_G_CNTL);
                stOut[OUT_CH6] &= ~OUT_TL_Y_CNTL;

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 3;

                tlCnt = TLAMP_200MSEC_TCNT;
            }
            else if(tlRGYFg == 3) // RG on
            {
                stOut[OUT_CH6] |= (OUT_TL_R_CNTL|OUT_TL_G_CNTL);
                stOut[OUT_CH6] &= ~OUT_TL_Y_CNTL;

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 0;

                tlCnt = TLAMP_200MSEC_TCNT;
            }
            else // off
            {
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 1;

                tlCnt = TLAMP_500MSEC_TCNT;
            }
        }
        else if(errorCode[3] & 0x0f) // servo error - R:200->Y:200
        {
            if(tlRGYFg == 1) // R on
            {
                stOut[OUT_CH6] |= OUT_TL_R_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 2;
            }
            else if(tlRGYFg == 2) // Y on
            {
                stOut[OUT_CH6] |= OUT_TL_Y_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 0;
            }
            else // off
            {
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 1;
            }

            tlCnt = TLAMP_200MSEC_TCNT;
        }
        else if(errorCode[0] & 0x02) // communication error - RGY:200,off:200
        {
            if(tlRGYFg)
            {
                stOut[OUT_CH6] |= (OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 0;
            }
            else
            {
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 1;
            }

            tlCnt = TLAMP_200MSEC_TCNT;
        }
        else if(errorCode[0] & 0x0c) // heater over current/under current or air pressure low - R:500->off:500->R:500->off:1000
        {
            if((tlRGYFg == 1) || (tlRGYFg == 3)) // R on
            {
                if(tlRGYFg == 1)
                {
                    tlRGYFg = 2;
                    tlCnt = TLAMP_500MSEC_TCNT;
                }
                else
                {
                    tlRGYFg = 0;
                    tlCnt = TLAMP_1000MSEC_TCNT;
                }

                stOut[OUT_CH6] |= OUT_TL_R_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);
            }
            else
            {
                if(tlRGYFg == 2) // off
                {
                    tlRGYFg = 3;
                    tlCnt = TLAMP_500MSEC_TCNT;
                }
                else
                {
                    tlRGYFg = 1;
                    tlCnt = TLAMP_1000MSEC_TCNT;
                }

                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);
            }
        }
        else if((errorCode[0] & 0xf0) || (errorCode[1] & 0x0f)) // limit and home sensor error - R:500->off:1000
        {
            if(tlRGYFg)
            {
                stOut[OUT_CH6] |= OUT_TL_R_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 0;

                tlCnt = TLAMP_500MSEC_TCNT;
            }
            else
            {
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 1;

                tlCnt = TLAMP_1000MSEC_TCNT;
            }
        }
        else if((errorCode[1] & 0xc0) || (errorCode[2] & 0x03)) // band lost1,2 and up/down thread error - R:500->off:500->R:500->off:500->R:500->off:1000
        {
            if((tlRGYFg == 1) || (tlRGYFg == 3) || (tlRGYFg == 5)) // R on
            {
                if(tlRGYFg == 1)        tlRGYFg = 2;
                else if(tlRGYFg == 3)   tlRGYFg = 4;
                else                    tlRGYFg = 0;
                
                stOut[OUT_CH6] |= OUT_TL_R_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlCnt = TLAMP_500MSEC_TCNT;
            }
            else
            {
                if(tlRGYFg == 2) // off
                {
                    tlRGYFg = 3;
                    tlCnt = TLAMP_500MSEC_TCNT;
                }
                else if(tlRGYFg == 4) // off
                {
                    tlRGYFg = 5;
                    tlCnt = TLAMP_500MSEC_TCNT;
                }
                else
                {
                    tlRGYFg = 1;
                    tlCnt = TLAMP_1000MSEC_TCNT;
                }

                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);
            }
        }
        else if(errorCode[1] & 0x30) // band empy warning - Y:500->off:500
        {
            if(tlRGYFg)
            {
                stOut[OUT_CH6] |= OUT_TL_Y_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 0;

                tlCnt = TLAMP_500MSEC_TCNT;
            }
            else
            {
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_G_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlRGYFg = 1;

                tlCnt = TLAMP_500MSEC_TCNT;
            }
        }
        else
        {
            if(((errorCode[2] & 0x03) == 0) && ((errorCode[3] & 0x0f) == 0)) // normal - G:on
            {
                stOut[OUT_CH6] |= OUT_TL_G_CNTL;
                stOut[OUT_CH6] &= ~(OUT_TL_R_CNTL|OUT_TL_Y_CNTL);

                outSignal(OUT_CH6,stOut[OUT_CH6]);

                tlCnt = TLAMP_10MSEC_TCNT;
            }
        }
        
        tlFg = TLAMP_RUN_TSTART;
    }
}

/**
  * @brief vaccum control function
  * @param None
  * @retval None
  */
void cntlVaccum(unsigned char mode)
{
	if(mode == VACCUM_OPEN)         stOut[OUT_CH6] &= ~OUT_VACCUM_CNTL; // vaccum open
	else if(mode == VACCUM_CLOSE)   stOut[OUT_CH6] |= OUT_VACCUM_CNTL;
	
	outSignal(OUT_CH6,stOut[OUT_CH6]);
}

/**
  * @brief clamp control function
  * @param None
  * @retval None
  */
void cntlClamp(unsigned char sel, unsigned char mode)
{
    if(mode == CLAMP_OPEN)          stOut[OUT_CH5] |= sel; // clamp open
    else if(mode == CLAMP_CLOSE)    stOut[OUT_CH5] &= ~sel; // clamp close
		
	outSignal(OUT_CH5,stOut[OUT_CH5]);
}

/**
  * @brief Band Cut control function
  * @param None
  * @retval None
  */
void cntlBandCut(void)
{
    if(bcStep == 0) //start
    {
        // initial : motor angle initial -> heater on -> current check -> moving -> time check
        rcDuty[0] = RC_DUTY_MIN;
        rcDuty[1] = RC_DUTY_MAX;

        //stOut[OUT_CH5] |= OUT_HEAT_CNTL; // heater on
        //outSignal(OUT_CH5,stOut[OUT_CH5]);

        heatOnFg = 1;

        HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);    // rc servo1 pwm start
        HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);    // rc servo2 pwm start

		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,rcDuty[0]);  // duty set rc servo1
        __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,rcDuty[1]);  // duty set rc servo2
            
        bcStep = 1;
        //rcInDc = 0; // increse

        bcCnt = RC_GEN_TCNT;
        bcFg = RC_GEN_TSTART;
    }
    else if(bcStep == 1)    // pwm start
    {
        if(bcCnt == RC_GEN_NOW)
        {
            __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,rcDuty[0]);  // duty set rc servo1
            __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,rcDuty[1]);  // duty set rc servo2

            if(rcInDc == 1) // decrease
            {
                rcDuty[0] -= RC_DUTY_STEP;
                rcDuty[1] += RC_DUTY_STEP;
            }
            else // increase
            {
                rcDuty[0] += RC_DUTY_STEP;
                rcDuty[1] -= RC_DUTY_STEP;
            }

            bcCnt = RC_GEN_TCNT;
            bcFg = RC_GEN_TSTART;

            if((rcDuty[0] > RC_DUTY_MAX) && (rcDuty[1] < RC_DUTY_MIN)) // moving top end
            {
            	rcInDc = 1;
            	rcDuty[0] = RC_DUTY_MAX;
                rcDuty[1] = RC_DUTY_MIN;
            }
            else if((rcDuty[0] < RC_DUTY_MIN) && (rcDuty[1] > RC_DUTY_MAX)) // end
            {
                //stOut[OUT_CH5] &= ~OUT_HEAT_CNTL; // heater off
                //outSignal(OUT_CH5,stOut[OUT_CH5]);
                
                heatOnFg = 0;
                
                HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);    // rc servo1 pwm stop
                HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);    // rc servo2 pwm stop
                
                rcDuty[0] = RC_DUTY_MIN;
                rcDuty[1] = RC_DUTY_MAX;
                
                rcInDc = 0;
                
                bcCnt = 0;
                bcFg = 0;

                //bcStep = 2;
                bcStep = 0;
            }
        }
    }
}

/**
  * @brief change pwm frquency function
  * @param None
  * @retval None
  */
void changeFRQ(unsigned char mode)
{
	if(mode == MODE_SEWING)
    {
        TIM4->ARR = VFAST_SEWING;   // 25KHz 일반 동작 모드...
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,VFAST_SEWING/2);
    }
	else if(mode == MODE_TEST)
    {
        TIM4->ARR = VSLOW_SEWING;   // 100Hz 테스트 모드 혹은 머신초기화..
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,VSLOW_SEWING/2);
    }
	else{;}
}

/**
  * @brief lifting home function
  * @param None
  * @retval None
  */
void initLifting(unsigned char mode)
{
	if(initStep[INIT_STEP_LIFTING] == 0) // start
	{
		initStep[INIT_STEP_LIFTING] = 1;

		liftStep = 0;
		liftCnt = 0;	// 미싱 바느질 가능한 위치 이동 카운터 초기화...

		stOut[3] &= ~0x01;
		
		outSignal(OUT_CH4,stOut[3]);

		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
		SV4_DIR_CCW; // lifting ac servo cw : down, ccw : up
		TIM4->ARR = FRQ_1KHz;
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

		initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
		initFg[INIT_STEP_LIFTING] = INIT_TSTART;
	}
	else if(initStep[INIT_STEP_LIFTING] == 1) // 체크 시작...
	{
		if(stSENin & ST_SEN_LIFTING_UP_LIM)	// up limit sensor on
		{
			liftStep = 1;
			
			SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

			initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
			initFg[INIT_STEP_LIFTING] = INIT_TSTART;;
		}
		else if(stSENin & ST_SEN_LIFTING_HOME) // lifting home sensor on -> stop
		{
			if(mode == 0x01)
			{
				if(liftStep == 1)	// up limit sensor 한번 인식하고 옴...
				{
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop

					liftCnt = (675*16);
					svEtiCnt = 0;
					liftStep = 2; // next stage

					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

					initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
					initFg[INIT_STEP_LIFTING] = INIT_TSTART;;
				}
			}
			else
			{
				initStep[INIT_STEP_LIFTING] = 2;

				initFg[INIT_STEP_LIFTING] = 0;
				initCnt[INIT_STEP_LIFTING] = 0;

				liftStep = 0;
				liftCnt = 0;
			}
		}
		else if((stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER))
		{
			initStep[INIT_STEP_LIFTING] = 3;

			initFg[INIT_STEP_LIFTING] = 0;
			initCnt[INIT_STEP_LIFTING] = 0;

			errorCode[0] |= 0x10;

			liftStep = 0;
			liftCnt = 0;
#if 0
#ifdef __DEBUG_SUB__
if(stSENin & ST_SEN_LIFTING_DN_LIM)
	putStr(&huart4,(const unsigned char *)"Lifting init DN LIM\n");
else
	putStr(&huart4,(const unsigned char *)"Lifting init TOV\n");
#endif
#endif
		}
		else
		{
			if(liftStep == 2)
			{
				if(liftCnt == svEtiCnt)
				{
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop

					initStep[INIT_STEP_LIFTING] = 2;

					initFg[INIT_STEP_LIFTING] = 0;
					initCnt[INIT_STEP_LIFTING] = 0;

					liftStep = 0;
					liftCnt = 0;

					svEtiCnt = 0;
#if 0
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"liftCnt == svEtiCnt\n");
#endif
#endif

				}
				else if(liftCnt < svEtiCnt)	// error
				{
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop
					
					initStep[INIT_STEP_LIFTING] = 3;

					initFg[INIT_STEP_LIFTING] = 0;
					initCnt[INIT_STEP_LIFTING] = 0;

					liftStep = 0;
					liftCnt = 0;
#if 0
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"liftCnt < svEtiCnt\n");
#endif
#endif

				}
			}
		}
	}
}

#if 0
void initLifting(unsigned char mode)
{
	if(stSENin & ST_SEN_LIFTING_HOME) // lifting home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop

		errorCode[0] &= ~0x40;
		errorCode[0] &= ~0x10;
			
		if(mode == 0x01)	// sewing machine initializing
		{
			if(liftStep == 1)	// up limit한번 인식하고 홈에 옴....
			{
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting sew init start\n");
#endif
				liftStep = 2; // 비교 해야 한다.
				liftCnt = (690*16);
				svEtiCnt = 0;
				
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	// lifting servo start
			}
		}
		else
		{
			initStep[INIT_STEP_LIFTING] = 2;

			initFg[INIT_STEP_LIFTING] = 0;
			initCnt[INIT_STEP_LIFTING] = 0;
			
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting init OK\n");
#endif
		}
	}
	else // lifting home sensor off
	{
		if(initStep[INIT_STEP_LIFTING] == 0) // start
		{
			initStep[INIT_STEP_LIFTING] = 1;

			liftStep = 0;
			liftCnt = 0;	// 미싱 바느질 가능한 위치 이동 카운터 초기화...

			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
			SV4_DIR_CCW; // lifting ac servo cw : down, ccw : up
			TIM4->ARR = FRQ_1KHz;	// 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

			initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
			initFg[INIT_STEP_LIFTING] = INIT_TSTART;
		}
		else if(initStep[INIT_STEP_LIFTING] == 1)
		{
			if((stSENin & ST_SEN_LIFTING_UP_LIM) || (stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER)) // error : home sensor not working
			//if(initFg[INIT_STEP_LIFTING] == IN0T_TOVER)
			//if((stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER)) // error : home sensor not working
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop
				
				if(stSENin & ST_SEN_LIFTING_UP_LIM)
				{
					SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

					initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
					initFg[INIT_STEP_LIFTING] = INIT_TSTART;

					liftStep = 1; // up limit 한번 인식 함...

#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting init UP LIM\n");
#endif
				}
				else if((stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER))
				{
					initStep[INIT_STEP_LIFTING] = 3;

					initFg[INIT_STEP_LIFTING] = 0;
					initCnt[INIT_STEP_LIFTING] = 0;

					errorCode[0] |= 0x10;

#ifdef __DEBUG_SUB__
if(stSENin & ST_SEN_LIFTING_DN_LIM)
	putStr(&huart4,(const unsigned char *)"Lifting init DN LIM\n");
else
	putStr(&huart4,(const unsigned char *)"Lifting init TOV\n");
#endif
				}
			}
			else	// 타임오버, 업리미트, 다운리미트 모두 아니다.....초기화 위로 이동중임....
			{
				if(mode == 0x01)	// sewing machine initializing
				{
					if(liftStep == 2)	// 초기화 위치로 이동 중인지 완료된것인지 확인
					{
						if(svEtiCnt == liftCnt) // 위치 이동 완료...
						{
							__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
							HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop

							initStep[INIT_STEP_LIFTING] = 2;

							initFg[INIT_STEP_LIFTING] = 0;
							initCnt[INIT_STEP_LIFTING] = 0;

#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting sew init ok\n");
#endif
						}
					}
				}
			}
		}
		else{;}
	}
}
#endif

#if 0
void initLifting(void)
{
	if(stSENin & ST_SEN_LIFTING_HOME) // lifting home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop

		initStep[INIT_STEP_LIFTING] = 2;

		initFg[INIT_STEP_LIFTING] = 0;
		initCnt[INIT_STEP_LIFTING] = 0;

		errorCode[0] &= ~0x40;
		errorCode[0] &= ~0x10;
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting init OK\n");
#endif

	}
	else // lifting home sensor off
	{
		if(initStep[INIT_STEP_LIFTING] == 0) // start
		{
			initStep[INIT_STEP_LIFTING] = 1;

			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
			SV4_DIR_CCW; // lifting ac servo cw : down, ccw : up
			TIM4->ARR = FRQ_1KHz;   // 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

			initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
			initFg[INIT_STEP_LIFTING] = INIT_TSTART;
		}
		else if(initStep[INIT_STEP_LIFTING] == 1)
		{
			if((stSENin & ST_SEN_LIFTING_UP_LIM) || (stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER)) // error : home sensor not working
			//if(initFg[INIT_STEP_LIFTING] == IN0T_TOVER)
			//if((stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER)) // error : home sensor not working
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);	// lifting servo stop
				
				if(stSENin & ST_SEN_LIFTING_UP_LIM)
				{
					SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

					initCnt[INIT_STEP_LIFTING] = INIT_TCNT;
					initFg[INIT_STEP_LIFTING] = INIT_TSTART;

#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Lifting init UP LIM\n");
#endif
				}
				else if((stSENin & ST_SEN_LIFTING_DN_LIM) || (initFg[INIT_STEP_LIFTING] == INIT_TOVER))
				{
					
					initStep[INIT_STEP_LIFTING] = 3;

					initFg[INIT_STEP_LIFTING] = 0;
					initCnt[INIT_STEP_LIFTING] = 0;

					errorCode[0] |= 0x10;

#ifdef __DEBUG_SUB__
if(stSENin & ST_SEN_LIFTING_DN_LIM)
	putStr(&huart4,(const unsigned char *)"Lifting init DN LIM\n");
else
	putStr(&huart4,(const unsigned char *)"Lifting init TOV\n");
#endif
				}
			}
		}
		else{;}
	}
}
#endif

/**
  * @brief initialization NEEDLE function
  * @param None
  * @retval None
  */
void initNeedle(void)
{
	if(stSENin & ST_SEN_NEEDLE_HOME) // needle home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);	// needle servo stop
		//TIM4->ARR = 0;

		initStep[INIT_STEP_NEEDLE] = 2;

		initFg[INIT_STEP_NEEDLE] = 0;
		initCnt[INIT_STEP_NEEDLE] = 0;

		errorCode[0] &= ~0x80;
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Needle init OK\n");
#endif

	}
	else // needle home sensor off
	{
		if(initStep[INIT_STEP_NEEDLE] == 0)
		{
			initStep[INIT_STEP_NEEDLE] = 1;

			stOut[0] &= ~0x01;
			outSignal(OUT_CH1,stOut[0]);
			
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
			SV1_DIR_CCW; // needle ac servo ccw
			TIM4->ARR = FRQ_200Hz;   // 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(FRQ_200Hz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

			initCnt[INIT_STEP_NEEDLE] = INIT_TCNT;
			initFg[INIT_STEP_NEEDLE] = INIT_TSTART;
		}
		else if(initStep[INIT_STEP_NEEDLE] == 1)
		{
			if(initFg[INIT_STEP_NEEDLE] == INIT_TOVER) // error
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);	// needle servo stop

				initStep[INIT_STEP_NEEDLE] = 3;

				initFg[INIT_STEP_NEEDLE] = 0;
				initCnt[INIT_STEP_NEEDLE] = 0;

				errorCode[0] |= 0x80;
                
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Needle init TOV\n");
#endif
			}
		}
		else{;}
	}
}

/**
  * @brief initialization LOOER function
  * @param None
  * @retval None
  */
void initLooper(void)
{
	if(stSENin & ST_SEN_LOOPER_HOME) // needle home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);	// needle servo stop

		initStep[INIT_STEP_LOOPER] = 2;

		initFg[INIT_STEP_LOOPER] = 0;
		initCnt[INIT_STEP_LOOPER] = 0;
		
		errorCode[1] &= ~0x01;
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Looper init OK\n");
#endif

	}
	else // needle home sensor off
	{
		if(initStep[INIT_STEP_LOOPER] == 0)
		{
			initStep[INIT_STEP_LOOPER] = 1;

			stOut[1] &= ~0x01;
			outSignal(OUT_CH2,stOut[1]);

			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
			SV2_DIR_CCW; // looper ac servo ccw
			TIM4->ARR = FRQ_200Hz;   // 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(FRQ_200Hz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

			initCnt[INIT_STEP_LOOPER] = INIT_TCNT;
			initFg[INIT_STEP_LOOPER] = INIT_TSTART;
		}
		else if(initStep[INIT_STEP_LOOPER] == 1)
		{
			if(initFg[INIT_STEP_LOOPER] == INIT_TOVER) // error
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);	// needle servo stop

				initStep[INIT_STEP_LOOPER] = 3;

				initFg[INIT_STEP_LOOPER] = 0;
				initCnt[INIT_STEP_LOOPER] = 0;

				errorCode[1] |= 0x01;
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Looper init TOV\n");
#endif

			}
		}
		else{;}
	}
}

/**
  * @brief initialization Moving function
  * @param None
  * @retval None
  */
void initMoving(unsigned char mode)
{
	if(stSENin & ST_SEN_MOVING_HOME) // moving home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	// needle servo stop

		initStep[INIT_STEP_MOVING] = 2;

		initFg[INIT_STEP_MOVING] = 0;
		initCnt[INIT_STEP_MOVING] = 0;
		
		errorCode[1] &= ~0x08;
		errorCode[1] &= ~0x02; // moving left limit sensor clear
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Moving init OK\n");
#endif
	}
	else // moving home sensor off
	{
		if(initStep[INIT_STEP_MOVING] == 0)
		{
			initStep[INIT_STEP_MOVING] = 1;

			if(mode == 1)
			{
				movCnt = 0;
				movFg = 1;
			}

			stOut[2] &= ~0x01;
			outSignal(OUT_CH3,stOut[2]);
			
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
			SV3_DIR_CW; // moving ac servo cw : backward, ccw : forward
			TIM4->ARR = FRQ_5KHz;   // 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_5KHz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

			initCnt[INIT_STEP_MOVING] = INIT_TCNT;
			initFg[INIT_STEP_MOVING] = INIT_TSTART;
	
//#ifdef __DEBUG_SUB__
//putStr(&huart4,(const unsigned char *)"Moving init start\n");
//#endif

		}
		else if(initStep[INIT_STEP_MOVING] == 1)
		{
			if((stSENin & ST_SEN_MOVING_BACKWARD_LIM) || (initFg[INIT_STEP_MOVING] == INIT_TOVER)) // error : home sensor not working
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	// needle servo stop

				initStep[INIT_STEP_MOVING] = 3;

				initFg[INIT_STEP_MOVING] = 0;
				initCnt[INIT_STEP_MOVING] = 0;

				errorCode[1] |= 0x08;

				if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)    errorCode[1] |= 0x02; // moving left limit sensor clear

//#ifdef __DEBUG_SUB__
//putStr(&huart4,(const unsigned char *)"Moving init TOV\n");
//#endif
			}

			if(mode == 1)
			{
				//if(movCnt == 85254) // 1400mm 이동....속도 줄이고 리프팅도 같이 시작
				if(movCnt == 92000) // 1400mm 이동....속도 줄이고 리프팅도 같이 시작
				{
					movFg = 2;
					movCnt = 0;
				}
			}
		}
		else{;}
	}
}

#if 0
void initMoving(void)
{
	if(stSENin & ST_SEN_MOVING_HOME) // moving home sensor on -> stop
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	// needle servo stop

		initStep[INIT_STEP_MOVING] = 2;

		initFg[INIT_STEP_MOVING] = 0;
		initCnt[INIT_STEP_MOVING] = 0;
		
		errorCode[1] &= ~0x08;
		errorCode[1] &= ~0x02; // moving left limit sensor clear
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"Moving init OK\n");
#endif
	}
	else // moving home sensor off
	{
		if(initStep[INIT_STEP_MOVING] == 0)
		{
			initStep[INIT_STEP_MOVING] = 1;

			stOut[2] &= ~0x01;
			outSignal(OUT_CH3,stOut[2]);
			
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
			SV3_DIR_CW; // moving ac servo cw : backward, ccw : forward
			TIM4->ARR = FRQ_5KHz;   // 25KHz 일반 동작 모드...
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_5KHz+1)/2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

			initCnt[INIT_STEP_MOVING] = INIT_TCNT;
			initFg[INIT_STEP_MOVING] = INIT_TSTART;
			
//#ifdef __DEBUG_SUB__
//putStr(&huart4,(const unsigned char *)"Moving init start\n");
//#endif

		}
		else if(initStep[INIT_STEP_MOVING] == 1)
		{
			if((stSENin & ST_SEN_MOVING_BACKWARD_LIM) || (initFg[INIT_STEP_MOVING] == INIT_TOVER)) // error : home sensor not working
			{
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	// needle servo stop

				initStep[INIT_STEP_MOVING] = 3;

				initFg[INIT_STEP_MOVING] = 0;
				initCnt[INIT_STEP_MOVING] = 0;

				errorCode[1] |= 0x08;

				if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)    errorCode[1] |= 0x02; // moving left limit sensor clear

//#ifdef __DEBUG_SUB__
//putStr(&huart4,(const unsigned char *)"Moving init TOV\n");
//#endif
			}
		}
		else{;}
	}
}
#endif

/**
  * @brief initialization clamp function
  * @param None
  * @retval None
  */
void initClamp(unsigned char sel, unsigned char mode)
{
    cntlClamp(sel,mode);
    
    if(mode == CLAMP_OPEN) // clamp open
    {
        if((stSENin & ST_SEN_CLAMP12_OPEN) && (stSENin & ST_SEN_CLAMP34_OPEN)
		&& (!(stSENin & ST_SEN_CLAMP12_CLOSE)) && (!(stSENin & ST_SEN_CLAMP34_CLOSE))) // clamp close ok
		{
            initStep[INIT_STEP_CLAMP] = 2;

    		initFg[INIT_STEP_CLAMP] = 0;
    		initCnt[INIT_STEP_CLAMP] = 0;
        }
        else // moving home sensor off
    	{
    		if(initStep[INIT_STEP_CLAMP] == 0) // 최초..
    		{
    			initStep[INIT_STEP_CLAMP] = 1;
    			
    			initCnt[INIT_STEP_CLAMP] = INIT_TCNT;
    			initFg[INIT_STEP_CLAMP] = INIT_TSTART;
    		}
    		else if(initStep[INIT_STEP_CLAMP] == 1) // 상태 확인 대기..
    		{
    			if(initFg[INIT_STEP_CLAMP] == INIT_TOVER) // 
    			{
    				initStep[INIT_STEP_CLAMP] = 3;

    				initFg[INIT_STEP_CLAMP] = 0;
    				initCnt[INIT_STEP_CLAMP] = 0;
    			}
    		}
    		else{;}
    	}
    }
    else // clamp close
    {
        if((stSENin & ST_SEN_CLAMP12_CLOSE) && (stSENin & ST_SEN_CLAMP34_CLOSE)
		&& (!(stSENin & ST_SEN_CLAMP12_OPEN)) && (!(stSENin & ST_SEN_CLAMP34_OPEN))) // clamp close ok
		{
            initStep[INIT_STEP_CLAMP] = 2;

    		initFg[INIT_STEP_CLAMP] = 0;
    		initCnt[INIT_STEP_CLAMP] = 0;
        }
        else // moving home sensor off
    	{
    		if(initStep[INIT_STEP_CLAMP] == 0) // 최초..
    		{
    			initStep[INIT_STEP_CLAMP] = 1;
    			
    			initCnt[INIT_STEP_CLAMP] = INIT_TCNT;
    			initFg[INIT_STEP_CLAMP] = INIT_TSTART;
    		}
    		else if(initStep[INIT_STEP_CLAMP] == 1)
    		{
    			if(initFg[INIT_STEP_CLAMP] == INIT_TOVER) // 상태 확인 대기..
    			{
    				initStep[INIT_STEP_CLAMP] = 3;

    				initFg[INIT_STEP_CLAMP] = 0;
    				initCnt[INIT_STEP_CLAMP] = 0;
    			}
    		}
    		else{;}
    	}
    }
}

/**
  * @brief initialization machine function
  * @param None
  * @retval None
  */
void initMachine(void)
{
    sewAct = SEW_ACT_BR;

	if(stStep == 0x00) // initialization start
	{
	    initFin = 0;
        
		//changeFRQ(MODE_TEST); // pwm speed change -> 100Hz
		//cntlVaccum(VACCUM_OPEN);
								
		initStep[INIT_STEP_LIFTING] = 0;
		initStep[INIT_STEP_NEEDLE] = 0;
		initStep[INIT_STEP_LOOPER] = 0;
		initStep[INIT_STEP_MOVING] = 0;
        initStep[INIT_STEP_CLAMP] = 0;

		stStep = 1;
		stCkCnt = 0;
		stCkFg = 0;
	}
	else if(stStep == 1) // clamp open check
	{
	    stPrevStep = 1;

        // clamp open check
        initClamp(OUT_CLAMP1_CNTL|OUT_CLAMP2_CNTL|OUT_CLAMP3_CNTL|OUT_CLAMP4_CNTL, CLAMP_OPEN);

        if(initStep[INIT_STEP_CLAMP] == 2)
		{
			stStep = 2; // next

            initStep[INIT_STEP_CLAMP] = 0; 
		}
		else if(initStep[INIT_STEP_CLAMP] == 3)
		{
			stStep = 10; // clamp open error

            initStep[INIT_STEP_CLAMP] = 0;
		}
	}
	else if(stStep == 2) // lifting home
	{
	    stPrevStep = 2;
        
		// lifting home check
		initLifting(0);

		if(initStep[INIT_STEP_LIFTING] == 2)
		{
			stStep = 3; // next

            initStep[INIT_STEP_LIFTING] = 0; 
		}
		else if(initStep[INIT_STEP_LIFTING] == 3)
		{
			stStep = 11; // lifting up sensor error

            initStep[INIT_STEP_LIFTING] = 0;
		}
	}
	else if(stStep == 3) // needle, looper, moving home
	{
	    stPrevStep = 3;
        
		// needle home
		initNeedle();
		// looper home
		initLooper();
		// moving home
		//initMoving();

		//if((initStep[INIT_STEP_NEEDLE] == 2) && (initStep[INIT_STEP_LOOPER] == 2)
        //&& (initStep[INIT_STEP_MOVING] == 2)) // all ok
        if((initStep[INIT_STEP_NEEDLE] == 2) && (initStep[INIT_STEP_LOOPER] == 2))
		{
			stStep = 4;
            
            stCkCnt = INIT_TCNT;
			stCkFg = INIT_TSTART;
		}
		else
		{
		    // stStep = 12 : needle home error, stStep = 13 : looper home error, stStep = 15 : moving home error
			if(initStep[INIT_STEP_NEEDLE] == 3) stStep = 12; // needle home sensor error

            if(initStep[INIT_STEP_LOOPER] == 3) stStep = 13; // looper home sensor error

            //if(initStep[INIT_STEP_MOVING] == 3) stStep = 14; // moving home sensor error
		}
	}
	else if(stStep == 4)
	{
		stPrevStep = 4;

		initMoving(0);

		if(initStep[INIT_STEP_MOVING] == 2)
        {
			stStep = 5;
            
            stCkCnt = INIT_TCNT;
			stCkFg = INIT_TSTART;
		}
		else
		{
		    // stStep = 12 : needle home error, stStep = 13 : looper home error, stStep = 15 : moving home error
			if(initStep[INIT_STEP_MOVING] == 3) stStep = 14; // moving home sensor error
		}
	}
    else if(stStep == 5) // band insert check
    {
        stPrevStep = 5;

        if((stSENin & ST_SEN_BAND_INSERT1) && (stSENin & ST_SEN_BAND_INSERT2)) // insert ok
        {
            stStep = 6;

            initStep[INIT_STEP_LIFTING] = 0;
    		initStep[INIT_STEP_NEEDLE] = 0;
    		initStep[INIT_STEP_LOOPER] = 0;
    		initStep[INIT_STEP_MOVING] = 0;
            initStep[INIT_STEP_CLAMP] = 0;

    		stStep = 1;
    		stCkCnt = 0;
    		stCkFg = 0;
        }
        else // band inser error
        {
            stStep = 16;
        }
    }
    else if(stStep == 6) // clamp close and check
    {
         stPrevStep = 6;

        // clamp open check
        initClamp(OUT_CLAMP1_CNTL|OUT_CLAMP2_CNTL|OUT_CLAMP3_CNTL|OUT_CLAMP4_CNTL, CLAMP_CLOSE);

        if(initStep[INIT_STEP_CLAMP] == 2)
		{
			stStep = 6; // end

            initStep[INIT_STEP_LIFTING] = 0;
    		initStep[INIT_STEP_NEEDLE] = 0;
    		initStep[INIT_STEP_LOOPER] = 0;
    		initStep[INIT_STEP_MOVING] = 0;
            initStep[INIT_STEP_CLAMP] = 0;

    		stStep = 0;
    		stCkCnt = 0;
    		stCkFg = 0;
		}
		else if(initStep[INIT_STEP_CLAMP] == 3)
		{
			stStep = 10; // clamp open error
		}
    }
	else{;} // 0x10 error occur
}

/**
  * @brief switch scan function
  * @param None
  * @retval None
  */	
void scanSw(void)
{
	if(sysStartFg)
	{
		if(swFg == SW_SCAN_NOW)	// now switch input check(per 10msec)
		{
			swCur = stSWin; // switch buffer

			if(swCur)	// set bit means switch on
			{
				if(swCur != swPrev) // previous switch input and current switch input is different
				{
					swPrev = swCur;	// set preivous switch to current switch

					swCur &= 0;

					swChkFg = 1;
				}
				else
				{
					if(swChkFg == 1)
					{
						swChkFg = 2;

						// select switch
						if(swCur & ST_SW_EMS)  
							sysMode |= MODE_EMS; // ems mode
						else
						{
							sysMode &= ~MODE_EMS;
							
							if(SW_AUTO_MANUAL)
							{
								sysMode |= MODE_MANUAL;	// manual mode
								sysMode &= ~MODE_AUTO;	// auto mode
							}
							else
							{
								sysMode |= MODE_AUTO;	// auto mode
								sysMode &= ~MODE_MANUAL;	// manual mode
							}
							
							if(swCur & ST_SW_TEST_SEWING)	sysMode |= MODE_MTEST;	// test mode
							//else							sysMode &= ~MODE_MTEST;

							if(sysMode & MODE_MANUAL)
							{
								/*
								if(swCur & ST_SW_MOVING_HOME) initMode |= 0x01;
								//else                        initMode &= ~0x01;

								if(swCur & ST_SW_UD_HOME)   initMode |= 0x02;
								//else                      initMode &= ~0x02;

								if(swCur & ST_SW_NEEDLE_HOME)   initMode |= 0x04;
								//else                          initMode &= ~0x04;

								if(swCur & ST_SW_LOOPER_HOME)   initMode |= 0x08;
								//else                          initMode &= ~0x08;


								if(swCur & ST_SW_ROE_X)   roeMode |= 0x01;
								else                      roeMode &= ~0x01;

								if(swCur & ST_SW_ROE_Y) roeMode |= 0x02;
								else                    roeMode &= ~0x02;

								if(swCur & ST_SW_ROE_Z) roeMode |= 0x04;
								else                    roeMode &= ~0x04;

								if(swCur & ST_SW_ROE_X1)   roeMode |= 0x20;
								else                       roeMode &= ~0x20;

								if(swCur & ST_SW_ROE_X10)   roeMode |= 0x40;
								else                        roeMode &= ~0x40;
								*/
								if(swCur & ST_SW_ROE_X)	// FB
								{
									roeMode |= 0x01; // roe mode이면 init mode는 안됨....
									initMode &= ~0x01;

									//if(changeModeFg == 0)	changeModeFg = 1; // init mode는 일단 정지....
								}
								else
								{
									if(swCur & ST_SW_MOVING_HOME)
										initMode |= 0x01;

									roeMode &= ~0x01; // init mode이면 roe mode는 안됨....
									roeRunFg = 0;
									//changeModeFg = 0;
								}

								if(swCur & ST_SW_ROE_Y)	// UD
								{
									roeMode |= 0x02; // roe mode이면 inint mode는 안됨....
									initMode &= ~0x02;

									//if(changeModeFg == 0)	changeModeFg = 1; // init mode는 일단 정지....
								}
								else
								{
									if(swCur & ST_SW_UD_HOME)
										initMode |= 0x02;
									
									roeMode &= ~0x02; // init mode이면 roe mode는 안됨....
									roeRunFg = 0;
									//changeModeFg = 0;
								}
								/*
								if(swCur & ST_SW_ROE_Z)	// needle
								{
									roeMode |= 0x04; // roe mode이면 inint mode는 안됨....
									initMode &= ~0x04;

									//if(changeModeFg == 0)	changeModeFg = 1; // init mode는 일단 정지....
								}
								else
								{
									if(swCur & ST_SW_UD_HOME)
										initMode |= 0x04;

									roeMode &= ~0x04; // init mode이면 roe mode는 안됨....
									roeRunFg = 0;
									//changeModeFg = 0;
								}
								
								if(swCur & ST_SW_ROE_4)	// LOOPER
								{
									roeMode |= 0x08; // roe mode이면 inint mode는 안됨....
									initMode &= ~0x08;

									//if(changeModeFg == 0)	changeModeFg = 1; // init mode는 일단 정지....
								}
								else
								{
									if(swCur & ST_SW_UD_HOME)
										initMode |= 0x08;

									roeMode &= ~0x08; // init mode이면 roe mode는 안됨....
									roeRunFg = 0;
									//changeModeFg = 0;
								}
								*/

								if(swCur & ST_SW_NEEDLE_HOME)	initMode |= 0x04;
                                //else                          initMode &= ~0x04;
                                
                                if(swCur & ST_SW_LOOPER_HOME)	initMode |= 0x08;
								//else                          initMode &= ~0x08;

                                if(swCur & ST_SW_ROE_X1)   roeMode |= 0x20;
                                else                       roeMode &= ~0x20;

                                if(swCur & ST_SW_ROE_X10)   roeMode |= 0x40;
                                else                        roeMode &= ~0x40;

                                // push swutch
             					if(swCur & ST_SW_CLAMP1_OPEN_CLOSE) swStFg |= ST_SW_CLAMP1_OPEN_CLOSE;	// clamp1 open/close
            					else                                swStFg &= ~ST_SW_CLAMP1_OPEN_CLOSE;
                                
                                if(swCur & ST_SW_CLAMP2_OPEN_CLOSE) swStFg |= ST_SW_CLAMP2_OPEN_CLOSE;	// clamp2 open/close
            					else                                swStFg &= ~ST_SW_CLAMP2_OPEN_CLOSE;	// clamp2 open/close
            					
            					if(swCur & ST_SW_CLAMP3_OPEN_CLOSE) swStFg |= ST_SW_CLAMP3_OPEN_CLOSE;	// clamp3 open/close
            					else                                swStFg &= ~ST_SW_CLAMP3_OPEN_CLOSE;
                                
                                if(swCur & ST_SW_CLAMP4_OPEN_CLOSE) swStFg |= ST_SW_CLAMP4_OPEN_CLOSE;	// clamp4 open/close
            					else                                swStFg &= ~ST_SW_CLAMP4_OPEN_CLOSE;
                                
                                if(swCur & ST_SW_VACCUM_OPEN_CLOSE) swStFg |= ST_SW_VACCUM_OPEN_CLOSE;	// vaccum open/close
            					else                                swStFg &= ~ST_SW_VACCUM_OPEN_CLOSE;	//vaccum open
                                
                                if(swCur & ST_SW_BAND_CUT)	swStFg |= ST_SW_BAND_CUT;	// band cut
                                //else						swStFg &= ~ST_SW_BAND_CUT;	// band cut

								if(swCur & ST_SW_TEST_SEWING)	sysMode |= MODE_MTEST;	// test mode
								//else							sysMode &= ~MODE_MTEST;
                            }
                            else if(sysMode & MODE_AUTO)
                            {
                                if(swCur & ST_SW_AUTO_STOP)
                                {
                                    swStFg |= ST_SW_AUTO_STOP;
                                    //swStFg &= ~ST_SW_AUTO_START;
                                    //sewRunFg = 0;
                                }
                                //else                            swStFg &= ~ST_SW_AUTO_STOP;

                                if(swCur & ST_SW_AUTO_START)
                                {
  
                                    swStFg |= ST_SW_AUTO_START;
                                    //swStFg &= ~ST_SW_AUTO_STOP;
                                    if(sewRunFg == 0)   sewRunFg = 1;
putChar(&huart4,(sewRunFg/100)+0x30);
putChar(&huart4,((sewRunFg%100)/10)+0x30);
putChar(&huart4,(sewRunFg%10)+0x30);

                                }
                                //else                            swStFg &= ~ST_SW_AUTO_START;
                            }
                        }
                    }
					else if(swChkFg == 2)
					{
						if(swCur & ST_SW_TEST_SEWING)	sysMode |= MODE_MTEST;	// test mode
					}
                }
			}
			else	// switch is not on
			{
				swPrev = 0;
				swCur = 0;
			    //swStFg = 0;
				swChkFg = 0;
			}
    		
			swCnt = SW_SCAN_TCNT;
			swFg = SW_SCAN_TSTART;
    	}
    }
}

/**
  * @brief moving/lifting/needle/looper function
  *			현재 위치를 기준(60000)으로 cw증가 ccw감소한 데이터를 이동 거리로 연산하여 동작
  * @param None
  * @retval None
  */	
void cntlRoeRun(void)
{
	unsigned char st=0;
	signed long mcnt=0;
	float cnt=0;

	if(roeRunFg == 0) // 1st : ready status
	{
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"ROE start\n");
#endif
		roeRunFg = 1; // for next step

		svRoeCnt = 0;
		svEtiCnt = 0;
		roeNowCnt = ROE_CLK_REF_CNT;
		roePrevCnt = ROE_CLK_REF_CNT;
		roeEvent = 0;
		roeDirFg = 0;
		frqBuf = 0;

		initMode = 0;

		initStep[INIT_STEP_NEEDLE] = 0;
		initStep[INIT_STEP_LOOPER] = 0;
		initStep[INIT_STEP_MOVING] = 0;
		initStep[INIT_STEP_LIFTING] = 0;
		
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);

		stOut[2] &= ~0x01;
		stOut[3] &= ~0x01;
		outSignal(OUT_CH3,stOut[2]);
		outSignal(OUT_CH4,stOut[3]);
		
		if(roeMode & 0x01) // X : forward/backward
		{
			frqBuf = FRQ_25KHz;
			TIM4->ARR = frqBuf;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0); // pwm gen off
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"ROE FB start\n");
#endif
		}
		else if(roeMode & 0x02) // Y : up/down
		{
			frqBuf = FRQ_5KHz;
			TIM4->ARR = frqBuf;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0); // pwm gen off
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"ROE UD start\n");
#endif
		}
	}
	else if(roeRunFg == 1) // 2nd : set run cw
	{
		if(roeNowCnt != roePrevCnt) // rotary encoder rounding
		{
			if(roeNowCnt > roePrevCnt)	// cw
			{
				if(roeMode & 0x01)	// forward/backward
				{
					if(stSENin & ST_SEN_MOVING_FORWARD_LIM)
						st = 1; // sensor not detect - forward limit
				}
				else if(roeMode & 0x02)	// up/down
				{
					if(stSENin & ST_SEN_LIFTING_UP_LIM)
						st = 1;	// sensor not detect - up limit
				}

				if(st == 0)
				{
					//if(roeEvent != roeDirFg) // 방향 전환이 이루어짐..
					//{
#if 0
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"CW\n");
#endif
#endif

						//roeDirFg = roeEvent;
						
						if(roeMode & 0x01)
						{
							//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
							SV3_DIR_CCW; // forward
						}
						else if(roeMode & 0x02)
						{
							//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
							SV4_DIR_CCW; // up
						}
					//}

					mcnt = roeNowCnt - roePrevCnt; // 입력된 클락 수 저장

					if(roeMode & 0x01)	//FB
					{
						if(roeMode & 0x20) // x1
							//cnt = (((float)mcnt*1)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*1)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
						else if(roeMode & 0x40) // x10
							//cnt = (((float)mcnt*10)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 10mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*10)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 10mm)/ONE_PULSE_MOV)
						else
							//cnt = (((float)mcnt*1)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*1)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
					}
					else if(roeMode & 0x02)	//UD
					{
						if(roeMode & 0x20) // x1
							cnt = mcnt*16;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
						//else if(roeMode & 0x40) // x10
							//cnt = mcnt*160;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
						else
							cnt = mcnt*16;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
					}
					
					svRoeCnt += (unsigned long)cnt;	// clock 추가
#if 0
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"CW:");
putChar(&huart4,(svRoeCnt/100000)+0x30);
putChar(&huart4,((svRoeCnt%100000)/10000)+0x30);
putChar(&huart4,((svRoeCnt%10000)/1000)+0x30);
putChar(&huart4,((svRoeCnt%1000)/100)+0x30);
putChar(&huart4,((svRoeCnt%100)/10)+0x30);
putChar(&huart4,(svRoeCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"roePrevCnt:");
putChar(&huart4,(roePrevCnt/100000)+0x30);
putChar(&huart4,((roePrevCnt%100000)/10000)+0x30);
putChar(&huart4,((roePrevCnt%10000)/1000)+0x30);
putChar(&huart4,((roePrevCnt%1000)/100)+0x30);
putChar(&huart4,((roePrevCnt%100)/10)+0x30);
putChar(&huart4,(roePrevCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"roeNowCnt:");
putChar(&huart4,(roeNowCnt/100000)+0x30);
putChar(&huart4,((roeNowCnt%100000)/10000)+0x30);
putChar(&huart4,((roeNowCnt%10000)/1000)+0x30);
putChar(&huart4,((roeNowCnt%1000)/100)+0x30);
putChar(&huart4,((roeNowCnt%100)/10)+0x30);
putChar(&huart4,(roeNowCnt%10)+0x30);
putChar(&huart4,'\n');
#endif
#endif
					roePrevCnt = roeNowCnt;
				}
				else
				{
					roeNowCnt = roePrevCnt;	// 원복
				}
			}
			else if(roeNowCnt < roePrevCnt) // ccw
			{
				if(roeMode & 0x01)	// forward/backward
				{
					if(stSENin & ST_SEN_MOVING_BACKWARD_LIM)
						st = 1; // sensor detect - backward limit
				}
				else if(roeMode & 0x02)	// up/down
				{
					if(stSENin & ST_SEN_LIFTING_DN_LIM)
						st = 1;	// sensor detect - down limit
				}

				if(st == 0)
				{
					//if(roeEvent != roeDirFg) // 방향 전환이 이루어짐..
					//{
#if 0
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"CCW\n");
#endif
#endif

						//roeDirFg = roeEvent;
						
						if(roeMode & 0x01)
						{
							//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
							SV3_DIR_CW; // backward
						}
						else if(roeMode & 0x02)
						{
							//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
							SV4_DIR_CW; // down
						}
						//else{;}
					//}

					mcnt = roePrevCnt - roeNowCnt; // 입력된 클락 수 저장

					if(roeMode & 0x01)	//FB
					{
						if(roeMode & 0x20) // x1
							//cnt = (((float)mcnt*1)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*1)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
						else if(roeMode & 0x40) // x10
							//cnt = (((float)mcnt*10)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 10mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*10)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 10mm)/ONE_PULSE_MOV)
						else
							//cnt = (((float)mcnt*1)*ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
							cnt = (((float)mcnt*1)/ONE_PULSE_MV); // total ac servo clock counter : ((mcnt * 1mm)/ONE_PULSE_MOV)
					}
					else if(roeMode & 0x02)	//UD
					{
						if(roeMode & 0x20) // x1
							cnt = mcnt*16;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
						//else if(roeMode & 0x40) // x10
							//cnt = mcnt*160;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
						else
							cnt = mcnt*16;	// 5mm pitch : 800CLK, 1mm : 800/5 = 160CLK, 0.1mm : 16CLK
					}

					svRoeCnt += (unsigned long)cnt;
#if 0
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"CCW:");
putChar(&huart4,(svRoeCnt/100000)+0x30);
putChar(&huart4,((svRoeCnt%100000)/10000)+0x30);
putChar(&huart4,((svRoeCnt%10000)/1000)+0x30);
putChar(&huart4,((svRoeCnt%1000)/100)+0x30);
putChar(&huart4,((svRoeCnt%100)/10)+0x30);
putChar(&huart4,(svRoeCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"roePrevCnt:");
putChar(&huart4,(roePrevCnt/100000)+0x30);
putChar(&huart4,((roePrevCnt%100000)/10000)+0x30);
putChar(&huart4,((roePrevCnt%10000)/1000)+0x30);
putChar(&huart4,((roePrevCnt%1000)/100)+0x30);
putChar(&huart4,((roePrevCnt%100)/10)+0x30);
putChar(&huart4,(roePrevCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"roeNowCnt:");
putChar(&huart4,(roeNowCnt/100000)+0x30);
putChar(&huart4,((roeNowCnt%100000)/10000)+0x30);
putChar(&huart4,((roeNowCnt%10000)/1000)+0x30);
putChar(&huart4,((roeNowCnt%1000)/100)+0x30);
putChar(&huart4,((roeNowCnt%100)/10)+0x30);
putChar(&huart4,(roeNowCnt%10)+0x30);
putChar(&huart4,'\n');
#endif
#endif

					roePrevCnt = roeNowCnt;	//원복
				}
				else
				{
					roeNowCnt = roePrevCnt;
				}
			}
			else{;}	// roeNowCnt == roePrevCnt
		}
		else // no input encoder
		{
			//svRoeCnt = 0;
			//svEtiCnt = 0; // over flow 방지
			roeNowCnt = ROE_CLK_REF_CNT;
			roePrevCnt = ROE_CLK_REF_CNT;
		}

		if(svRoeCnt > svEtiCnt)
		{
			if(roeMode & 0x01)
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(frqBuf+1)/2);
			else if(roeMode & 0x02)
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(frqBuf+1)/2);
		}
		else
		{
			if(roeMode & 0x01)
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			else if(roeMode & 0x02)
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);

			//svRoeCnt = 0;
			//svEtiCnt = 0; // over flow 방지

			roeNowCnt = ROE_CLK_REF_CNT;
			roePrevCnt = ROE_CLK_REF_CNT;

#if 1
#ifdef	__DEBUG_SUB__
if(testROEData != svRoeCnt)
{
putStr(&huart4,(const unsigned char *)"Roe:");
putChar(&huart4,(svRoeCnt/100000)+0x30);
putChar(&huart4,((svRoeCnt%100000)/10000)+0x30);
putChar(&huart4,((svRoeCnt%10000)/1000)+0x30);
putChar(&huart4,((svRoeCnt%1000)/100)+0x30);
putChar(&huart4,((svRoeCnt%100)/10)+0x30);
putChar(&huart4,(svRoeCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"Eti:");
putChar(&huart4,(svEtiCnt/100000)+0x30);
putChar(&huart4,((svEtiCnt%100000)/10000)+0x30);
putChar(&huart4,((svEtiCnt%10000)/1000)+0x30);
putChar(&huart4,((svEtiCnt%1000)/100)+0x30);
putChar(&huart4,((svEtiCnt%100)/10)+0x30);
putChar(&huart4,(svEtiCnt%10)+0x30);
putChar(&huart4,'\n');

testROEData = svRoeCnt;
}
#endif
#endif
		}
	}
}

/**
  * @brief manual mode function
  * @param None
  * @retval None
  */	
void mTestMode(void)
{
	if(tSewRunFg == 0)	// initializing test sewing mode
	{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"MTS srart\n");
#endif
		if((!(stSENin & ST_SEN_NEEDLE_HOME))|| (!(stSENin & ST_SEN_LOOPER_HOME)))	// forward sen on or needle sen off or looper sen off
		{
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);	// needle stop
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	// looper stop
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);	// mov stop
			
			tSewRunFg = 0;
			tMovFg = 0;
			tSewCnt = 0;
			tMovCnt = 0;
			tMovMaxCnt = 0;

			sysMode &= ~MODE_MTEST;
			
#ifdef	__DEBUG_SUB__
if(stSENin & ST_SEN_MOVING_FORWARD_LIM)
putStr(&huart4,(const unsigned char *)"FLS on\n");
if(!(stSENin & ST_SEN_NEEDLE_HOME))
putStr(&huart4,(const unsigned char *)"NHS off\n");
if(!(stSENin & ST_SEN_LOOPER_HOME))
putStr(&huart4,(const unsigned char *)"LHS off\n");
#endif
		}
		else
		{
			tSewRunFg = 1;

			tMovFg = 0;
			tSewCnt = 0;
			tMovCnt = 0;
			tMovMaxCnt = 0;

			stOut[0] &= ~0x01;
			stOut[1] &= ~0x01;
			stOut[2] &= ~0x01;
			outSignal(OUT_CH1,stOut[0]);
			outSignal(OUT_CH2,stOut[1]);
			outSignal(OUT_CH3,stOut[2]);

			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);	// needle stop
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	// looper stop
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);	// mov stop
			
			TIM4->ARR = FRQ_200Hz;	// PWM frequency change 200Hz
			
			SV1_DIR_CCW;	// needle direction ccw set
			SV2_DIR_CCW;	// looper direction ccw set
			SV3_DIR_CCW;	// moving direction forward set

			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(FRQ_200Hz+1)/2);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(FRQ_200Hz+1)/2);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_200Hz+1)/2);
			
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);	// needle start
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);	// looper start
		}
	}
	else if(tSewRunFg == 1)	// start sewing machine
	{
		if(tMovCnt == MOV_1CYCLE_PULSE)
		{
			tMovFg = 0;

			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
			
			if(tSewCnt == SEW_1CYCLE_PULSE)
			{
				
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1); // needle stop
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2); // looper stop
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"tmmc:");
putChar(&huart4,(tMovMaxCnt/1000)+0x30);
putChar(&huart4,((tMovMaxCnt%1000)/100)+0x30);
putChar(&huart4,((tMovMaxCnt%100)/10)+0x30);
putChar(&huart4,(tMovMaxCnt%10)+0x30);
putChar(&huart4,'\n');
putStr(&huart4,(const unsigned char *)"tsc:");
putChar(&huart4,(tSewCnt/1000)+0x30);
putChar(&huart4,((tSewCnt%1000)/100)+0x30);
putChar(&huart4,((tSewCnt%100)/10)+0x30);
putChar(&huart4,(tSewCnt%10)+0x30);
putChar(&huart4,'\n');
#endif
				tSewCnt = 0;	// counter clear
						
				tSewRunFg = 0;
				tMovFg = 0;
				tMovCnt = 0;
				tMovMaxCnt = 0;

				sysMode &= ~MODE_MTEST;

#ifdef	__DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"MTS end\n");
#endif
			}
		}
		else
		{
			if(tSewCnt == SEW_1CYCLE_PULSE)	// 1 cycle moving
			{
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);	// needle stop
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	// looper stop
			}
			else if(tSewCnt == SEW_OFF_FABRIC_PULSE)	// moving now
			{
				//tSewCnt = 0;
				tMovCnt = 0;
				
				if(tMovFg == 0)
				{
					tMovFg = 1;

					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);	// moving start
				}
			}
			else{;}

			/*
			if(tMovFg == 1)	// moving check
			{
				if(tMovCnt == MOV_1CYCLE_PULSE)	// moving motor 1 cycle running
				{
					tMovCnt = 0;
					tMovFg = 0;

					HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
				}
			}
			else{;}
			*/
		}
	}
}


/**
  * @brief manual mode function
  * @param None
  * @retval None
  */	
void manualMode(void)
{
	if(SW_CLAMP1_OPEN_CLOSE)	cntlClamp(OUT_CLAMP1_CNTL,CLAMP_CLOSE); // clamp1 close
	else									cntlClamp(OUT_CLAMP1_CNTL,CLAMP_OPEN); // clamp1 open
	    
	if(SW_CLAMP2_OPEN_CLOSE)	cntlClamp(OUT_CLAMP2_CNTL,CLAMP_CLOSE); // clamp1 close
	else									cntlClamp(OUT_CLAMP2_CNTL,CLAMP_OPEN); // clamp1 open

	if(SW_CLAMP3_OPEN_CLOSE)	cntlClamp(OUT_CLAMP3_CNTL,CLAMP_CLOSE); // clamp1 close
	else									cntlClamp(OUT_CLAMP3_CNTL,CLAMP_OPEN); // clamp1 open

	if(SW_CLAMP4_OPEN_CLOSE)	cntlClamp(OUT_CLAMP4_CNTL,CLAMP_CLOSE); // clamp1 close
	else									cntlClamp(OUT_CLAMP4_CNTL,CLAMP_OPEN); // clamp1 open
	            
	if(SW_VACCUM_OPEN_CLOSE)	cntlVaccum(VACCUM_OPEN); // vaccum open
	else									cntlVaccum(VACCUM_CLOSE); // vaccum close

	if(roeMode & 0x1f)
	{
		cntlRoeRun();
	}
	else
	{
		if(initMode & 0x01)
		{
			initMoving(0);

			if((initStep[INIT_STEP_MOVING] == 2) || (initStep[INIT_STEP_MOVING] == 3))
			{
				initMode &= ~0x01;
				initStep[INIT_STEP_MOVING] = 0;
			}
		}

		if(initMode & 0x02)
		{
			initLifting(1);

			if((initStep[INIT_STEP_LIFTING] == 2) || (initStep[INIT_STEP_LIFTING] == 3))
			{
				initMode &= ~0x02;
				initStep[INIT_STEP_LIFTING] = 0;
			}
		}

		if(initMode & 0x04)
		{
			initNeedle();

			if((initStep[INIT_STEP_NEEDLE] == 2) || (initStep[INIT_STEP_NEEDLE] == 3))
			{
				initMode &= ~0x04;
				initStep[INIT_STEP_NEEDLE] = 0;
			}
		}

		if(initMode & 0x08)
		{
			initLooper();

			if((initStep[INIT_STEP_LOOPER] == 2) || (initStep[INIT_STEP_LOOPER] == 3))
			{
				initMode &= ~0x08;
				initStep[INIT_STEP_LOOPER] = 0;
			}
		}

		if(sysMode & MODE_MTEST)
		{
			mTestMode();
		}
	}

	if(swStFg & ST_SW_BAND_CUT)
	{
		cntlBandCut();

		if(stStep == 2) // end
		{
			stStep = 0;
			swStFg &= ~ST_SW_BAND_CUT;
		}
	}
}

/**
  * @brief auto mode function
  * @param None
  * @retval None
  */	
void autoMode(void)
{
    //float buf=0;
    
    //if((!(errorCode[0] & 0xf3)) && (!(errorCode[1] & 0xcf)) && (!(errorCode[2] & 0x03)) && (!(errorCode[3] & 0x0f)))  // not occured error
    //{
        //if((swStFg & ST_SW_AUTO_STOP) || (comSoftStop == 1)) // stop
        /*
		if(swStFg & ST_SW_AUTO_STOP) // stop
		{
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);

			if(swStFg & ST_SW_AUTO_START) // stop 후  start가 눌림...
			{
				swStFg &= ~ST_SW_AUTO_STOP;
				swStFg &= ~ST_SW_AUTO_START;
			}
		}
		else
		{
		*/
            if(sewRunFg == 1) // sewing machine set befor run
            {
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"auto init\n");
#endif
				//buf = (((float)metLen)/ONE_PULSE_MV);
				//sewMovMaxPulse = (unsigned long)buf;
				sewAct = SEW_ACT_FR;
				
				sewRunFg = 2;
				//sewRunFg = 8;

				looperFg = 0;
				looperCnt = 0;
				lfFg = 0;
				lfCnt = 0;
				lfCLK = 0;
				movFg = 0;
				movCnt = 0;
				movCLK = 0;

				initStep[INIT_STEP_MOVING] = 0;
				initStep[INIT_STEP_CLAMP] = 0;
				initStep[INIT_STEP_NEEDLE] = 0;
				initStep[INIT_STEP_LOOPER] = 0;

				stOut[0] &= ~0x01;
				stOut[1] &= ~0x01;
				stOut[2] &= ~0x01;
				outSignal(OUT_CH1,stOut[0]);
				outSignal(OUT_CH2,stOut[1]);
				outSignal(OUT_CH3,stOut[2]);

				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);	// needle stop
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	// looper stop
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);	// mov stop
				
				//TIM4->ARR = FRQ_25KHz;	// PWM frequency change 200Hz
				//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(FRQ_25KHz+1)/2);
				//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(FRQ_25KHz+1)/2);
				//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_25KHz+1)/2);

				TIM4->ARR = FRQ_15KHz;	// PWM frequency change 500Hz
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(FRQ_10KHz+1)/2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(FRQ_10KHz+1)/2);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_10KHz+1)/2);
				
				SV1_DIR_CCW;	// needle direction ccw set
				SV2_DIR_CCW;	// looper direction ccw set
				SV3_DIR_CCW;	// moving direction forward set

				//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);	// needle start
				//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);	// looper start
            }
			else if(sewRunFg == 2)	// close clamp all
			{
				initClamp(OUT_CLAMP1_CNTL|OUT_CLAMP2_CNTL|OUT_CLAMP3_CNTL|OUT_CLAMP4_CNTL, CLAMP_CLOSE);

				//DWT_Delay_us(100000);
				testCnt = 1000;
				testFg = 1;

				sewRunFg = 3;
				
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp close\n");
#endif

				/*
				if(initStep[INIT_STEP_CLAMP] == 2)
				{
					sewRunFg = 3;
					initStep[INIT_STEP_CLAMP] = 0; 
				}
				else if(initStep[INIT_STEP_CLAMP] == 3)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp close err\n");
#endif
				}
				*/
			}
			else if(sewRunFg == 3)	// delay for clamp close
			{
				if(testFg == 2)
				{
					testFg = 0;
					testCnt = 0;

					sewRunFg = 4;
				}
			}
			else if(sewRunFg == 4) // moving 15mm 밴드를 당기기 위해서...
			{
				if(movFg == 0)
				{
					movCLK = (unsigned long)(20/ONE_PULSE_MV);	// 20mm moving
#ifdef	__DEBUG_SUB__
putChar(&huart4,(movCLK/10000)+0x30);
putChar(&huart4,((movCLK%10000)/1000)+0x30);
putChar(&huart4,((movCLK%1000)/100)+0x30);
putChar(&huart4,((movCLK%100)/10)+0x30);
putChar(&huart4,(movCLK%10)+0x30);

putStr(&huart4,(const uint8_t *)"mov 20mm start\n");
#endif
					movFg = 1;
					movCnt = 0;

					//movCLK = MOV_10MM_CLOCK;	// 10mm moving
					//movCLK = (unsigned long)(15/ONE_PULSE_MV);	// 15mm moving

					//TIM4->ARR = FRQ_25KHz;	// PWM frequency change 200Hz
					//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_25KHz+1)/2);
					//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_100Hz+1)/2);
					
					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);	// moving start
				}
				else
				{
					if(movCLK == movCnt)	// 200mm 이동 완료
					{
						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);	// moving stop

						movFg = 0;
						movCnt = 0;
						movCLK = 0;
						
						sewRunCnt = 0;
						sewMovMaxCnt = 0; // moving
						sewMovCnt = 0;
						sewMovFg = 0;

						sewRunFg = 5;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"20mm mov end\n");
#endif
						
						HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);	// needle start
						HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);	// looper start
					}
				}
			}
            else if(sewRunFg == 5) // sewing machine run
            {
                if(sewMovMaxCnt == MOV_MAX_PULSE)	// moving maximum lenght
        		{
        			sewMovFg = 0;
        			
        			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);	// moving motor stop

        			if(sewRunCnt == SEW_1CYCLE_PULSE)	// 1 cycle moving
        			{
        				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
        				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
						
        				sewRunCnt = 0;
        				sewMovMaxCnt = 0; // moving
        				sewMovCnt = 0;

						sewRunFg = 6;
						
						sewAct = SEW_ACT_FIN;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"sew end\n");
#endif

						
        				//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
        			}
        			else	// not yet finish sewing machine driving 1 cycle
        			{
        				;
        			}	
        		}
        		else	// not yet finish drive moving maximum length
        		{
        			if(sewRunCnt == SEW_1CYCLE_PULSE)	// 1 cycle moving
        			{
        				sewRunCnt = 0;
        			}
        			else if(sewRunCnt == SEW_OFF_FABRIC_PULSE)	// moving now
        			{
        				sewMovCnt = 0;
        				sewMovFg = 1;

        				//sewRunFg = 0;

        				//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
        				//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);

        				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
        			}
        			else{;}
        			
    				if(sewMovFg == 1)	// 
    				{
    					if(sewMovCnt == MOV_1CYCLE_PULSE)	// moving motor 1 cycle running
    					{
	  						sewMovFg = 0;
    						sewMovCnt = 0;

    						//sewRunFg = 0;
    						
    						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
    					}
    				}
    				else{;}
        		}
            }
			else if(sewRunFg == 6)	// looper 반대 방향으로 반바퀴 회전.....
			{
				if(looperFg == 0)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"looper back start\n");
#endif

					looperFg = 1;
					looperCnt = 0;
					

					TIM4->ARR = FRQ_200Hz;	// PWM frequency change 200Hz
					
					SV2_DIR_CW;	// looper 반대방향으로 반바퀴....

					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(FRQ_200Hz+1)/2);
					
					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);	// looper start
				}
				else
				{
					if(looperCnt == 800)	// looper 반바퀴
					{
						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	// looper stop
						
						looperFg = 0;
						looperCnt = 0;

						sewRunFg = 7;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"looper back end\n");
#endif

					}
				}
			}
			else if(sewRunFg == 7)	// mov forward 200mm
			{
				if(movFg == 0)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"200mm mov start\n");
#endif

					movFg = 1;
					movCnt = 0;

					movCLK = MOV_200MM_CLOCK;	// 200mm moving

					//TIM4->ARR = FRQ_25KHz;	// PWM frequency change 25KHz
					//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_25KHz+1)/2);

					TIM4->ARR = FRQ_5KHz;	// PWM frequency change 25KHz
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_5KHz+1)/2);
					
					SV3_DIR_CCW;	// forward direction ccw set
					
					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);	// moving start
				}
				else
				{
					if(movCLK == movCnt)	// 200mm 이동 완료
					{
						HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);	// moving stop

						movFg = 0;
						movCnt = 0;
						movCLK = 0;

						sewRunFg = 8;

						cntlVaccum(VACCUM_OPEN);
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"mov 200mm end and vac open\n");
#endif

					}
				}
			}
			else if(sewRunFg == 8)	// band cut
			{
				cntlBandCut();

				if(bcStep == 2)	// band cut end
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"band cut end\n");
#endif

					bcStep = 0;

					sewRunFg = 9;
				}
			}
			else if(sewRunFg == 9)	// lifting 10mm
			{
				if(lfFg == 0)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"lf 10mm up start\n");
#endif

					lfFg = 1;
					lfCnt = 0;
					lfCLK = 1600;	// 10mm lifting 5mm:800 = 10mm:1600

					SV4_DIR_CCW; // lifting ac servo cw : down, ccw : up
					TIM4->ARR = FRQ_1KHz;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
				}
				else if(lfFg == 1)
				{
					if(lfCLK == lfCnt)	// 1600 clock 이동
					{
						HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);

						lfFg = 0;
						lfCnt = 0;
						lfCLK = 0;

						sewRunFg = 10;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"lf 10mm up end\n");
#endif

					}
				}
			}
			else if(sewRunFg == 10)	// clamp all open
			{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp all open\n");
#endif

				initClamp(OUT_CLAMP1_CNTL|OUT_CLAMP2_CNTL|OUT_CLAMP3_CNTL|OUT_CLAMP4_CNTL, CLAMP_OPEN);

				sewRunFg = 11;
				/*
				if(initStep[INIT_STEP_CLAMP] == 2)
				{
					sewRunFg = 10;
					initStep[INIT_STEP_CLAMP] = 0; 
				}
				else if(initStep[INIT_STEP_CLAMP] == 3)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp open err\n");
#endif
				}
				*/
			}
			else if(sewRunFg == 11)	// moving back
			{
				sewAct = SEW_ACT_BR;
				
				initMoving(1);

				if(initStep[INIT_STEP_MOVING] == 2)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"mov home end\n");
#endif

					sewRunFg = 12;
					initStep[INIT_STEP_MOVING] = 0;
				}
				else if(initStep[INIT_STEP_MOVING] == 3)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"moving home err\n");
#endif
				}

				if(movFg == 2)
				{
					if(lfFg == 0)
					{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"lift down 10mm start\n");
#endif

						lfFg = 1;
						lfCnt = 0;
						lfCLK = 1600;	// 10mm lifting 5mm:800 = 10mm:1600

						SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
						TIM4->ARR = FRQ_1KHz;
						__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
						__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(FRQ_1KHz+1)/2);
						HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
					}
					else
					{
						if(lfFg == 1)
						{
							if(lfCLK == lfCnt)	// 1600 clock 이동
							{
								HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);

								lfFg = 2;
								lfCnt = 0;
								lfCLK = 0;
								movFg = 0;

								//sewRunFg = 0;

								//sewAct = SEW_ACT_WAIT;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"lift dn 10mm end\n");
#endif
							}
						}
					}
				}
			}
			else if(sewRunFg == 12)	// close clamp all
			{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp all close\n");
#endif
				initClamp(OUT_CLAMP1_CNTL|OUT_CLAMP2_CNTL|OUT_CLAMP3_CNTL|OUT_CLAMP4_CNTL, CLAMP_CLOSE);

				sewRunFg = 13;

				lfFg = 0;
				/*
				if(initStep[INIT_STEP_CLAMP] == 2)
				{
					sewRunFg = 12;
					initStep[INIT_STEP_CLAMP] = 0; 
				}
				else if(initStep[INIT_STEP_CLAMP] == 3)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"clamp close err\n");
#endif
				}
				*/
			}
			else if(sewRunFg == 13)	// 니들과 루퍼 초기화...
			{
				initNeedle();
				initLooper();

				if((initStep[INIT_STEP_NEEDLE] == 2) && (initStep[INIT_STEP_LOOPER] == 2))
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"needle and looper init end\n");
#endif

					//sewRunFg = 13;
					sewRunFg = 0;
					initStep[INIT_STEP_NEEDLE] = 0;
					initStep[INIT_STEP_LOOPER] = 0;

					sewRunFg = 0;

					sewAct = SEW_ACT_WAIT;

					cntlVaccum(VACCUM_CLOSE);
				}
				else
				{
					if(initStep[INIT_STEP_NEEDLE] == 3)
					{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"needle init err\n");
#endif
					}

					if(initStep[INIT_STEP_LOOPER] == 3)
					{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"needle init err\n");
#endif
					}
				}
			}
			/*
			else if(sewRunFg == 13)	// 리프팅 10mm 다운...
			{
				if(lfFg == 0)
				{
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"auto start\n");
#endif

					lfFg = 1;
					lfCnt = 0;
					lfCLK = 1600;	// 10mm lifting 5mm:800 = 10mm:1600

					SV4_DIR_CW; // lifting ac servo cw : down, ccw : up
					TIM4->ARR = FRQ_1KHz;
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(FRQ_1KHz+1)/2);
					HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
				}
				else
				{
					if(lfCLK == lfCnt)	// 1600 clock 이동
					{
						HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

						lfFg = 0;
						lfCnt = 0;
						lfCLK = 0;

						sewRunFg = 0;

						sewAct = SEW_ACT_WAIT;
#ifdef	__DEBUG_SUB__
putStr(&huart4,(const uint8_t *)"auto end\n");
#endif

					}
				}
			}
			*/
        //}
	/*
	}
	else // error occured
    {
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
    }
    */
}

/**
  * @brief switch scan function
  * @param None
  * @retval None
  */	
void runningProcess(void)
{
    unsigned int err=0;

    if(sysStartFg == 1)  // all input channel check done
    {
        if(errorCode[3] & 0x01) err |= 0x0001; // servo1 alarm
        else
        {
			stOut[0] &= ~0x01; // servo1 ready -> servo1 on
			outSignal(OUT_CH1,stOut[0]); //servo1 on
			
            if(svRdy & 0x01) err |= 0x0100;
        }

		if(errorCode[3] & 0x02) err |= 0x0002; // servo2 alarm
        else
        {
        	stOut[1] &= ~0x01; // servo1 ready -> servo1 on
			outSignal(OUT_CH2,stOut[1]); //servo1 on

			if(svRdy & 0x02) err |= 0x0200;
        }

        if(errorCode[3] & 0x04) err |= 0x0004; // servo3 alarm
        else
        {
            stOut[2] &= ~0x01; // servo1 ready -> servo1 on
			outSignal(OUT_CH3,stOut[2]); //servo1 on

			if(svRdy & 0x04) err |= 0x0400;
        }

        if(errorCode[3] & 0x08) err |= 0x0008; // servo4 alarm
        else
        {
            stOut[3] &= ~0x01; // servo1 ready -> servo1 on
			outSignal(OUT_CH4,stOut[3]); //servo1 on

			if(svRdy & 0x08) err |= 0x0800;
        }
#if 0
        if(stSWin & ST_SW_EMS)
        {
            err |= 0x0010; // ems on
        }

        if(!(stSENin & ST_SEN_MOVING_HOME))
        {
            err |= 0x0020; // moving home sensor not active
        }

        if(comOKFg != 1)
        {
            err |= 0x0040; // moving home sensor not active        
        }
#endif

        if(err == 0)
        {
#ifdef __DEBUG_SUB__
putStr(&huart4,(const unsigned char *)"all OK\n");
#endif
            //sysStartFg = 2; // not error issue
            sysStartFg = 2; // not error issue

            initStep[INIT_STEP_LIFTING] = 0;
            initStep[INIT_STEP_NEEDLE] = 0;
            initStep[INIT_STEP_LOOPER] = 0;
			initStep[INIT_STEP_MOVING] = 0;
			
            stStep = 0;

			//changeFRQ(MODE_TEST); // pwm speed change -> 100Hz
        }
#ifdef __DEBUG_SUB__
if(err & 0x0001)  putStr(&huart4,(const unsigned char *)"SV1 ALARM\n");
else
{
    if(err & 0x0100)    putStr(&huart4,(const unsigned char *)"SV1 NOT READY\n");
    else                putStr(&huart4,(const unsigned char *)"SV1 READY\n");
}
if(err & 0x0002)  putStr(&huart4,(const unsigned char *)"SV2 ALARM\n");
else
{
    if(err & 0x0200)    putStr(&huart4,(const unsigned char *)"SV2 NOT READY\n");
    else                putStr(&huart4,(const unsigned char *)"SV2 READY\n"); 

}
if(err & 0x0004)  putStr(&huart4,(const unsigned char *)"SV3 ALARM\n");
else
{
    if(err & 0x0400)    putStr(&huart4,(const unsigned char *)"SV3 NOT READY\n");
    else                putStr(&huart4,(const unsigned char *)"SV3 READY\n");
}
if(err & 0x0008)  putStr(&huart4,(const unsigned char *)"SV4 ALARM\n");
else
{
    if(err & 0x0800)    putStr(&huart4,(const unsigned char *)"SV4 NOT READY\n");
    else                putStr(&huart4,(const unsigned char *)"SV4 READY\n");
}
#endif
    }
    #if 0
    else if(sysStartFg == 2)  // position initalization initiate(lifting -> needle & looper
    {
        if(sysMode == MODE_EMS)
        {
            initStep[INIT_STEP_LIFTING] = 0;
    		initStep[INIT_STEP_NEEDLE] = 0;
    		initStep[INIT_STEP_LOOPER] = 0;
    		initStep[INIT_STEP_MOVING] = 0;
            initStep[INIT_STEP_CLAMP] = 0;

            stStep = 0;
    		stCkCnt = 0;
    		stCkFg = 0;
        }
        else
        {
            initMachine();

            if((stStep >= 10) && (stStep <= 16))
            {
                if(stStep == 10) // clamp open error
                {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"Init Clamp open err\n");
#endif
                    stStep = 1;
                }
                else if(stStep == 11)
                {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"Init Lifting home err\n");
#endif
                    stStep = 2;
                }
                else if((stStep >= 12) && (stStep <= 14))
                {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"Init Needle,Looper,Moving err\n");
#endif
                    stStep = 3;
                }
                else if(stStep == 16)
                {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"Init Clamp close err\n");
#endif
                    stStep = 4;
                }
                else
                {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"Unknown err\n");
#endif
                    stStep = 0;
                }
            }
            else
            {
#ifdef __DEBUG_SUB__
    putStr(&huart4,"init ok\n");
#endif        
                sysStartFg = 3;

                stCkCnt = 0;
            }

            initStep[INIT_STEP_LIFTING] = 0;
            initStep[INIT_STEP_NEEDLE] = 0;
            initStep[INIT_STEP_LOOPER] = 0;
            initStep[INIT_STEP_MOVING] = 0;
            initStep[INIT_STEP_CLAMP] = 0;

            stCkCnt = 0;
            stCkFg = 0;
        }
    }
    #endif
    //else if(sysStartFg == 3) // auto or manual mode - if no error band empty12 normal, band lost12 normal, up-down thread normal, 
    else if(sysStartFg == 2) // auto or manual mode - if no error band empty12 normal, band lost12 normal, up-down thread normal, 
    {
        if(sysMode & MODE_EMS) // ems switch lock
        {
            // ems tower lamp display
		}
        else
        {
            if(sysMode & MODE_MANUAL) // manual mode
            {
                manualMode();
            }
            else if(sysMode & MODE_AUTO) // auto mode - if run before communication to pc at once!!
            {
                //if(comOKFg == 1)
                //{
                    autoMode();

                    #if 0
                    if((!(errorCode[0] & 0xf3)) && (!(errorCode[1] & 0xcf)) && (!(errorCode[2] & 0x03) && (!(errorCode[3] & 0x0f))
                    {
                        ;
                    }
                    else // error occured
                    {
                        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
                        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
                        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
                        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
                    }
                    #endif
                //}
            }
            else{;}
        }
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


