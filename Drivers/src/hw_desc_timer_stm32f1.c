/************************************************************************************************************
* @file	hw_desc_timer_stm32f1.c
* @date	19.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle the timers
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include <string.h>
#include "hw_desc_timer_stm32f1.h"
#include "stm32f100xe.h"



/************************************************************************************************************
*	                         DEFINES 
************************************************************************************************************/
#define PRIORITY_0         (u32)0
#define PRESCALER_49       (u16)49
#define AUTORELOAD_479     (u16)479
#define CLEAR_REG          (u16)0
#define CBK_FUNCT_MAX      (u8)10
#define TIMER_ONE          (u8)1



/************************************************************************************************************
*	                         STRUCTURE
************************************************************************************************************/

/**
 * @brief Structure  containing data to register in timer
 */
typedef struct sTimerData_tTag
{
  volatile  cbkFun_t  vCbk;                /* Callback function */
  volatile  uint32_t  u32Period;           /* Period to call vCbk */
  volatile  uint32_t  u32NextTime;         /* time after which the function will be called again */

  union
  {
     volatile uint32_t  u32Flag;
     struct
     {
        volatile uint32_t  REGFUNCBK   : 1;     /* this bit is set automatically to '1' by the software to register the function */
        volatile uint32_t  REP         : 8;     /* set the repeating time of call back function. The correct value are in the range of 1 to 255 , otherwise set '0' to disable this function */
        volatile uint32_t  REPIND      : 1;     /* set this bit to '1' allows function to be call indefinitely */
        volatile uint32_t              : 22;    /* unused bits */
     } u32Flag_b;
  };
} sTimerData_t;

/************************************************************************************************************
*	                         PRIVATE VARIABLES
************************************************************************************************************/
static uint32_t     u32TickMs  = 0;
static sTimerData_t sTimerCtrl = {0};


/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/   

/** @brief Start the counting for timer 2
 **/
static inline void vStartID0(void)
{
    NVIC_EnableIRQ(TIM2_IRQn);               /* Enable timer 2 interrupt request */
    NVIC_SetPriority(TIM2_IRQn, PRIORITY_0); /* set the priority to 0 */
    TIMER2->CR1 |= TIM_CR1_CEN;              /* Activate the counter for timer 2 */
}


/** @brief Get the timer 2 ticks
  * 
  * @return tick in ms
 **/
static inline uint32_t u32GetTicksMsID0(void)
{
    return u32TickMs;
}


/** @brief       registration of callback function
  * @param [IN]  cbkFunct       : function to called
  * @param [IN]  u32Period      : it indicates the period after which the function is to be called up
  * @param [IN]  u8RepeatedTime : it indicates This parameter specifies the number of times the function is to be called. , set 0 : to call the function indefinitely
  * @return      true if the function was succesfully registered, false otherwise
 **/
static bool bRegisterCbkID0(cbkFun_t cbkFunct, uint32_t u32Period, uint8_t u8RepeatedTime);


/**
 * @brief Check if the specified time has elapsed
 * 
 * @param u32ActualTime the actual recorded time
 * @param u32TargetTime the target time
 * @return true if the time is elapsed and false otherwise
 */
static inline bool bHasElapseID0(u32 u32ActualTime, u32 u32TargetTime);



/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static bool bRegisterCbkID0(cbkFun_t cbkFunct, uint32_t  u32Period, uint8_t u8RepeatedTime)
{
    bool bRet = false;

    if((cbkFunct != NULL_PTR) && (u32Period != 0) && (sTimerCtrl.vCbk == NULL_PTR))
    {
		/* fetch function and period */
		sTimerCtrl.vCbk        = cbkFunct;
		sTimerCtrl.u32Period   = u32Period;
		sTimerCtrl.u32NextTime = u32GetTicksMsID0() + u32Period;

		/* Initialization of flag */
		sTimerCtrl.u32Flag = 0;

		/* set registration callback function flag */
		sTimerCtrl.u32Flag_b.REGFUNCBK = TIMER_ONE;

		/* check if function shall called indefinitely */
		if (u8RepeatedTime == 0)
		{
			sTimerCtrl.u32Flag_b.REPIND = TIMER_ONE;
		}
		else
		{
			sTimerCtrl.u32Flag_b.REP = u8RepeatedTime;
		}
		bRet = true;
    }

    return bRet;
}


static bool bHasElapseID0(u32 u32ActualTime, u32 u32TargetTime)
{
    bool bRet = false;

    if (u32GetTicksMsID0() >= u32ActualTime)
    {
        if ((u32GetTicksMsID0() - u32ActualTime) >= u32TargetTime)
        {
            bRet = true;
        }
    }
    else
    {
        if ((u32GetTicksMsID0() + (UINT32_MAX - u32ActualTime)) >= u32TargetTime)
        {
            bRet = true;
        }
    }
    
    return bRet;
}


void TIM2_IRQHandler(void)
{
	if (TIMER2->SR & TIM_SR_UIF)
	{
        /* clear the interrupt update flag for timer 2 */
        TIMER2->SR &= ~TIM_SR_UIF;
	}

	u32TickMs++;

	/* check if function was registered and period elapsed */
	if ((sTimerCtrl.vCbk != NULL_PTR) &&
		(u32TickMs >= sTimerCtrl.u32NextTime) &&
		(sTimerCtrl.u32Flag_b.REGFUNCBK == TIMER_ONE))
	{
		/* check if the function must be called indefinitely */
		if (sTimerCtrl.u32Flag_b.REPIND == TIMER_ONE)
		{
			/* call the function */
			sTimerCtrl.vCbk();

			/* compute next period */
			sTimerCtrl.u32NextTime += sTimerCtrl.u32Period;
		}
		else if (sTimerCtrl.u32Flag_b.REP > 0)
		{
			/* call the function */
			sTimerCtrl.vCbk();

			/* compute next period */
			sTimerCtrl.u32NextTime += sTimerCtrl.u32Period;

			/* decrement the repeated count value */
			sTimerCtrl.u32Flag_b.REP -= TIMER_ONE;
		}
	}
}


/************************************************************************************************************
 *                           PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vTimerInit(sTIMObj_t* psTIMObj)
{
    CHECK_PTR(psTIMObj)

    RCCTRL->APB1ENR         |= RCC_APB1ENR_TIM2EN;  /* Power the timer 2 */
    TIMER2->CR1              = CLEAR_REG;           /* Clear the configuration register 1 */
    TIMER2->CNT              = CLEAR_REG;           /* Clear the counter register */
    TIMER2->DIER            |= TIM_DIER_UIE;        /* enable the interrupt on update event */
    TIMER2->PSC              = PRESCALER_49;        /* set the prescaler value to 49 */
    TIMER2->ARR              = AUTORELOAD_479;      /* set the autoreload value to 479 */

    psTIMObj->pfvStart       = vStartID0;           /* Set the start function for timer 2 */
    psTIMObj->pfu32GetTickMs = u32GetTicksMsID0;    /* Set the getTicksMs function for timer 2 */
    psTIMObj->pfbRegisterCbk = bRegisterCbkID0;     /* Set register callback function */
	psTIMObj->pfbHasElapsed  = bHasElapseID0;       /* set the elapsed time function */
}


/************************************************************************************************************
 *                           END OF MODULE
 ***********************************************************************************************************/
