
/************************************************************************************************************
* @file	hw_desc_timer_stm32f1.h
* @date	19.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle the timers
*
************************************************************************************************************/

#ifndef _HW_DESC_TIMER_STM32F1_H_
#define _HW_DESC_TIMER_STM32F1_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include <stdbool.h>
#include "com_stm32f1.h"


/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

/* define callback functions */
typedef void (*cbkFun_t)(void);
typedef void (*startCbk_t)(void);
typedef u32  (*getTickMsCbk_t)(void);
typedef bool (*registerCbkFunc_t)(cbkFun_t, u32, u8);
typedef bool (*hasElapsedCbk_t)(u32, u32);


/*
 * Init structure of Timer
 */
typedef struct
{
    startCbk_t          pfvStart;        /*< callback function to start the timer */
    getTickMsCbk_t      pfu32GetTickMs;  /*< callback function to get the number of ticks */
    registerCbkFunc_t   pfbRegisterCbk;  /*< callback function to register a callback */
    hasElapsedCbk_t     pfbHasElapsed;   /*< callback function to tell if the specified time has elapsed */

}sTIMObj_t;


/************************************************************************************************************
 *                                     PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief Initialize the timer peripheral
 * 
 * @param [in] sTIMObj  timer object
 */
void vTimerInit(sTIMObj_t* psTIMObj);


#endif /* _HW_DESC_TIMER_STM32F1_H_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/

/************************ Copyright (C) 2023 CACES-SARL.  All rights reserved. ****/
