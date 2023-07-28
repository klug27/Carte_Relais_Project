/************************************************************************************************************
* @file	watchdog_conf.h
* @date	26.07.2023
*************************************************************************************************************
* @brief This file contains some functions to handle independent watchdog timer
*
************************************************************************************************************/

#ifndef WATCHDOG_CONF_H_
#define WATCHDOG_CONF_H_

#include "hw_desc_wdt_stm32f1.h"

/********************************************************************************************************************
*													                                                                *
*	                          P U B L I C S  F U N C T I O N  D E C L A R A T I O N                                 *
*   													                                                            *
********************************************************************************************************************/

/**
 * @brief   this function initialize watchdog
 * @return  Nothing
 */
void vWatchdogHwInit(void);


/**
 * @brief   this function is used to refresh watchdog
 * @return  Nothing
 */
void vWatchdogRefresh(void);

#endif
