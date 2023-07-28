/************************************************************************************************************
* @file	m41t56_rtc.h
* @date	27.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle the rtc module
*
************************************************************************************************************/

#ifndef _M41T56_RTC_H_
#define _M41T56_RTC_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "stdint.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"



/************************************************************************************************************
 *                            PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief Initialize the rtc module
 * 
 * @param [in] hi2c i2c handle structure
 * 
 * @return true if the module is able to communicate, false otherwise 
 */
bool bM41t56Init(I2C_HandleTypeDef *psI2C);


/**
 * @brief Set the time in the m41t56 rtc module
 * 
 * @param [in] u32UnixTime unix time
 * 
 * @return true if the time was set correctly, false otherwise 
 */
bool bM41t56SetTime(uint32_t u32UnixTime);


/**
 * @brief Get the unix time from the m41t56 rtc module
 * 
 * @return the unix time compute from the module or 0 in case of error
 */
uint32_t u32M41t56GetTime(void);



#endif /* _M41T56_RTC_H_ */

/************************************************************************************************************
 *                            END OF MODULE
 ***********************************************************************************************************/
