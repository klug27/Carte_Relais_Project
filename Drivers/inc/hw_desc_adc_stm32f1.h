/************************************************************************************************************
* @file		hw_desc_adc_stm32f1.h
* @date		25.06.2023
*************************************************************************************************************
* @brief	This file contains the functions need to handle the ADC peripheral
*
************************************************************************************************************/

#ifndef _HW_DESC_ADC_STM32F1_H_
#define _HW_DESC_ADC_STM32F1_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "com_stm32f1.h"
#include "hw_desc_timer_stm32f1.h"


/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

/* callback pointers definitions */
typedef void (*startADCConvCbk_t)(void);
typedef void (*getConvResultCkt_t)(uint16_t* pu16ConvResult);


/**
 * @brief adc object
 */
typedef struct
{
    startADCConvCbk_t  pfvStartADCConv;     /*< start the adc conversion */
    getConvResultCkt_t pfvGetConvResult;    /*< get the conversion result */
}sADCObj_t;


/************************************************************************************************************
 *                                     PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief Initialze the ADC
 * 
 * @param [in] psADCObj   adc object
 * @param [in] psTimerObj timer object
 */
void vADCInit(sADCObj_t *psADCObj, const sTIMObj_t *psTimerObj);


#endif /* _HW_DESC_ADC_STM32F1_H_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
