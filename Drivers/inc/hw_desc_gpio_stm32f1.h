/************************************************************************************************************
* @file	  hw_desc_gpio_stm32f1.h
* @date	  22.06.2023
*************************************************************************************************************
* @brief  This file contains the functions need to handle the GPIO peripheral
*
************************************************************************************************************/

#ifndef _HW_DESC_GPIO_STM32F1_H_
#define _HW_DESC_GPIO_STM32F1_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include <stdbool.h>


/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

/**
 * @brief select the input configuration
 */
typedef enum
{
   INPUT_ANALOG = 0,
   INPUT_FLOATING  ,
   INPUT_PULL_UP   ,
   INPUT_PULL_DOWN ,
   INPUT_PULL_MAX
}eInputCfg_t;


/**
 * @brief select the ouput configuration
 */
typedef enum
{
   OUTPUT_PUSH_PULL  = 1,
   OUTPUT_OPEN_DRAIN = 2
}eOuputCfg_t;


/**
 * @brief select the alternate function configuration
 */
typedef enum
{
   ALTERNATE_PUSH_PULL  = 2,
   ALTERNATE_OPEN_DRAIN = 3
}eAltCfg_t;


/**
 * @brief select the GPIO port
 */
typedef enum
{
   GPIO_PORTA = 0,
   GPIO_PORTB ,
   GPIO_PORTC ,
   GPIO_PORTD ,
   GPIO_PORTE ,
   GPIO_PORT_MAX
}eGPIOPort_t;


/**
 * @brief select the GPIO pin
 */
typedef enum
{
   GPIO_PIN0 = 0,
   GPIO_PIN1  ,
   GPIO_PIN2  ,
   GPIO_PIN3  ,
   GPIO_PIN4  ,
   GPIO_PIN5  ,
   GPIO_PIN6  ,
   GPIO_PIN7  ,
   GPIO_PIN8  ,
   GPIO_PIN9  ,
   GPIO_PIN10 ,
   GPIO_PIN11 ,
   GPIO_PIN12 ,
   GPIO_PIN13 ,
   GPIO_PIN14 ,
   GPIO_PIN15 ,
}eGPIOPin_t;


typedef void (*gpioCfgInputCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t, eInputCfg_t);
typedef void (*gpioCfgAlternateCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t, eAltCfg_t);
typedef bool (*gpioReadPinCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t);
typedef void (*gpioCfgOutputCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t, eOuputCfg_t);
typedef void (*gpioSetPinCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t);
typedef void (*gpioResetPinCbk_t)(eGPIOPort_t eGPIOPort, eGPIOPin_t);


/**
 * @brief GPIO object
 */
typedef struct
{
   gpioCfgInputCbk_t     pfvCfgInput    ;    /*< function used to configure a pin as input pin */
   gpioCfgOutputCbk_t    pfvCfgOutput   ;    /*< function used to configure a pin as output pin */
   gpioCfgAlternateCbk_t pfvCfgAlternate;    /*< function used to configure a pin as alternate mode pin */
   gpioReadPinCbk_t      pfbReadPin     ;    /*< function used to read the state of a pin */
   gpioSetPinCbk_t       pfvSetPin      ;    /*< function used to set a pin at logic high */
   gpioResetPinCbk_t     pfvResetPin    ;    /*< function used to reset a pin at logic low */
}sGPIObj_t;


/************************************************************************************************************
 *                                     PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief Initialize the GPIO port
 * 
 * @param [in] psGPIObj  gpio object
 * @param [in] eGPIOPort gpio port
 */
void vGPIOInit(sGPIObj_t* psGPIObj, eGPIOPort_t eGPIOPort);


#endif /* _HW_DESC_GPIO_STM32F1_H_ */

/************************************************************************************************************
 *                                      END OF MODULE
 ***********************************************************************************************************/

/************************ Copyright (C) 2023 CACES-SARL.  All rights reserved. ****/
