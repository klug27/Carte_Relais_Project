/************************************************************************************************************
* @file	   com_stm32f1.h
* @author  Awatsa Hermann
* @date	   11.06.2023
*************************************************************************************************************
* @brief   stm32f1xx export file
*
*************************************************************************************************************
*@remarks
*
************************************************************************************************************/


#ifndef __COM_STM32F1__
#define __COM_STM32F1__


/************************************************************************************************************
 *                                              I N C L U D E S
 ***********************************************************************************************************/
#include "stm32f1_hw_def.h"



/************************************************************************************************************
 *                                          EXPORT TYPES
 ***********************************************************************************************************/
extern DMA_HWREG*         const DMA;
extern TIM_HWREG*         const TIMER2;
extern TIM_HWREG*         const TIMER3;
extern GPIO_HWREG*        const PORTA;
extern GPIO_HWREG*        const PORTB;
extern GPIO_HWREG*        const PORTC;
extern GPIO_HWREG*        const PORTD;
extern GPIO_HWREG*        const PORTE;
extern RCC_HWREG*         const RCCTRL;
extern USART_HWREG*       const UART1;
extern USART_HWREG*       const UART2;
extern USART_HWREG*       const UART3;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL1;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL2;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL3;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL4;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL5;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL6;
extern DMA_CHANNEL_HWREG* const DMA1_CHANNEL7;



#endif	/* _COM_STM32F1_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
