
/************************************************************************************************************
* @file		stm32f1_comT02.c
* @author	AWATSA HERMANN	
* @date		11.06.2023
*************************************************************************************************************
* @brief	peripherals access layer
*
*************************************************************************************************************
*@remarks
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "stm32f1_hw_def.h"



/************************************************************************************************************
*	                          DMA POINTERS DEFINITIONS 
************************************************************************************************************/
DMA_HWREG*         const DMA            =      (DMA_HWREG*)DMA1_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL1  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL1_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL2  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL2_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL3  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL3_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL4  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL4_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL5  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL5_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL6  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL6_ADDRESS;
DMA_CHANNEL_HWREG* const DMA1_CHANNEL7  =      (DMA_CHANNEL_HWREG*)DMA1_CHANNEL7_ADDRESS;


/************************************************************************************************************
*	                          GPIO POINTERS DEFINITIONS 
************************************************************************************************************/
GPIO_HWREG* const PORTA                 =      (GPIO_HWREG*)GPIOA_ADDRESS;
GPIO_HWREG* const PORTB                 =      (GPIO_HWREG*)GPIOB_ADDRESS;
GPIO_HWREG* const PORTC                 =      (GPIO_HWREG*)GPIOC_ADDRESS;
GPIO_HWREG* const PORTD                 =      (GPIO_HWREG*)GPIOD_ADDRESS;
GPIO_HWREG* const PORTE                 =      (GPIO_HWREG*)GPIOE_ADDRESS;


/************************************************************************************************************
*	                          RESET AND CLOCK POINTERS DEFINITIONS 
************************************************************************************************************/
RCC_HWREG* const RCCTRL                 =      (RCC_HWREG*)RCC_ADDRESS;


/************************************************************************************************************
*	                          USART POINTERS DEFINITIONS 
************************************************************************************************************/
USART_HWREG* const UART1               =      (USART_HWREG*)USART1_ADDRESS;
USART_HWREG* const UART2               =      (USART_HWREG*)USART2_ADDRESS;
USART_HWREG* const UART3               =      (USART_HWREG*)USART3_ADDRESS;


/************************************************************************************************************
*	                          TIMERS POINTERS DEFINITIONS 
************************************************************************************************************/
TIM_HWREG* const TIMER2                =      (TIM_HWREG*)TIM2_ADDRESS;
TIM_HWREG* const TIMER3                =      (TIM_HWREG*)TIM3_ADDRESS;


/************************************************************************************************************
*	                          INDEPENDENT WATCHDOG POINTER DEFINITION
************************************************************************************************************/


/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
