
/************************************************************************************************************
* @file		 stm32f1_comT02.h
* @author	 Awatsa Hermann
* @date		 11.06.2023
*************************************************************************************************************
* @brief	 peripherals access layer
*
*************************************************************************************************************
*@remarks
*
************************************************************************************************************/

#ifndef _STM32F1_COMT02_H_
#define _STM32F1_COMT02_H_


#include "stm32f1_types.h"


/* define UART BASE address */
#define USART1_ADDRESS               (u32)(0x40013800UL)
#define USART2_ADDRESS               (u32)(0x40004400UL)
#define USART3_ADDRESS               (u32)(0x40004800UL)

/* define RCC BASE address */
#define RCC_ADDRESS                  (u32)(0x40021000UL)

/* define DMAs and DMA channels addresses */
#define DMA1_ADDRESS                 (u32)(0x40020000UL)
#define DMA1_CHANNEL1_ADDRESS        (u32)(0x40020008UL)
#define DMA1_CHANNEL2_ADDRESS        (u32)(0x4002001CUL)
#define DMA1_CHANNEL3_ADDRESS        (u32)(0x40020030UL)
#define DMA1_CHANNEL4_ADDRESS        (u32)(0x40020044UL)
#define DMA1_CHANNEL5_ADDRESS        (u32)(0x40020058UL)
#define DMA1_CHANNEL6_ADDRESS        (u32)(0x4002006CUL)
#define DMA1_CHANNEL7_ADDRESS        (u32)(0x40020080UL)

/* define GPIO base addresses */
#define GPIOA_ADDRESS                (u32)(0x40010800UL)
#define GPIOB_ADDRESS                (u32)(0x40010C00UL)
#define GPIOC_ADDRESS                (u32)(0x40011000UL)
#define GPIOD_ADDRESS                (u32)(0x40011400UL)
#define GPIOE_ADDRESS                (u32)(0x40011800UL)

/* alternate function base address */
#define AFIO_ADDRESS                 (u32)(0x40010000UL)

/* Timers base addresses */
#define TIM2_ADDRESS                 (u32)(0x40000000UL)
#define TIM3_ADDRESS                 (u32)(0x40000400UL)

/* Watchdog base addess */
#define IWDG_ADDRESS                 (u32)(0x40003000UL)



/************************************************************************************************************
 *                                           STRUCTURES DEFINITIONS
 ***********************************************************************************************************/

/** 
 * @brief Structure type to access the DMA channel registers.
 */
typedef struct
{
  _IO uint32_t CCR;
  _IO uint32_t CNDTR;
  _IO uint32_t CPAR;
  _IO uint32_t CMAR;

} DMA_CHANNEL_HWREG;

/**
 \brief  Structure type to access the DMA registers.
 */
typedef struct
{
  _IO uint32_t ISR;
  _IO uint32_t IFCR;

} DMA_HWREG;

/** 
 * @brief General Purpose I/O
 */
typedef struct
{
  _IO uint32_t CR[2];
  _IO uint32_t IDR;
  _IO uint32_t ODR;
  _IO uint32_t BSRR;
  _IO uint32_t BRR;
  _IO uint32_t LCKR;

} GPIO_HWREG;

/** 
 * @brief Alternate Function I/O
 */
typedef struct
{
  _IO uint32_t EVCR;
  _IO uint32_t MAPR;
  _IO uint32_t EXTICR[4];
      uint32_t RESERVED0;
  _IO uint32_t MAPR2;  

} AFIO_HWREG;

/** 
 * @brief Reset and Clock Control
 */
typedef struct
{
  _IO uint32_t CR;
  _IO uint32_t CFGR;
  _IO uint32_t CIR;
  _IO uint32_t APB2RSTR;
  _IO uint32_t APB1RSTR;
  _IO uint32_t AHBENR;
  _IO uint32_t APB2ENR;
  _IO uint32_t APB1ENR;
  _IO uint32_t BDCR;
  _IO uint32_t CSR;

} RCC_HWREG;

/** 
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */
typedef struct
{
  _IO uint32_t SR;
  _IO uint32_t DR;
  _IO uint32_t BRR;
  _IO uint32_t CR1;
  _IO uint32_t CR2;
  _IO uint32_t CR3;
  _IO uint32_t GTPR;

} USART_HWREG;

/**
  * @brief Timers registers
  */
typedef struct
{
  _IO uint32_t CR1;  
  _IO uint32_t CR2;  
  _IO uint32_t SMCR; 
  _IO uint32_t DIER; 
  _IO uint32_t SR;   
  _IO uint32_t EGR;  
  _IO uint32_t CCMR1;
  _IO uint32_t CCMR2;
  _IO uint32_t CCER; 
  _IO uint32_t CNT;  
  _IO uint32_t PSC;  
  _IO uint32_t ARR;  
  _IO uint32_t RCR;  
  _IO uint32_t CCR1; 
  _IO uint32_t CCR2; 
  _IO uint32_t CCR3; 
  _IO uint32_t CCR4; 
  _IO uint32_t BDTR; 
  _IO uint32_t DCR;  
  _IO uint32_t DMAR; 
  _IO uint32_t OR;   
}TIM_HWREG;



/**
 * @brief IWDG registers
 * 
 */
typedef struct
{
  _IO uint32_t KR;  
  _IO uint32_t PR;  
  _IO uint32_t RLR; 
  _IO uint32_t SR;      
}IWDG_HWREG;


#endif /* _STM32F1_COMT02_H_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
