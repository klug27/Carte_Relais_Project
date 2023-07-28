/************************************************************************************************************
* @file	   hw_desc_uart_stm32f1.h
* @date	   11.06.2023
*************************************************************************************************************
* @brief   This file contains the functions need to handle the UART peripheral
*
************************************************************************************************************/

#ifndef _HW_DESC_UART_STM32F1_H_
#define _HW_DESC_UART_STM32F1_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "hw_desc_dma_stm32f1.h"


/************************************************************************************************************
*	         DEFINES
************************************************************************************************************/
#define BUFFER_SIZE  (uint16_t)256



/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

typedef enum
{
  UART_ID0 = 0,
  UART_ID1,
  UART_ID2,
  UART_MAX
}eUART_ID;

/**
 * Select the parameters of uart
 */
typedef enum
{
  UART_2K4_8b_NONE_ONE   = 2400U,
  UART_9K6_8b_NONE_ONE   = 9600U,
  UART_57K6_8b_NONE_ONE  = 57600U,
  UART_115K2_8b_NONE_ONE = 115200U,
  UART_921K6_8b_NONE_ONE = 921600U
}eUARTParams_t;

/**
 * @struct uart buffer structure
 */
typedef struct sBuffer_tTag
{
	uint8_t  u8Data[BUFFER_SIZE];
	uint16_t u16ReadIndex;
	uint16_t u16WriteIndex;
} sBuffer_t;

/**
 * @struct uart parameters
 */
typedef struct sUartCfg_tTag
{
 eUART_ID      eID;         /**< uart ID */
 eUARTParams_t eUartParams; /**< uart parameters */
}sUartCfg_t;

/**
 * callback functions types declarations
 */
typedef void (*dmaTriggerTxFunc_t)(uint16_t u16TxSize, uint32_t u32TxData);  
typedef u16  (*dmaGetRcvIdxFunc_t)(void);
typedef bool (*dmaTxIsCpltFunc_t)(void);
typedef bool (*dmaTxIsOngoingFunc_t)(void);


/**
 * @brief The uart structure
 */
typedef struct
{
  sUartCfg_t            sCfg;
  sBuffer_t             sRxBuffer;        /**< reception buffer */
  sBuffer_t             sTxBuffer;        /**< transmission buffer */
  dmaTriggerTxFunc_t    pfvTriggerTx;     /**< trigger transmission dma callback  */
  dmaGetRcvIdxFunc_t    pfu16GetRcvIdx;   /**< current index of dma transfer callback */
  dmaTxIsCpltFunc_t     pfbTxIsCplt;      /**< dma transmission completed state callback */
  dmaTxIsOngoingFunc_t  pfbIsTxOngoing;   /**< dma transmission ongoing state callback */
}sUARTObj_t;


/************************************************************************************************************
 *                                     PUBLIC FUNCRTIONS DECLARATION
 ***********************************************************************************************************/

/**
 * @brief Initialize the uart peripheral
 * 
 * @param [in] psUartCfg uart configuration parameters
 */
void vUartHwInit(const sUartCfg_t* psUartCfg);


/**
 * @brief Get the uart object
 * 
 * @param [in] psUartCfg uart config parameters
 * @return the uart object
 */
const sUARTObj_t* GetUARTObj(const sUartCfg_t* psUartCfg);


#endif /* _HW_DESC_UART_STM32F1_H_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
