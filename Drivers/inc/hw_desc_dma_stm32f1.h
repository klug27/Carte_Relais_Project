/************************************************************************************************************
* @file		hw_desc_dma_stm32f1.h
* @date		11.06.2023
*************************************************************************************************************
* @brief	This file contains the functions need to handle the DMA controller
*
************************************************************************************************************/

#ifndef _HW_DESC_DMA_STM32F1_H_
#define _HW_DESC_DMA_STM32F1_H_


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include <stdbool.h>
#include "com_stm32f1.h"


/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

/**
 * @brief Select the dma channel
 */
typedef enum
{
    DMA_CHANNEL_1 = 0,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
    DMA_CHANNEL_7,
	DMA_CHANNEL_MAX
}eDmaChanSel_t;

/**
 * @brief Select the type of the dma transfert
 */
typedef enum
{
    DMA_TRANSMIT = 0, 
    DMA_RECEIVE,
    MAX_DMA_TRANSFER

}eDmaXferType_t;

/**
 * @brief Select the dma word size
 */
typedef enum
{
    DMA_WORD_8 = 0,
    DMA_WORD_16,
    DMA_WORD_32,
	DMA_WORD_MAX

}eDmaWordSize_t;


/**
 * @brief The dma initialisation structure
 */
typedef struct
{
    uint32_t         u32DmaSrcReg;    /*< Dma Source Address register */
    uint32_t         u32DmaDesReg;    /*< Dma destination Address register */
    uint32_t         u32RcvAddress;   /*< Buffer containing received datas */
    uint16_t         u16RcvDmaSize;   /*< Dma transfer size for reception */
    eDmaChanSel_t    eDmaChanSel;     /*< Dma channel for transmission or reception */
    eDmaXferType_t   eTransferType;   /*< The type of transfer. reception or transmission */
    eDmaWordSize_t   eWordSize;       /*< The word size of a transfer, can be 8-bit, 16-bit or 32-bit */
}sDmaParams_t;



/************************************************************************************************************
 *                                     PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
* @brief Initialize the DMA controller for reception or transmission
*
* @param [in] psDmaParams dma parameters structure
*/
void vDmaInit(const sDmaParams_t *psDmaParams);


/**
* @brief Register Write data using DMA
*       
* @param [in] psDmaParams  dma parameters structure
* @param [in] u16TxSize    The lenght of data to write
* @param [in] u32TxBufAddr The array containing data to write
*/
void vDmaRegisterTxData(const sDmaParams_t *psDmaParams, uint16_t u16TxSize, uint32_t u32TxBufAddr);

/**
* @brief Get the current index in the given DMA channel
*
* @param [in] psDmaParams dma parameters structure
*
* @return The number of bytes of data sent or received.
*/
uint16_t u16DMAGetRcvIndex(const sDmaParams_t *psDmaParams);

/**
* @brief Check if the transmission is complete or not
* 
* @param [in] psDmaParams dma parameters structure
* 
* @return true if Tx is completed, false otherwise
*/
bool bDmaTxIsCompleted(const sDmaParams_t *psDmaParams);

/**
 * @brief Check if a transmission is ongoing
 * 
 * @param [in] psDmaParams dma parameters structure
 * @return true is a transmission is ongoing, false otherwise
 */
bool bTxIsOngoing(const sDmaParams_t *psDmaParams);


#endif /* _HW_DESC_DMA_STM32F1_H_ */

/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
