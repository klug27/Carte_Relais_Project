/************************************************************************************************************
* @file	   hw_desc_dma_stm32f1.c
* @date	   11.06.2023
*************************************************************************************************************
* @brief   This file contains the functions need to handle the DMA controller
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "hw_desc_dma_stm32f1.h"
#include "stm32f100xe.h"



/************************************************************************************************************
*	                          DEFINES 
************************************************************************************************************/
#define CLEAR_REG       (u32)0x0UL
#define CLEAR_FLAGS     (u32)0xFFFFFFFFUL
#define PSIZE_POS       (u32)8U
#define MSIZE_POS       (u32)10U
#define NUM_OF_CHANNELS (u32)7


/************************************************************************************************************
*	                          PRIVATE VARIABLES
************************************************************************************************************/
static DMA_CHANNEL_HWREG *psDmaChannel[NUM_OF_CHANNELS] = {0};

/* array of dma channels pointers */
static DMA_CHANNEL_HWREG* psDmaAddr[] =
{
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL1_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL2_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL3_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL4_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL5_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL6_ADDRESS,
    (DMA_CHANNEL_HWREG*)DMA1_CHANNEL7_ADDRESS,
};



/************************************************************************************************************
 *                                     PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vDmaInit(const sDmaParams_t *psDmaParams)
{
    CHECK_PTR(psDmaParams);

    /* Enable clock for the DMA1 controller */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* Clear the DMA1 status register */
    DMA1->IFCR   = CLEAR_FLAGS;

    /* Select the DMA channel */
    psDmaChannel[psDmaParams->eDmaChanSel] = psDmaAddr[psDmaParams->eDmaChanSel];

    /* Clear all the channel registers */
    psDmaChannel[psDmaParams->eDmaChanSel]->CCR   = CLEAR_REG;
    psDmaChannel[psDmaParams->eDmaChanSel]->CPAR  = CLEAR_REG;
    psDmaChannel[psDmaParams->eDmaChanSel]->CMAR  = CLEAR_REG;
    psDmaChannel[psDmaParams->eDmaChanSel]->CNDTR = CLEAR_REG;

    /* Disable the channel and wait for it to become correctly disabled */
    psDmaChannel[psDmaParams->eDmaChanSel]->CCR &= ~DMA_CCR_EN;
    while(psDmaChannel[psDmaParams->eDmaChanSel]->CCR & DMA_CCR_EN);

    switch (psDmaParams->eTransferType)
    {
        case DMA_TRANSMIT:
            
            /* Set the peripheral address for transmission */
            psDmaChannel[psDmaParams->eDmaChanSel]->CPAR = psDmaParams->u32DmaDesReg;

            /* Set the transfer priority, the direction memory to peripheral
               and the memory incremented mode */
            psDmaChannel[psDmaParams->eDmaChanSel]->CCR |= (DMA_CCR_PL | DMA_CCR_DIR | DMA_CCR_MINC);

            /* Set the word size of the transmission */
            psDmaChannel[psDmaParams->eDmaChanSel]->CCR |= ((psDmaParams->eWordSize << PSIZE_POS)|
                                                            (psDmaParams->eWordSize << MSIZE_POS));
            break;

        case DMA_RECEIVE:
            
            /* Set the received buffer address for dma reception */
            psDmaChannel[psDmaParams->eDmaChanSel]->CMAR  = psDmaParams->u32RcvAddress;

            /* Set the peripheral address for dma reception */
            psDmaChannel[psDmaParams->eDmaChanSel]->CPAR  = psDmaParams->u32DmaSrcReg;

            /* Set the size of the dma reception */
            psDmaChannel[psDmaParams->eDmaChanSel]->CNDTR = psDmaParams->u16RcvDmaSize;

            /* Set the transfer priority, the memory incremented and circular modes */
            psDmaChannel[psDmaParams->eDmaChanSel]->CCR  |= (DMA_CCR_PL | DMA_CCR_MINC | DMA_CCR_CIRC);
            
            /* Set the word size of the dma reception */
            psDmaChannel[psDmaParams->eDmaChanSel]->CCR  |= ((psDmaParams->eWordSize << PSIZE_POS)|
                                                            (psDmaParams->eWordSize << MSIZE_POS));
             
            /* Activate the dma channel */
            psDmaChannel[psDmaParams->eDmaChanSel]->CCR  |= DMA_CCR_EN;
            break;
        
        default:
        	break;
    }
}


void vDmaRegisterTxData(const sDmaParams_t* psDmaParams, uint16_t  u16TxSize, uint32_t u32TxBufAddr)
{
    if (bTxIsOngoing(psDmaParams) == false)
    {
        /* Disable the channel to prepare the next transfer */
        psDmaChannel[psDmaParams->eDmaChanSel]->CCR &= ~DMA_CCR_EN;

        /* Clear the corresponding dma channel status register */
        DMA->IFCR |= (1U << (((u32)4 * psDmaParams->eDmaChanSel) + (u32)1));

        /* Set the transfer size */
        psDmaChannel[psDmaParams->eDmaChanSel]->CNDTR = u16TxSize;

        /* Set the address of array containing the data to write */
        psDmaChannel[psDmaParams->eDmaChanSel]->CMAR  = u32TxBufAddr;

        /* Activate the dma channel to send the data */
        psDmaChannel[psDmaParams->eDmaChanSel]->CCR  |= DMA_CCR_EN;
    }
}


uint16_t u16DMAGetRcvIndex(const sDmaParams_t *psDmaParams)
{ 
	/* return the current index of the transfer */
    return (u16)(psDmaParams->u16RcvDmaSize -
            psDmaChannel[psDmaParams->eDmaChanSel]->CNDTR);
}


bool bDmaTxIsCompleted(const sDmaParams_t *psDmaParams)
{
	bool bRet = false;

	/* check if the corresponding transfer completed flag is set */
    if ((psDmaChannel[psDmaParams->eDmaChanSel]->CCR & DMA_CCR_EN)&&
        (DMA->ISR & (1U << (((u32)4 * psDmaParams->eDmaChanSel) + (u32)1))))
    {
        bRet = true;	/* transmission is completed */
    }

    return bRet;
}


bool bTxIsOngoing(const sDmaParams_t *psDmaParams)
{
    return (psDmaChannel[psDmaParams->eDmaChanSel]->CNDTR != 0) ? true : false;
}


/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
