/*************************************************************************************************************
 * @file hw_desc_uart_stm32f1.c
 * @date 11.06.2023
 *************************************************************************************************************
 * @brief UART functions.
 *
 * This file contains functions to handle UART peripherals .
 *
 *
 ************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "hw_desc_uart_stm32f1.h"
#include "stm32f100xe.h"
#include "stm32f1_types.h"



/************************************************************************************************************
*	                          DEFINES 
************************************************************************************************************/
#define CLEAR_REG           (u32)0x0UL
#define FIXED_POINT_POS     (u32)4U

#define FREQ_12MHZ          12000000U
#define FREQ_24MHZ          24000000U

#define RECEIVE_OFFSET      (uint8_t)0
#define TRANSMIT_OFFSET     (uint8_t)1

#define UART_CONF_GPIO(x)   ((0xF << ((x%8)*4)) | (0xF << ((x%8)*4)))
#define UART1_CONFIG_TX     ((0x3 << 4U) | (0x2 << 6U))
#define UART1_CONFIG_RX     (0x1 << 10U)
#define UART2_CONFIG_TX     ((0x3 << 8U) | (0x2 << 10U))
#define UART2_CONFIG_RX     (0x1 << 14U)
#define UART3_CONFIG_TX     ((0x3 << 8U) | (0x2 << 10U))
#define UART3_CONFIG_RX     (0x1 << 14U)



/************************************************************************************************************
*	                          PRIVATE VARIABLES
************************************************************************************************************/
static USART_HWREG* ptrUART = NULL_PTR;
static sDmaParams_t sDmaCtrl[6];
static sUARTObj_t   sUART1;
static sUARTObj_t   sUART2;
static sUARTObj_t   sUART3;
static sUARTObj_t*  sUartObjMap[UART_MAX] = {&sUART1, &sUART2, &sUART3};


/************************************************************************************************************
 *                            PRIVATE FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * \brief Initialize the UART
 * 
 * \param [in] psUARTObj uart object
 */
static void vUartInit(sUARTObj_t* psUARTObj);

/**
 * @brief Initialize the uart peripheral
 * 
 * @param psUARTObj uart object
 */
static void vHwInit(const sUARTObj_t* psUARTObj);

/**
 * @brief Trigger the dma transmission for uart1
 * 
 * @param u16TxSize    The dma transfert size
 * @param u32TxBufAddr The buffer address
 */
static void vTriggerTxID1 (uint16_t u16TxSize, uint32_t u32TxBufAddr);

/**
 * @brief Trigger the dma transmission for uart2
 * 
 * @param u16TxSize    The dma transfert size
 * @param u32TxBufAddr The buffer address
 */
static void vTriggerTxID2 (uint16_t u16TxSize, uint32_t u32TxBufAddr);

/**
 * @brief Trigger the dma transmission for uart3
 * 
 * @param u16TxSize    The dma transfert size
 * @param u32TxBufAddr The buffer address
 */
static void vTriggerTxID3 (uint16_t u16TxSize, uint32_t u32TxBufAddr);


/** @brief Select the correct baudrate
 *
 * @param  psUART    uart peripheral
 * @param  psUARTObj uart object
 */
static void vBaudSelect (USART_HWREG *psUART, const sUARTObj_t* psUARTObj);

/**
 * @brief Configure the dma for uart
 * 
 * @param psUART    uart peripheral
 * @param psUartObj uart object
 */
static void vUartDmaConfig(USART_HWREG *psUART, const sUARTObj_t* psUARTObj);


/**
 * @brief Get the index of the dma reception for uart1
 * 
 * @return the index of the dma transfert
 */
static u16 u16GetRcvIdxID1 (void);

/**
 * @brief Get the index of the dma reception for uart2
 * 
 * @return the index of the dma transfert
 */
static u16 u16GetRcvIdxID2 (void);

/**
 * @brief Get the index of the dma reception for uart3
 * 
 * @return the index of the dma transfert
 */
static u16 u16GetRcvIdxID3 (void);


/**
 * @brief Return the state of the dma transmission for uart1
 * 
 * @return true if the transfert is complete and false otherwise
 */
static bool bDmaTxIsCpltID1 (void);

/**
 * @brief Return the state of the dma transmission for uart2
 * 
 * @return true if the transfert is complete and false otherwise
 */
static bool bDmaTxIsCpltID2 (void);

/**
 * @brief Return the state of the dma transmission for uart3
 * 
 * @return true if the transfert is complete and false otherwise
 */
static bool bDmaTxIsCpltID3 (void);


/**
 * @brief check if there's an ongoing dma transmission on uart1
 * 
 * @return true if a transmission is ongoing, false otherwise
 */
static bool bDmaIsTxOngoingID1 (void);

/**
 * @brief check if there's an ongoing dma transmission on uart2
 * 
 * @return true if a transmission is ongoing, false otherwise
 */
static bool bDmaIsTxOngoingID2 (void);

/**
 * @brief check if there's an ongoing dma transmission on uart3
 * 
 * @return true if a transmission is ongoing, false otherwise
 */
static bool bDmaIsTxOngoingID3 (void);



/************************************************************************************************************
 *                                     PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static void vUartInit(sUARTObj_t* psUARTObj)
{
    CHECK_PTR(psUARTObj);
    
    vHwInit(psUARTObj);  /* UART hardware initialization */

    switch (psUARTObj->sCfg.eID)
    {
        /* assigning the callback functions addresses */
        case UART_ID0:
            psUARTObj->pfvTriggerTx   = vTriggerTxID1;
            psUARTObj->pfu16GetRcvIdx = u16GetRcvIdxID1;
            psUARTObj->pfbTxIsCplt    = bDmaTxIsCpltID1;
            psUARTObj->pfbIsTxOngoing = bDmaIsTxOngoingID1;
            break;

        case UART_ID1:
            psUARTObj->pfvTriggerTx   = vTriggerTxID2;
            psUARTObj->pfu16GetRcvIdx = u16GetRcvIdxID2;
            psUARTObj->pfbTxIsCplt    = bDmaTxIsCpltID2;
            psUARTObj->pfbIsTxOngoing = bDmaIsTxOngoingID2;
            break;

        case UART_ID2:
            psUARTObj->pfvTriggerTx   = vTriggerTxID3;
            psUARTObj->pfu16GetRcvIdx = u16GetRcvIdxID3;
            psUARTObj->pfbTxIsCplt    = bDmaTxIsCpltID3;
            psUARTObj->pfbIsTxOngoing = bDmaIsTxOngoingID3;
            break;

        default: break;
    }
}


static void vHwInit(const sUARTObj_t* psUARTObj)
{
    switch (psUARTObj->sCfg.eID)
    {
        case UART_ID0:
            RCCTRL->APB2ENR |= RCC_APB2ENR_USART1EN;    /* Enable clock for UART1 */
            RCCTRL->APB2ENR |= RCC_APB2ENR_IOPAEN;      /* Enable clock for GPIOA */
            PORTA->CR[1]    &= ~UART_CONF_GPIO(9);      /* Configure the pin 9 of GPIOA to uart1 Tx */
            PORTA->CR[1]    |= UART1_CONFIG_TX;         /* Configure the pin 9 of GPIOA to uart1 Tx */
            PORTA->CR[1]    |= UART1_CONFIG_RX;         /* Configure the pin 10 of GPIOA to uart1 Rx */
            ptrUART          = UART1;                   /* Select the uart1 peripheral */
            break;

        case UART_ID1:
            RCCTRL->APB1ENR |= RCC_APB1ENR_USART2EN;    /* Enable clock for UART2 */
            RCCTRL->APB2ENR |= RCC_APB2ENR_IOPAEN;      /* Enable clock for GPIOA */

            PORTA->CR[0]    &= ~UART_CONF_GPIO(2);      /* Configure the pin 2 of GPIOA for uart2 Tx */
            PORTA->CR[0]    |= UART2_CONFIG_TX;         /* Configure the pin 2 of GPIOA for uart2 Tx */
            PORTA->CR[0]    |= UART2_CONFIG_RX;         /* Configure the pin 3 of GPIOA for uart2 Rx */

            ptrUART         = UART2;                    /* Select the uart1 peripheral */
            break;

        case UART_ID2:
            RCCTRL->APB1ENR |= RCC_APB1ENR_USART3EN;    /* Enable clock for UART3 */
            RCCTRL->APB2ENR |= RCC_APB2ENR_IOPBEN;      /* Enable clock for GPIOB */

            PORTB->CR[1]    &= ~UART_CONF_GPIO(10);     /* Configure the pin 10 of GPIOA for uart3 Tx */
            PORTB->CR[1]    |= UART3_CONFIG_TX;         /* Configure the pin 10 of GPIOB for uart3 Tx */
            PORTB->CR[1]    |= UART3_CONFIG_RX;         /* Configure the pin 11 of GPIOB for uart3 Rx */

            ptrUART         = UART3;                    /* Select the uart1 peripheral */
            break;

        default:  while (1);                            /* Should not happen */
    }

    ptrUART->CR1 |= USART_CR1_UE;                       /* Enable the uart */
    ptrUART->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);  /* Enable DMA for transmission and reception */

    /* Set the uart baudrate */
    vBaudSelect(ptrUART, psUARTObj);

    /* Configure the dma for the given uart peripheral */
    vUartDmaConfig(ptrUART, psUARTObj);

    /* Clear status flags and activate the transmission and reception */
    ptrUART->SR   = CLEAR_REG;
    ptrUART->CR1 |= (USART_CR1_TE | USART_CR1_RE);
}


static void vUartDmaConfig(USART_HWREG *psUART, const sUARTObj_t* psUARTObj)
{
    eDmaChanSel_t eChannel[2] = {0};

    switch (psUARTObj->sCfg.eID)
    {
        case UART_ID0:
            eChannel[RECEIVE_OFFSET]  = DMA_CHANNEL_5;
            eChannel[TRANSMIT_OFFSET] = DMA_CHANNEL_4;
            break;

        case UART_ID1:
            eChannel[RECEIVE_OFFSET]  = DMA_CHANNEL_6;
            eChannel[TRANSMIT_OFFSET] = DMA_CHANNEL_7;
            break;

        case UART_ID2:
            eChannel[RECEIVE_OFFSET]  = DMA_CHANNEL_3;
            eChannel[TRANSMIT_OFFSET] = DMA_CHANNEL_2;
            break;
        
        default: break;
    }

    /* Select the correct channel for the selected uart */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].eDmaChanSel    = eChannel[RECEIVE_OFFSET];
    sDmaCtrl[psUARTObj->sCfg.eID + TRANSMIT_OFFSET].eDmaChanSel   = eChannel[TRANSMIT_OFFSET];

    /* Set the dma transfer word size to 8bits for uart transfers */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].eWordSize      = DMA_WORD_8;
    sDmaCtrl[psUARTObj->sCfg.eID + TRANSMIT_OFFSET].eWordSize     = DMA_WORD_8;

    /* Configure the dma for reception */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].eTransferType  = DMA_RECEIVE;

    /* Configure the dma for transmission */
    sDmaCtrl[psUARTObj->sCfg.eID + TRANSMIT_OFFSET].eTransferType = DMA_TRANSMIT;

    /* Set uart TDR address as source address for reception */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].u32DmaSrcReg   = (u32)&psUART->DR;

    /* Set uart TDR address as destination address for transmission */
    sDmaCtrl[psUARTObj->sCfg.eID + TRANSMIT_OFFSET].u32DmaDesReg  = (u32)&psUART->DR;

    /* Set the dma reception size */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].u16RcvDmaSize  = BUFFER_SIZE;

    /* Set the buffer address for reception */
    sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET].u32RcvAddress  = (u32)psUARTObj->sRxBuffer.u8Data;

    /* Initialize the dma reception */
    vDmaInit(&(sDmaCtrl[psUARTObj->sCfg.eID + RECEIVE_OFFSET]));

    /* Initialize the dma transmission */
    vDmaInit(&(sDmaCtrl[psUARTObj->sCfg.eID + TRANSMIT_OFFSET]));
}


static void vBaudSelect(USART_HWREG *psUART, const sUARTObj_t* psUARTObj)
{
    u16 u16Baud      = 0;
    u32 u32ClockFreq = 0;

    if (psUART == UART1)
    {
        u32ClockFreq = FREQ_24MHZ;
    }
    else if (psUART == UART2 || psUART == UART3)
    {
        u32ClockFreq = FREQ_12MHZ;
    }
    else
    {
        while(1);   /* Should not happen */
    }
    
    /* Compute the baudrate. 4 bits for the decimal part and 12bits for the integer part */
    u16Baud = (u16)((u32ClockFreq << FIXED_POINT_POS) / (psUARTObj->sCfg.eUartParams * 16U));

    psUART->BRR = u16Baud;  /* Set the baudrate */
}


static void vTriggerTxID1(uint16_t u16TxSize, uint32_t u32TxBufAddr)
{
    vDmaRegisterTxData(&sDmaCtrl[UART_ID0 + TRANSMIT_OFFSET], u16TxSize, u32TxBufAddr);
}

static void vTriggerTxID2(uint16_t u16TxSize, uint32_t u32TxBufAddr)
{
    vDmaRegisterTxData(&sDmaCtrl[UART_ID1 + TRANSMIT_OFFSET], u16TxSize, u32TxBufAddr);
}

static void vTriggerTxID3(uint16_t u16TxSize, uint32_t u32TxBufAddr)
{
    vDmaRegisterTxData(&sDmaCtrl[UART_ID2 + TRANSMIT_OFFSET], u16TxSize, u32TxBufAddr);
}


static u16 u16GetRcvIdxID1(void)
{
    return u16DMAGetRcvIndex(&sDmaCtrl[UART_ID0 + RECEIVE_OFFSET]);
}

static u16 u16GetRcvIdxID2(void)
{
    return u16DMAGetRcvIndex(&sDmaCtrl[UART_ID1 + RECEIVE_OFFSET]);
}

static u16 u16GetRcvIdxID3(void)
{
    return u16DMAGetRcvIndex(&sDmaCtrl[UART_ID2 + RECEIVE_OFFSET]);
}


static bool bDmaTxIsCpltID1(void)
{
    return bDmaTxIsCompleted(&sDmaCtrl[UART_ID0 + TRANSMIT_OFFSET]);
}

static bool bDmaTxIsCpltID2(void)
{
    return bDmaTxIsCompleted(&sDmaCtrl[UART_ID1 + TRANSMIT_OFFSET]);
}

static bool bDmaTxIsCpltID3(void)
{
    return bDmaTxIsCompleted(&sDmaCtrl[UART_ID2 + TRANSMIT_OFFSET]);
}


static bool bDmaIsTxOngoingID1(void)
{
    return bTxIsOngoing(&sDmaCtrl[UART_ID0 + TRANSMIT_OFFSET]);
}

static bool bDmaIsTxOngoingID2(void)
{
    return bTxIsOngoing(&sDmaCtrl[UART_ID1 + TRANSMIT_OFFSET]);
}

static bool bDmaIsTxOngoingID3(void)
{
    return bTxIsOngoing(&sDmaCtrl[UART_ID2 + TRANSMIT_OFFSET]);
}



/************************************************************************************************************
 *                            PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vUartHwInit(const sUartCfg_t* psUartCfg)
{
	CHECK_PTR(psUartCfg);
	CHECK_RANGE(psUartCfg->eID, UART_MAX);

	sUARTObj_t* psObj = sUartObjMap[psUartCfg->eID];
	psObj->sCfg.eUartParams = psUartCfg->eUartParams;
	psObj->sCfg.eID = psUartCfg->eID;

    vUartInit(psObj);
}


const sUARTObj_t* GetUARTObj(const sUartCfg_t* psUartCfg)
{
	CHECK_PTR(psUartCfg)

	return sUartObjMap[psUartCfg->eID];
}



/************************************************************************************************************
 *                                            END OF MODULE
 ***********************************************************************************************************/
