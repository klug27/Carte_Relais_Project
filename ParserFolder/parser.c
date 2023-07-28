/************************************************************************************************************
* @file	parser.c
* @date	27.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle uart data
*
************************************************************************************************************/

#include <string.h>
#include <stdio.h>
#include "hw_desc_timer_stm32f1.h"
#include "parser.h"
#include "stm32f1xx_hal.h"
#include "hw_desc_adc_stm32f1.h"
#include "uart_conf.h"
#include "parserFunction.h"
#include "pcf8574.h"
#include "m41t56_rtc.h"


/********************************************************************************************************************
*													                                                                *
*	                  P R I V A T E S     D E F I N I T I O N    O F    C O N S T A N T S                           *
*   													                                                            *
********************************************************************************************************************/



#define  FRAME_LEN                                            (uint16_t)64
#define  DATA_LENGHT_MIN                                      (uint8_t)1
#define  FRAME_NBR                                            (uint16_t)20
#define  HEADER_LEN                                           (uint8_t)3
#define  CMD_LEN                                              (uint8_t)1
#define  BYTE2_LEN                                            (uint8_t)1
#define  ADC_CHANNEL_MAX                                      (uint8_t)3



#define  UART_BUFFER_SIZE                                     (uint16_t)256
#define  UART_TX_BUFFER_SIZE                                  (uint8_t)40
#define  RESP_SIZE                                            (uint8_t)100
#define  TEMP_BUFFER_SIZE                                     (uint8_t)100
#define  UART_TIME_OUT                                        (uint32_t)100
#define  MAX_CMD_FIELD                                        (uint8_t)40
#define  HDR_FIRST_CHAR                                       (uint8_t)'U'
#define  HDR_SECOND_CHAR                                      (uint8_t)'P'
#define  HDR_THIRD_CHAR                                       (uint8_t)'P'
/********************************************************************************************************************
*													                                                                *
*	                                  T Y P E D E F                                                                 *
*   													                                                            *
********************************************************************************************************************/
/* callback function for parser */
typedef void (*vCmdParserFunc_t) (const uint8_t *pu8RxBuffer, uint8_t  *pu8TxBuffer);


/********************************************************************************************************************
*													                                                                *
*	                                  E N U M E R A T I O N                                                         *
*   													                                                            *
********************************************************************************************************************/

/**
 * @enum Command list
 *
 */
typedef enum eCmdList_tTag
{
    CMD_REL_VCC_1     = 0,
    CMD_READ_ADC_4   = 1,
    CMD_UPP_RST_25   = 2,
    CMD_SW_R_REL36   = 3,
    CMD_READ_AIN_37  = 4,
    CMD_I2C_TEST_39  = 5,
    CMD_SET_TIME_48  = 6,
    CMD_GET_TIME_49  = 7,

    CMD_MAX          = 8
} eCmdList_t;


/**
 * @enum Parser state
 *
 */
typedef enum eParserState_tTag
{
	PARSER_STATE_HEADER_ONE     = 0,
	PARSER_STATE_HEADER_TWO     = 1,
	PARSER_STATE_HEADER_THREE   = 2,
	PARSER_STATE_HEADER_SEMI    = 3,
	PARSER_STATE_CMD_FIELD      = 4,
	PARSER_STATE_BYTE2          = 5,
	PARSER_STATE_CMD            = 6,
	PARSER_STATE_DATA           = 7,
	PARSER_STATE_COMPLETE       = 8,
	PARSER_STATE_MAX            = 9

} eParserState_t;

/********************************************************************************************************************
*													                                                                *
*	                                 S T R U C T U R E                                                              *
*   													                                                            *
********************************************************************************************************************/

/**
 * @struct receive frame
 *
 */
typedef struct sRcvFrame_tTag
{
	char           cHeader[HEADER_LEN];      /**< string "PC_" for response command*/
	uint8_t        u8DataLen;                    /**< data length  */
	uint8_t        u8Cmd;                        /**< Command */
	uint8_t        u8Data[FRAME_LEN];      /**< data field */
} sRcvFrame_t;


/**
 * @struct Raw message decode
 *
 */
typedef struct sRawMessageDecode_tTag
{
	eParserState_t  eState;          /**< Parser state */
	uint8_t         u8BufferIndex;   /**< data buffer index */
	uint8_t         u8RcvByteCount;  /**< parsed byte */
	uint32_t        u32LastTime;     /**< current time of timer */
	uint32_t        u32ElapseTime;   /**< uart time out */
	sRcvFrame_t     sFrame;          /**< received frame */
} sRawMessageDecode_t;


typedef struct sTxdFrame_tTag
{
	uint8_t u8Data[FRAME_LEN];
	uint8_t u8Length; 
}sTxdFrame_t ; 


typedef struct sTxdFrameQueue_tTag
{
	sTxdFrame_t sTxdFrameMap[FRAME_NBR]; 
	uint8_t     u8ReadIndex; 
	uint8_t     u8WriteIndex; 
}sTxdFrameQueue_t; 




/********************************************************************************************************************
*													                                                                *
*	          D E C L A R A T I O N        O F        P R I V A T E         V A R I A B L E S                       *
*   													                                                            *
********************************************************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

sADCObj_t                sADCObj;
static sTIMObj_t*        psTimerInst;
static sUARTObj_t*       psUARTObjInst;

static sTxdFrameQueue_t  sUartTxQueue;
static sTxdFrame_t       sUartTxFrame;

static const uint8_t*  u8CmdMap[CMD_MAX] =
{
 [CMD_REL_VCC_1]     = 	(const uint8_t*)  "9;1",    /*Turn VCC*/
 [CMD_READ_ADC_4]    =  (const uint8_t*)  "1;4\r",  /*adc cmd */
 [CMD_UPP_RST_25] 	 =  (const uint8_t*)  "1;25",   /*UPP reset*/
 [CMD_SW_R_REL36]    =  (const uint8_t*)  "8;36",   /*Switch remote relais*/
 [CMD_READ_AIN_37]   =  (const uint8_t*)  "1;37",   /*read analog Input*/
 [CMD_I2C_TEST_39]   =  (const uint8_t*)  "1;39",   /*I2c Test*/
 [CMD_SET_TIME_48]   =  (const uint8_t*)  "2;48",   /*set Unix time*/
 [CMD_GET_TIME_49]   =  (const uint8_t*)  "1;49"    /*get Unix time*/
};

static sRawMessageDecode_t sMsgDecode =
{
	.eState         = PARSER_STATE_HEADER_ONE,
	.u8RcvByteCount = 0,
};


static const vCmdParserFunc_t cmdParser[CMD_MAX] =
{
  [CMD_REL_VCC_1]    =  vFunctionRelaisID1,
  [CMD_READ_ADC_4]   =  vFunctionReadADCID4,
  [CMD_UPP_RST_25]   =  vFunctionUPPID25 ,
  [CMD_SW_R_REL36]   =	vFunctionSWRemoteRelaisID36 ,
  [CMD_READ_AIN_37]	 =  vFunctionReadAnalogInputsID37 ,
  [CMD_I2C_TEST_39]  =  vFunctionI2CTestID39,
  [CMD_SET_TIME_48]  =  vFunctionSetTimeID48,
  [CMD_GET_TIME_49]  =	vFunctionGetTimeID49
};

/********************************************************************************************************************
*													                                                                *
*	          D E C L A R A T I O N        O F     P R I V A T E         F U N C T I O N S                          *
*   													                                                            *
********************************************************************************************************************/

/**
 * @brief   this function decode the receive frame
 * @param   psMsgDecode : pointer to raw frame data
 * @param   ByteIn       : data to read
 * @return      Nothing
 */
static void vMsgDeserialize(sRawMessageDecode_t *psMsgDecode, uint8_t u8ByteIn);


/**
 * @brief Enqueue a frame into the frame queue
 * 
 * @param psFrameQueue frame queue
 */
static void vEnqueueFrame(sTxdFrameQueue_t* psFrameQueue);


/**
 * @brief Dequeue a frame from the frame queue
 * 
 * @param psFrameQueue frame queue
 * @param psTxdFrame   frame to dequeue
 * @return true if the frame was correctly remove in the queue, false otherwise
 */
static bool bDequeueFrame(sTxdFrameQueue_t* psFrameQueue, sTxdFrame_t* psTxdFrame);


/**
 * @brief flush queue
 * 
 * @param psFrameQueue frame queue
 */
static void vFlushQueue(sTxdFrameQueue_t* psFrameQueue);


/**
 * @brief send response
 * 
 */
static void vTcvMsg(void);


/********************************************************************************************************************
*													                                                                *
*	          D E F I N I T I O N        O F     P R I V A T E         F U N C T I O N S                            *
*   													                                                            *
********************************************************************************************************************/

static void vMsgSerialize(const uint8_t* u8MsgData)
{
	eCmdList_t eCmdID = CMD_MAX;
	uint8_t*   u8Ptr  =  NULL_PTR;

	CHECK_PTR(u8MsgData)

	/* search command */
	for(eCmdList_t index = 0; index < CMD_MAX; index++)
	{
		if(strstr((const char*)u8MsgData, (const char*)u8CmdMap[index]) != NULL_PTR)
		{ eCmdID = index; break; }
	}

    /* check if command valid */
	CHECK_RANGE(eCmdID,CMD_MAX)
	
	/* call the function */
	if (cmdParser[eCmdID] != NULL_PTR)
	{
		cmdParser[eCmdID](u8MsgData, &sUartTxFrame.u8Data[0]);
	}

    /* Set pointer */
	u8Ptr = &sUartTxFrame.u8Data[0];

    /* initialization of tx frame length */
	sUartTxFrame.u8Length = 1;

	/* compute tx frame lenght */
	for (int i = 0; u8Ptr[i] != CMD_END; i++)
	{
		sUartTxFrame.u8Length++;
	}

    /* insert data in the queue */
	vEnqueueFrame(&sUartTxQueue);

    /* send the reply */
	vTcvMsg();

}


static void vMsgDeserialize(sRawMessageDecode_t* const psMsgDecode, uint8_t u8ByteIn)
{
	switch (psMsgDecode->eState)
	{
		case PARSER_STATE_COMPLETE :
			/* do nothing */
		  break;

		case PARSER_STATE_HEADER_ONE :
		{
			/* check character U */
			if (u8ByteIn == HDR_FIRST_CHAR)
			{
			  /* resetting the byte counter */
				psMsgDecode->u8RcvByteCount = 0;

				/* resetting the buffer index */
				psMsgDecode->u8BufferIndex = 0;

				/* store the data */
				psMsgDecode->sFrame.cHeader[psMsgDecode->u8RcvByteCount] = u8ByteIn;

				/* increment the byte counter */
				psMsgDecode->u8RcvByteCount++;

				/* go to the next step */
				psMsgDecode->eState = PARSER_STATE_HEADER_TWO ;

				/* Record the current time since the timer was started */
				psMsgDecode->u32LastTime = psTimerInst->pfu32GetTickMs();


				/* Initialization of uart timeout */
				psMsgDecode->u32ElapseTime = 0;

				/* resetting the buffers */
				memset(psMsgDecode->sFrame.u8Data, 0, sizeof(psMsgDecode->sFrame.u8Data));
			}

		}
		break;

		case PARSER_STATE_HEADER_TWO :
		{
			/* compute elapse time */
			psMsgDecode->u32ElapseTime = (psTimerInst->pfu32GetTickMs() - psMsgDecode->u32LastTime);

			/* check if time elapsed */
			if ((psMsgDecode->u32ElapseTime >= UART_TIME_OUT)  || (u8ByteIn != 'P'))
			{
			   psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
			}
			else if (u8ByteIn == HDR_SECOND_CHAR)
			{
				/* recovery of data */
			   psMsgDecode->sFrame.cHeader[psMsgDecode->u8RcvByteCount] = u8ByteIn;

			   /* increment the byte counter */
			   psMsgDecode->u8RcvByteCount++;

			   /* Record the current time since the timer was started */
			   psMsgDecode->u32LastTime = psTimerInst->pfu32GetTickMs();

			   psMsgDecode->eState = PARSER_STATE_HEADER_THREE;
			}

		}
		break;

		case PARSER_STATE_HEADER_THREE:

		psMsgDecode->u32ElapseTime = (psTimerInst->pfu32GetTickMs() - psMsgDecode->u32LastTime);

		if ((psMsgDecode->u32ElapseTime >= UART_TIME_OUT)  || (u8ByteIn != 'P'))
		{
			psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
		}
		else if (u8ByteIn == HDR_THIRD_CHAR)
		{
			/* recovery of data */
			psMsgDecode->sFrame.cHeader[psMsgDecode->u8RcvByteCount] = u8ByteIn;

			/* increment the byte counter */
			psMsgDecode->u8RcvByteCount++;

			/* Record the current time since the timer was started */
			psMsgDecode->u32LastTime = psTimerInst->pfu32GetTickMs();

			psMsgDecode->eState = PARSER_STATE_HEADER_SEMI;
		}
		break;

		case PARSER_STATE_HEADER_SEMI:
		{
			psMsgDecode->u32ElapseTime = (psTimerInst->pfu32GetTickMs() - psMsgDecode->u32LastTime);

			if ((psMsgDecode->u32ElapseTime >= UART_TIME_OUT)  || (u8ByteIn != SEMI))
			{
			    psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
			}
			else if (u8ByteIn == SEMI)
			{
			 /* go to the next step */
			 psMsgDecode->eState = PARSER_STATE_CMD_FIELD;

			 psMsgDecode->sFrame.cHeader[psMsgDecode->u8RcvByteCount] = u8ByteIn;

			 /* Record the current time since the timer was started */
			 psMsgDecode->u32LastTime = psTimerInst->pfu32GetTickMs();

			 psMsgDecode->sFrame.cHeader[psMsgDecode->u8RcvByteCount] = u8ByteIn;

			 /* increment the byte counter */
			 psMsgDecode->u8RcvByteCount = 0;

			/* Record the current time since the timer was started */
			 psMsgDecode->u32LastTime = psTimerInst->pfu32GetTickMs();
			}
		}
		break;

		case PARSER_STATE_CMD_FIELD :
		{
			psMsgDecode->u32ElapseTime = (psTimerInst->pfu32GetTickMs() - psMsgDecode->u32LastTime);

			if (psMsgDecode->u32ElapseTime >= UART_TIME_OUT)
			{
				psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
			}
			else if (psMsgDecode->u8RcvByteCount == MAX_CMD_FIELD)
			{
				psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
			}
			else if (CHECK_CHAR(u8ByteIn) == TRUE)
			{
				psMsgDecode->sFrame.u8Data[psMsgDecode->u8RcvByteCount] = u8ByteIn;
				psMsgDecode->u8RcvByteCount++;
			}
			else if (u8ByteIn == CMD_END)
			{
				psMsgDecode->sFrame.u8Data[psMsgDecode->u8RcvByteCount]  = u8ByteIn;
				psMsgDecode->eState = PARSER_STATE_COMPLETE;
			}
			else if (CHECK_CHAR(u8ByteIn) == FALSE)
			{
				/* return to the initial state if character is not valid */
				psMsgDecode->eState = PARSER_STATE_HEADER_ONE;
			}
		}
		break;

		default : break;
	}
}


static bool bDequeueFrame(sTxdFrameQueue_t* psFrameQueue, sTxdFrame_t* psTxdFrame)
{
    bool bRetVal = false;

    if (psFrameQueue->u8ReadIndex != psFrameQueue->u8WriteIndex)
    {
		/* retrieve data from the queue */
		memcpy(psTxdFrame, &psFrameQueue->sTxdFrameMap[psFrameQueue->u8ReadIndex], FRAME_LEN);

		/* increment index */
        psFrameQueue->u8ReadIndex++;

		/* check if queue is full */
        CHECK_OVERRUN(psFrameQueue->u8ReadIndex, FRAME_NBR, 0)

        bRetVal = true;
    }

    return bRetVal;
}


static void vEnqueueFrame(sTxdFrameQueue_t* psFrameQueue)
{
	/* insert data in the queue */
	memcpy(&psFrameQueue->sTxdFrameMap[psFrameQueue->u8WriteIndex], &sUartTxFrame, FRAME_LEN);

	/* increment index */
	psFrameQueue->u8WriteIndex++;

	/* check if queue is full */
	CHECK_OVERRUN(psFrameQueue->u8WriteIndex, FRAME_NBR, 0)
}


static void vFlushQueue(sTxdFrameQueue_t* psFrameQueue)
{
	psFrameQueue->u8ReadIndex = psFrameQueue->u8WriteIndex;
}


static void vTcvMsg(void)
{
	/* check that there is no transfer in progress and send the reply */
	if (psUARTObjInst->pfbIsTxOngoing() == false)
	{
		/* retrieve data from queue to the frame buffer */
		if (bDequeueFrame(&sUartTxQueue, &sUartTxFrame) == TRUE)
		{
			/* send data */
	    	psUARTObjInst->pfvTriggerTx(sUartTxFrame.u8Length, (uint32_t)&sUartTxFrame.u8Data[0]);
		}
	}
}



/********************************************************************************************************************
*													                                                                *
*	          D E F I N I T I O N        O F     P U B L I C         F U N C T I O N S                              *
*   													                                                            *
********************************************************************************************************************/

void vRcvMsg(void)
{
	  uint32_t u32CurrTime = psTimerInst->pfu32GetTickMs();

	/* initialize variable to parse received data */
	psUARTObjInst->sRxBuffer.u16WriteIndex  = psUARTObjInst->pfu16GetRcvIdx();

	while (psUARTObjInst->sRxBuffer.u16ReadIndex != psUARTObjInst->sRxBuffer.u16WriteIndex)
	{
		/* Parse receive data */
		vMsgDeserialize(&sMsgDecode, psUARTObjInst->sRxBuffer.u8Data[psUARTObjInst->sRxBuffer.u16ReadIndex]);

		if (sMsgDecode.eState == PARSER_STATE_COMPLETE)
		{
			vMsgSerialize(&sMsgDecode.sFrame.u8Data[0]);
			
			/* Initialization of parser state */
			sMsgDecode.eState = PARSER_STATE_HEADER_ONE;
		}

		/* When we try to increment read index and we reach the end of buffer,
		 * we start from the beginning of the buffer */
		if (psUARTObjInst->sRxBuffer.u16ReadIndex == (BUFFER_SIZE - 1))
		{
			psUARTObjInst->sRxBuffer.u16ReadIndex = 0;
		}
		else
		{
			psUARTObjInst->sRxBuffer.u16ReadIndex++;
		}

		if (psUARTObjInst->sRxBuffer.u16ReadIndex == psUARTObjInst->sRxBuffer.u16WriteIndex)
		{
			 uint32_t u32Elapsetime = psTimerInst->pfu32GetTickMs();
			 u32Elapsetime -= u32CurrTime;
			 u32CurrTime = 0;
		}
	}
}


void vParserInit(const sTIMObj_t *psTimerObj, const sUARTObj_t* psUartObj)
{
	/* check the uart and timers objects */
	CHECK_PTR(psTimerObj);
	CHECK_PTR(psUartObj);

	/* reset the buffers */
	memset((void*)&sMsgDecode, 0, sizeof(sRawMessageDecode_t));

	/* reset tx queue and tx frame */
	memset((void*)&sUartTxQueue, 0, sizeof(sTxdFrameQueue_t));
	memset((void*)&sUartTxFrame, 0, sizeof(sTxdFrame_t));

	/* fetch timer object */
	psTimerInst   = psTimerObj;
	psUARTObjInst = psUartObj;

	/* Initialization of ADC */
	vADCInit(&sADCObj, psTimerObj);

    /* Initialization of port expander */
    vPcf8574Init(PCF8574_ADDR0, &hi2c1);

    /* Initialization of RTC */
    bM41t56Init(&hi2c1);

	/* Initialization of UPP GPIO */
	vUPPGPIOInit();
}
