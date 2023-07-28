/************************************************************************************************************
* @file	parserFunction.c
* @date	27.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle command
*
************************************************************************************************************/


/************************************************************************************************************
*	  INCLUDES
************************************************************************************************************/
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include "parserFunction.h"
#include "pcf8574.h"
#include "m41t56_rtc.h"
#include "hw_desc_gpio_stm32f1.h"
#include "hw_desc_adc_stm32f1.h"



/************************************************************************************************************
*	  DEFINES
************************************************************************************************************/

#define  PCF8574_PIN_MAX                                      (uint8_t)8
#define  CMD_RELAIS_OFFSET                                    (uint8_t)4   
#define  CMD_READ_ADC_OFFSET                                  (uint8_t)3
#define  CMD_UPP_RST_OFFSET                                   (uint8_t)4
#define  CMD_SW_R_REL_OFFSET                                  (uint8_t)5
#define  CMD_READ_AIN_OFFSET                                  (uint8_t)4
#define  CMD_I2C_TEST_OFFSET                                  (uint8_t)4
#define  CMD_GET_TIME_OFFSET                                  (uint8_t)4

#define  CMD_RELAIS_LEN                                       (uint8_t)8

#define  RELAIS_1_PIN									      GPIO_PIN4
#define  RELAIS_2_PIN									      GPIO_PIN5
#define  RELAIS_3_PIN									      GPIO_PIN6
#define  RELAIS_4_PIN									      GPIO_PIN7
#define  RELAIS_5_PIN									      GPIO_PIN4
#define  RELAIS_6_PIN									      GPIO_PIN5
#define  RELAIS_VCC_PIN									      GPIO_PIN0
#define  RELAIS_8_PIN									      GPIO_PIN1

#define  RELAIS_1_PIN									      GPIO_PIN4
#define  RELAIS_2_PIN									      GPIO_PIN5
#define  RELAIS_3_PIN									      GPIO_PIN6
#define  RELAIS_4_PIN									      GPIO_PIN7
#define  RELAIS_5_PIN									      GPIO_PIN4
#define  RELAIS_6_PIN									      GPIO_PIN5
#define  RELAIS_VCC_PIN									      GPIO_PIN0
#define  RELAIS_8_PIN									      GPIO_PIN1

#define  VCC_X320_8_PIN                                       GPIO_PIN5
#define  VCC_X320_8_PORT                                      GPIO_PORTC

#define  RL_V3V1_X330_1_PIN                                   GPIO_PIN2                           
#define  RL_V3V2_X330_2_PIN                                   GPIO_PIN7
#define  RL_PA_X330_3_PIN                                     GPIO_PIN8
#define  RL_ESC_X330_4_PIN                                    GPIO_PIN9
#define  RL_AUX1_X330_7_PIN                                   GPIO_PIN10
#define  RL_AUX2_X330_8_PIN                                   GPIO_PIN11
#define  RL_AUX3_X330_9_PIN                                   GPIO_PIN12

#define  Z1_X360_1_PIN                                        GPIO_PIN6
#define  Z2_X360_2_PIN                                        GPIO_PIN7
#define  Z3_X360_3_PIN                                        GPIO_PIN8
#define  Z4_X360_4_PIN                                        GPIO_PIN9
#define  Z5_X380_1_PIN                                        GPIO_PIN4
#define  Z6_X380_2_PIN                                        GPIO_PIN5
#define  Z7_X380_3_PIN                                        GPIO_PIN6

#define  RL_V3V1_X330_1_PORT                                  GPIO_PORTB                           
#define  RL_V3V2_X330_2_PORT                                  GPIO_PORTE
#define  RL_PA_X330_3_PORT                                    GPIO_PORTE
#define  RL_ESC_X330_4_PORT                                   GPIO_PORTE
#define  RL_AUX1_X330_7_PORT                                  GPIO_PORTE
#define  RL_AUX2_X330_8_PORT                                  GPIO_PORTE
#define  RL_AUX3_X330_9_PORT                                  GPIO_PORTE

#define  Z1_X360_1_PORT                                       GPIO_PORTC
#define  Z2_X360_2_PORT                                       GPIO_PORTC
#define  Z3_X360_3_PORT                                       GPIO_PORTC
#define  Z4_X360_4_PORT                                       GPIO_PORTC
#define  Z5_X380_1_PORT                                       GPIO_PORTD
#define  Z6_X380_2_PORT                                       GPIO_PORTD
#define  Z7_X380_3_PORT                                       GPIO_PORTD

#define  RELAIS_1_PORT                                        GPIO_PORTA
#define  RELAIS_2_PORT                                        GPIO_PORTA
#define  RELAIS_3_PORT                                        GPIO_PORTA
#define  RELAIS_4_PORT                                        GPIO_PORTA
#define  RELAIS_5_PORT                                        GPIO_PORTC
#define  RELAIS_6_PORT                                        GPIO_PORTC
#define  RELAIS_VCC_PORT                                      GPIO_PORTB
#define  RELAIS_8_PORT                                        GPIO_PORTB

#define  TURN_OUTPUT_OFF                                      (uint8_t)0
#define  TURN_OFF_RELAIS                                      (uint8_t)0x7F
#define  TURN_OUTPUT_ON                                       (uint8_t)128
#define  WRITE_RELAIS_OK                                      (uint8_t)0
#define  TURN_RELAIS_ON                                       (uint8_t)128
#define  TURN_RELAIS_OFF                                      (uint8_t)0
#define  TURN_RELAIS_ON_DELAY_MIN                             (uint8_t)1
#define  TURN_RELAIS_ON_DELAY_MAX                             (uint8_t)126
#define  TIME_OUT                                             (uint8_t)5
#define  TRIAL                                                (uint8_t)3
#define  M41T56_ADDR                                          (uint16_t)0xD0         /* M41T56 I2C control byte */
#define  CH_MAX                                               (uint8_t)3


#define  UNUSED_PARAMETER(PARAM)                              ((void)(PARAM))
#define  GET_PIN_LEVEL(LEVEL)                                 ((LEVEL) == TRUE) ? 1 : 0
#define  ONE_TIME                                             (uint8_t)1

/************************************************************************************************************
*	  STRUCTURES
************************************************************************************************************/

typedef struct sRelVal_tTag
{
	uint8_t u8String[CH_MAX] ;
	uint8_t u8Val;

} sRelVal_t;


/************************************************************************************************************
*	  ENUMERATIONS
************************************************************************************************************/

typedef enum eRelay_tTag
{
	RL_AUX1_POS     = 0,
	RL_ESC_POS      = 1,
	RL_PA_POS       = 2,
	RL_AUX2_POS     = 3,
	RL_AUX3_POS     = 4,
	RL_V3V1_POS     = 5,
	RL_V3V2_POS     = 6,

	RL_POS_MAX      = 7
} eRelay_t;

/************************************************************************************************************
*	  EXTERN VARIABLES
************************************************************************************************************/
extern sADCObj_t         sADCObj;
extern sTIMObj_t         sTimerObj;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;



/************************************************************************************************************
*	  PRIVATE VARIABLES
************************************************************************************************************/
static sLongType_t sUnixTime;
static sLongType_t sAdcData = {0};
static sGPIObj_t   sGPIObj[5];
static uint8_t     u8Mapp[7] =
{
	RL_V3V1_POS,
	RL_V3V2_POS, 
	RL_PA_POS, 
	RL_ESC_POS, 
	RL_AUX1_POS, 
	RL_AUX2_POS, 
	RL_AUX3_POS
 };


/************************************************************************************************************
*	  PRIVATE FUNCTIONS DECLARATIONS
************************************************************************************************************/


/**
 * @brief  convert input string data to integer
 */
static bool bComputeData(uint8_t u8RlNumber, sRelVal_t* sRLMap ,const uint8_t* pu8InputData);


/**
 * @brief This function is used to turn off relai VCC
 */
static void vTurnOffVcc(void);


/************************************************************************************************************
*	  PRIVATE FUNCTIONS DEFINITION
************************************************************************************************************/

/**
 * @brief This function is used to turn off relai VCC
 */
static void vTurnOffVcc(void)
{
	sGPIObj[3].pfvResetPin(VCC_X320_8_PORT, VCC_X320_8_PIN);
}


/**
 * @brief  convert input string data to integer
 */
static bool bComputeData(uint8_t u8RlNumber, sRelVal_t* sRLMap ,const uint8_t* pu8InputData)
{
	uint8_t     u8Idx  = 0;
	uint8_t     u8RelIndex = 0;
	uint8_t     u8ValPosition = 0;
	sRelVal_t*  psRelVal =  &sRLMap[0];
	bool        bRet = TRUE;

    /* Extract all substrings that contain numbers */
    while (pu8InputData[u8Idx] != '\r')
	{
		if (pu8InputData[u8Idx] == ';')
		{
			u8RelIndex++;
			u8ValPosition = 0;

			/* check index */
			if (u8RelIndex >= u8RlNumber)
			{
				bRet = FALSE;
				break;
			}

			psRelVal = &sRLMap[u8RelIndex];
		}
		else
		{
			/* check index */
			if (u8ValPosition >= CH_MAX)
			{
				bRet = FALSE;
				break;
			}

			/* fetch number */
			psRelVal->u8String[u8ValPosition] = pu8InputData[u8Idx];
			u8ValPosition++;
		}

		u8Idx++;
	}

	if (bRet == TRUE)
	{
		/* convert each substring to integer */
		for (u8Idx = 0; u8Idx < u8RlNumber; u8Idx++)
		{
			psRelVal = &sRLMap[u8Idx];
			psRelVal->u8Val = (uint8_t)atoi((char*)psRelVal->u8String);
		}
	}

	return bRet;
}



/************************************************************************************************************
*	  PUBLIC FUNCTIONS DEFINITION
************************************************************************************************************/
void vUPPGPIOInit(void)
{
	/* Initialization of GPIO ports */
	vGPIOInit(&sGPIObj[0], GPIO_PORTE);
	vGPIOInit(&sGPIObj[1], GPIO_PORTB);
	vGPIOInit(&sGPIObj[2], GPIO_PORTD);
	vGPIOInit(&sGPIObj[3], GPIO_PORTC);
	vGPIOInit(&sGPIObj[4], GPIO_PORTA);

	/* Configuration of relays pins as ouputs pins on UPP */
	sGPIObj[4].pfvCfgOutput(RELAIS_1_PORT,   RELAIS_1_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_2_PORT,   RELAIS_2_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_3_PORT,   RELAIS_3_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_4_PORT,   RELAIS_4_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[3].pfvCfgOutput(RELAIS_5_PORT,   RELAIS_5_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[3].pfvCfgOutput(RELAIS_6_PORT,   RELAIS_6_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[1].pfvCfgOutput(RELAIS_VCC_PORT, RELAIS_VCC_PIN, OUTPUT_PUSH_PULL);
	sGPIObj[1].pfvCfgOutput(RELAIS_8_PORT,   RELAIS_8_PIN,   OUTPUT_PUSH_PULL);

	/* Configuration of remote relays pins as input pins on UPP */
	sGPIObj[1].pfvCfgInput(RL_V3V1_X330_1_PORT, RL_V3V1_X330_1_PIN, INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_V3V2_X330_2_PORT, RL_V3V2_X330_2_PIN, INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_PA_X330_3_PORT  , RL_PA_X330_3_PIN  , INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_ESC_X330_4_PORT , RL_ESC_X330_4_PIN , INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_AUX1_X330_7_PORT, RL_AUX1_X330_7_PIN, INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_AUX2_X330_8_PORT, RL_AUX2_X330_8_PIN, INPUT_PULL_DOWN);
	sGPIObj[0].pfvCfgInput(RL_AUX3_X330_9_PORT, RL_AUX3_X330_9_PIN, INPUT_PULL_DOWN);

	/* Configuration of varistors pins as input pins on UPP */
	sGPIObj[3].pfvCfgInput(Z1_X360_1_PORT, Z1_X360_1_PIN, INPUT_PULL_DOWN);
	sGPIObj[3].pfvCfgInput(Z2_X360_2_PORT, Z2_X360_2_PIN, INPUT_PULL_DOWN);
	sGPIObj[3].pfvCfgInput(Z3_X360_3_PORT, Z3_X360_3_PIN, INPUT_PULL_DOWN);
	sGPIObj[3].pfvCfgInput(Z4_X360_4_PORT, Z4_X360_4_PIN, INPUT_PULL_DOWN);
	sGPIObj[2].pfvCfgInput(Z5_X380_1_PORT, Z5_X380_1_PIN, INPUT_PULL_DOWN);
	sGPIObj[2].pfvCfgInput(Z6_X380_2_PORT, Z6_X380_2_PIN, INPUT_PULL_DOWN);
	sGPIObj[2].pfvCfgInput(Z7_X380_3_PORT, Z7_X380_3_PIN, INPUT_PULL_DOWN);

	/* Configuration of relays pins as ouputs pins on UPP */
	sGPIObj[4].pfvCfgOutput(RELAIS_1_PORT,   RELAIS_1_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_2_PORT,   RELAIS_2_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_3_PORT,   RELAIS_3_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[4].pfvCfgOutput(RELAIS_4_PORT,   RELAIS_4_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[3].pfvCfgOutput(RELAIS_5_PORT,   RELAIS_5_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[3].pfvCfgOutput(RELAIS_6_PORT,   RELAIS_6_PIN,   OUTPUT_PUSH_PULL);
	sGPIObj[1].pfvCfgOutput(RELAIS_VCC_PORT, RELAIS_VCC_PIN, OUTPUT_PUSH_PULL);
	sGPIObj[1].pfvCfgOutput(RELAIS_8_PORT,   RELAIS_8_PIN,   OUTPUT_PUSH_PULL);
}


void vFunctionRelaisID1(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	#define MAX_RELAIS 8
	#define OFFSET     4
	#define RL_VCC_POS 6

	sRelVal_t   sRelValMap[MAX_RELAIS] = {0};
	uint32_t    u32Delay = 0;
	bool        bRet = FALSE;

    /* Convert data to string */
	bRet = bComputeData(MAX_RELAIS, &sRelValMap[0], &pu8RxBuffer[OFFSET]);

	if (bRet == FALSE)
	{
		/* send error */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;0;0;0;0;0;0;-1;0\r");
	}
	else
	{
		if (sRelValMap[RL_VCC_POS].u8Val  == TURN_RELAIS_OFF)
		{
			/* Turn off VCC */
			sGPIObj[3].pfvResetPin(VCC_X320_8_PORT, VCC_X320_8_PIN);
		}
		else if (sRelValMap[RL_VCC_POS].u8Val == TURN_RELAIS_ON)
		{
			/* Turn on VCC */
			sGPIObj[3].pfvSetPin(VCC_X320_8_PORT, VCC_X320_8_PIN);
		}
		else if ((sRelValMap[RL_VCC_POS].u8Val >= TURN_RELAIS_ON_DELAY_MIN) &&
				(sRelValMap[RL_VCC_POS].u8Val <= TURN_RELAIS_ON_DELAY_MAX))
		{
			/* fetch delay */
			u32Delay = sRelValMap[RL_VCC_POS].u8Val;

			/* Turn on VCC */
			sGPIObj[3].pfvSetPin(VCC_X320_8_PORT, VCC_X320_8_PIN);

			/* set delay and turn off VCC */
			sTimerObj.pfbRegisterCbk(vTurnOffVcc, u32Delay, ONE_TIME);
		}

		/* put command */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;%d;%d;%d;%d;%d;%d;%d;%d\r",
							sRelValMap[0].u8Val,sRelValMap[1].u8Val,sRelValMap[2].u8Val,
							sRelValMap[3].u8Val,sRelValMap[4].u8Val,sRelValMap[5].u8Val,
							sRelValMap[6].u8Val,sRelValMap[7].u8Val);
	}
}


void vFunctionReadADCID4(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	(void)(pu8RxBuffer);

	/* fetch adc conversion results */
	sADCObj.pfvGetConvResult(&sAdcData.u16Word[0]);

	if (sAdcData.u64LongWord != 0)
	{
	  sprintf((char*)pu8TxBuffer,"PC_;3;4;0;%hu;%hu;%hu\r",sAdcData.u16Word[0],sAdcData.u16Word[1],sAdcData.u16Word[2]);
	}
	else
	{ /* set error flag and return the default values */
	  sprintf((char*)pu8TxBuffer,"PC_;3;4;-1;%d;%d;%d\r",sAdcData.u16Word[0],sAdcData.u16Word[1],sAdcData.u16Word[2]);
	}

}


void vFunctionUPPID25(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	UNUSED_PARAMETER(pu8RxBuffer);
	sGPIObj[3].pfvResetPin(VCC_X320_8_PORT, VCC_X320_8_PIN);
	sprintf((char*)pu8TxBuffer,"PC_;9;1;0;0;0;0;0;0;0;0\r");
}


void vFunctionSWRemoteRelaisID36(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	#define OFFSET_REMOTE_RELAIS 5
	#define MAX_REMOTE_RELAIS    7

	bool bRet = FALSE;
	uint8_t u8OutputData = 0;
	sRelVal_t sRelValMap[MAX_REMOTE_RELAIS] = { 0 };

    /* convert data to integer */
	bRet = bComputeData(MAX_REMOTE_RELAIS, &sRelValMap[0], &pu8RxBuffer[OFFSET_REMOTE_RELAIS]);

	if (bRet == FALSE)
	{
		/* Send error status */
		sprintf((char*) pu8TxBuffer, "PC_;2;36;-2\r");
	}
	else
	{
		/* Read port */
		bPcf8574ReadPort(&u8OutputData);

		for (uint8_t u8Index = 0; u8Index < MAX_REMOTE_RELAIS; u8Index++)
		{
			if (sRelValMap[u8Index].u8Val == TURN_OUTPUT_OFF)
			{
				/* turn output off */
				u8OutputData |= (1 << u8Mapp[u8Index]);
			}
			else if (sRelValMap[u8Index].u8Val == TURN_OUTPUT_ON)
			{
				/* turn output on */
				u8OutputData &= ~(1 << u8Mapp[u8Index]);

			}
		}

		/* write data on the port */
		bRet = bPcf8574WritePort(u8OutputData);

		if (bRet == TRUE)
		{
			/* write operation was done successfully */
			sprintf((char*) pu8TxBuffer, "PC_;2;36;0\r");
		}
		else
		{
			/* Send error status */
			sprintf((char*) pu8TxBuffer, "PC_;2;36;-2\r");
		}
	}

	
}


void vFunctionReadAnalogInputsID37(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	#define  ANALOG_INPUT_LEN  (uint8_t)16

	char cTemp[4]      = {0};
	u8   u8TempBuf[16] = {0};

	/* set status to ok */
	u8TempBuf[0]  = WRITE_RELAIS_OK;

	/* read relays states */
	u8TempBuf[1]  = GET_PIN_LEVEL(sGPIObj[1].pfbReadPin(RL_V3V1_X330_1_PORT, RL_V3V1_X330_1_PIN));
	u8TempBuf[2]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_V3V2_X330_2_PORT, RL_V3V2_X330_2_PIN));
	u8TempBuf[3]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_PA_X330_3_PORT  , RL_PA_X330_3_PIN));
	u8TempBuf[4]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_ESC_X330_4_PORT , RL_ESC_X330_4_PIN));
	u8TempBuf[5]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_AUX1_X330_7_PORT, RL_AUX1_X330_7_PIN));
	u8TempBuf[6]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_AUX2_X330_8_PORT, RL_AUX2_X330_8_PIN));
	u8TempBuf[7]  = GET_PIN_LEVEL(sGPIObj[0].pfbReadPin(RL_AUX3_X330_9_PORT, RL_AUX3_X330_9_PIN));

	/* read varistors states */
	u8TempBuf[8]  = GET_PIN_LEVEL(sGPIObj[3].pfbReadPin(Z1_X360_1_PORT, Z1_X360_1_PIN));
	u8TempBuf[9]  = GET_PIN_LEVEL(sGPIObj[3].pfbReadPin(Z2_X360_2_PORT, Z2_X360_2_PIN));
	u8TempBuf[10] = GET_PIN_LEVEL(sGPIObj[3].pfbReadPin(Z3_X360_3_PORT, Z3_X360_3_PIN));
	u8TempBuf[11] = GET_PIN_LEVEL(sGPIObj[3].pfbReadPin(Z4_X360_4_PORT, Z4_X360_4_PIN));
	u8TempBuf[12] = GET_PIN_LEVEL(sGPIObj[2].pfbReadPin(Z5_X380_1_PORT, Z5_X380_1_PIN));
	u8TempBuf[13] = GET_PIN_LEVEL(sGPIObj[2].pfbReadPin(Z6_X380_2_PORT, Z6_X380_2_PIN));
	u8TempBuf[15] = GET_PIN_LEVEL(sGPIObj[2].pfbReadPin(Z7_X380_3_PORT, Z7_X380_3_PIN));

	/* put command */
	sprintf((char*)pu8TxBuffer, "%s", "PC_;9;37");

	/* filling the output buffer */
	for (u8 u8Index = 0; u8Index < ANALOG_INPUT_LEN; u8Index++)
	{
		/* resetting the temporary buffer */
		memset(cTemp, 0, sizeof(cTemp));

		/* put the semi-colon */
		strcat((char*)pu8TxBuffer, ";");

		/* convert the read input state into string */
		sprintf(&cTemp[0], "%d", u8TempBuf[u8Index]);

		/* add the converted number in the output buffer */
		strcat((char*)pu8TxBuffer, &cTemp[0]);
	}

	/* add the end of frame character */
	pu8TxBuffer[strlen((char*)pu8TxBuffer)] = '\r';

	UNUSED_PARAMETER(pu8RxBuffer);
}


void vFunctionI2CTestID39(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	uint8_t u8DummyRead = 0;
	bool bRTC_OK        = FALSE;
	bool bRELAIS_OK     = FALSE;

	/* Read port */
	bRELAIS_OK = bPcf8574ReadPort(&u8DummyRead);

	/* Read RTC */
	bRTC_OK = (HAL_I2C_IsDeviceReady(&hi2c1, M41T56_ADDR, TRIAL, TIME_OUT) == HAL_OK);

	if ((bRELAIS_OK == TRUE) && (bRTC_OK == TRUE))
	{
		/* put command */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;0\r");
	}
	else if ((bRTC_OK == FALSE) && (bRELAIS_OK == FALSE))
	{
		/* put command */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;-3\r");
	}
	else if (!bRTC_OK)
	{
		/* put command */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;-2\r");
	}
	else if (!bRELAIS_OK)
	{
		/* put command */
		sprintf((char*)pu8TxBuffer, "PC_;9;1;-1\r");
	}

	UNUSED_PARAMETER(pu8RxBuffer);
}


void vFunctionSetTimeID48(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
    #define UNIX_TIME_LEN        10
	#define DECIMAL_BASE         10
	#define CMD_SET_TIME_OFFSET  5

	const uint8_t* u8Ptr     = &pu8RxBuffer[CMD_SET_TIME_OFFSET];
	bool           bRet      = FALSE;
	char           cTemp[11] = "";
	uint8_t        u8Cnt     = 0;

    /* extract the UNIX time substring from the command */
	for (uint8_t u8Index = 0; u8Ptr[u8Index]!= CMD_END; u8Index++)
	{
		if ((u8Ptr[u8Index] >= START_NBR) && (u8Ptr[u8Index] <= END_NBR))
		{
			/* retrieve number */
			cTemp[u8Index] = u8Ptr[u8Index];

			/* count the number of retrieved number */
			u8Cnt++;
		}
		else
		{
			break;
		}
	}

	/* we make sure that we have recovered ten values */
	if (u8Cnt == UNIX_TIME_LEN)
	{
		/* convert unix time to integer */
		sUnixTime.u32LongWord[0] = strtoul(cTemp, NULL_PTR, DECIMAL_BASE);

		/* Set the unix time */
		bRet = bM41t56SetTime(sUnixTime.u32LongWord[0]);
	}

	/* Check for any error */
	if (bRet == FALSE)
	{
		sprintf((char*)pu8TxBuffer,"PC_;2;48;-1\r");
	}
	else
	{
		sprintf((char*)pu8TxBuffer,"PC_;2;48;0\r");
	}
}


void vFunctionGetTimeID49(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer)
{
	/* Initialization of variables */
	sUnixTime.u64LongWord = 0;

	/* Get the unix time */
	sUnixTime.u32LongWord[0] = u32M41t56GetTime();

	/* Check for any error */
	if (sUnixTime.u32LongWord[0] == 0)
	{
		sprintf((char*)pu8TxBuffer,"PC_;3;49;-1;0\r");
	}
	else
	{
		sprintf((char*)pu8TxBuffer,"PC_;3;49;0;%" PRIu32 "\r", sUnixTime.u32LongWord[0]);
	}

	UNUSED_PARAMETER(pu8RxBuffer);
}
