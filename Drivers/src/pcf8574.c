/************************************************************************************************************
* @file	pcf8574.c
* @date	27.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle pcf8574 module
*
************************************************************************************************************/

#include "pcf8574.h"
#include "com_stm32f1.h"
#include "hw_desc_timer_stm32f1.h"


/************************************************************************************************************
*	  DEFINES
************************************************************************************************************/

#define PCF8574_WRITECMD(ADDR)  ((uint8_t)((ADDR) << 1) | (0))
#define PCF8574_READCMD(ADDR)   ((uint8_t)((ADDR) << 1) | (1))
#define TIME_OUT                (uint8_t)10
#define TRIAL_3                 (uint8_t)3



/************************************************************************************************************
*	  STATIC VARIABLES
************************************************************************************************************/

static bool               bPcf8574Initialized = false;
static uint8_t            u8SlaveAdress       = PCF8574_ADDR0;
static uint8_t            u8OutputData        = 0;
static I2C_HandleTypeDef* psLocalI2C          = NULL_PTR;
extern sTIMObj_t sTimerObj;

/************************************************************************************************************
 *  PRIVATE FUNCTIONS DECLARATION
 ***********************************************************************************************************/

/**
 * @brief   check if I2C bus is busy
 * @return  true : i2c bus is busy, false otherwise
 */
static bool bI2CBusBusy(void);


/************************************************************************************************************
 *  PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static bool bI2CBusBusy(void)
{
	/* fetch elapse time */
	uint32_t u32ElapseTime = sTimerObj.pfu32GetTickMs();

	/* Initialization of status flag */
	HAL_StatusTypeDef eStatus = HAL_BUSY;

	while ((eStatus == HAL_BUSY) && (sTimerObj.pfbHasElapsed(u32ElapseTime, TIME_OUT) == false))
	{
		eStatus = HAL_I2C_IsDeviceReady(psLocalI2C, PCF8574_WRITECMD(u8SlaveAdress), TRIAL_3, TIME_OUT);
	}

	return (HAL_BUSY == eStatus);
}


/************************************************************************************************************
 *  PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vPcf8574Init(ePCF8574Address_t eSlaveAddress, I2C_HandleTypeDef *psI2C)
{
	CHECK_PTR(psI2C);

	u8SlaveAdress = (u8)eSlaveAddress;
	psLocalI2C    = psI2C;

	/* check if the device is ready for communication */
	if (HAL_I2C_IsDeviceReady(psI2C, PCF8574_WRITECMD(u8SlaveAdress), TRIAL_3, TIME_OUT) == HAL_OK)
	{
		/* initialization succeeded */
		bPcf8574Initialized = true;
	}
}


bool bPcf8574WritePort(uint8_t u8PortValue)
{
	bool bRet    = false;
    u8OutputData = u8PortValue;

    /* check if the device is ready for communication */
    if ((bPcf8574Initialized == true) && (bI2CBusBusy() == false))
    {
    	/* write the data to the port */
        bRet = (HAL_I2C_Master_Transmit_IT(psLocalI2C, PCF8574_WRITECMD(u8SlaveAdress), &u8OutputData, 1) == HAL_OK);
    }

    return bRet;
}


bool bPcf8574ReadPort(uint8_t *pu8Buffer)
{
    bool bRet = false;

    CHECK_PTR(pu8Buffer)

    if ((bPcf8574Initialized == true) && (bI2CBusBusy() == false))
    {
    	/* read the data from the port */
        bRet =  (HAL_I2C_Master_Receive(psLocalI2C, PCF8574_READCMD(u8SlaveAdress), &pu8Buffer[0], 1, TIME_OUT) == HAL_OK);
    }

    return bRet;
}

