/************************************************************************************************************
* @file	m41t56_rtc.c
* @date	27.06.2023
*************************************************************************************************************
* @brief This file contains some functions to handle the rtc module
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "m41t56_rtc.h"
#include "stm32f1_types.h"



/************************************************************************************************************
*	                          DEFINES 
************************************************************************************************************/
#define READ                 (uint8_t)0
#define WRITE                (uint8_t)1

#define TRIALS_3             (uint32_t)3
#define LENGHT7              (uint16_t)7
#define TIMEOUT_10MS         (uint32_t)10
#define TIMEOUT_100MS        (uint32_t)100
#define UNIX_YEAR            (uint32_t)1970
#define HOUR_BASE            (uint32_t)24
#define SEC_MIN_BASE         (uint32_t)60
#define MIL_BASE             (uint32_t)2000
#define SEC_IN_LEAP_YEAR     (uint32_t)31622400
#define SEC_IN_ORDI_YEAR     (uint32_t)31536000
#define SEC_IN_ONE_DAY       (uint32_t)86400
#define SEC_IN_ONE_HOUR      (uint32_t)3600
#define NUM_OF_RTC_REG       (uint8_t)7

#define SECS                 (uint8_t)0
#define MINS                 (uint8_t)1
#define HOUR                 (uint8_t)2
#define W_DAY                (uint8_t)3
#define DAY                  (uint8_t)4
#define MNTH                 (uint8_t)5
#define YEAR                 (uint8_t)6

#define DEC_BASE             (uint8_t)10
#define SEC_MASK             (uint8_t)0x7F
#define DAY_MASK             (uint8_t)0x07
#define HOUR_MASK            (uint8_t)0x3F
#define DATE_MASK            (uint8_t)0x7F
#define HALF_BYTE            (uint8_t)0x04
#define MONTH_MASK           (uint8_t)0x1F
#define HALF_BYTE_MSK        (uint8_t)0x0F

#define RTCSEC_ADDR          (uint8_t)0x00          /* RTC Seconds register address */
#define CONTROL_ADDR         (uint8_t)0x07          /* RTC control register address */
#define M41T56_ADDR          (uint16_t)0xD0         /* M41T56 I2C control byte */



/************************************************************************************************************
*	                          DECLARATION OF STRUCTURES AND ENUMERATIONS
************************************************************************************************************/

/* Time structure */
typedef struct
{
    uint32_t   u32Sec;    /* Current seconds */
    uint32_t   u32Min;    /* Current minutes */
    uint32_t   u32Hour;   /* Current hour */
    uint32_t   u32Day;    /* Current day */
    uint32_t   u32Month;  /* Month of the year */
    uint32_t   u32Year;   /* current year */
    uint32_t   u32WkDay;  /* Day of the week */
}sTime_t;



/************************************************************************************************************
*	                         PRIVATE VARIABLES
************************************************************************************************************/
static I2C_HandleTypeDef* psLocalI2C                  = NULL;
static sTime_t            localTime                   =  {0};
static bool               bM41t56Initialized          = false;
static uint8_t            u8ReadTime[NUM_OF_RTC_REG]  =  {0};
static uint8_t            u8WriteTime[NUM_OF_RTC_REG] =  {0};

/* contains the number of seconds in different months */
static const uint32_t u32SecsInMonth[] = 
{
    (uint32_t)0,        /* value not used */
    (uint32_t)2678400,  /* number of seconds in January */
    (uint32_t)2419200,  /* number of seconds in February */
    (uint32_t)2678400,  /* number of seconds in March */
    (uint32_t)2592000,  /* number of seconds in April */
    (uint32_t)2678400,  /* number of seconds in May */
    (uint32_t)2592000,  /* number of seconds in June */
    (uint32_t)2678400,  /* number of seconds in July */
    (uint32_t)2678400,  /* number of seconds in August */
    (uint32_t)2592000,  /* number of seconds in September */
    (uint32_t)2678400,  /* number of seconds in October */
    (uint32_t)2592000,  /* number of seconds in November */
    (uint32_t)2678400   /* number of seconds in December */
};



/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief Convert RTC data to BCD values
 * 
 * @param psDate the time object
 */
static void vRTCDataToBCD(sTime_t const *psTime);


/**
 * @brief Convert RTC data to decimal values
 * 
 * @param psTime time object
 */
static void vRTCDataToDec(sTime_t *psTime);


/**
 * @brief Initiate an I2C transfert
 * 
 * @param u8Data    the array containing the data to handle
 * @param u8CmdAddr the address of the rtc memory to access
 * @param u8Direction direction of the transfer (can be READ or WRITE)
 */
static bool bI2CTransfertRTC(uint8_t* u8Data,
                             uint8_t  u8CmdAddr,
                             uint8_t  u8Direction);


/**
 * @brief Set the masks to the raw RTC datas
 */
static void vSetRTCDataMask(void);


/**
 * @brief Check if the specified year is a leap year
 * 
 * @param u16Year year
 * @return true is the year is a leap year, false otherwise
 */
static bool bIsLeapYear(uint32_t u16Year);


/**
 * @brief Convert a bcd value to a decimal value
 * 
 * @param u8BcdVal bcd value
 * 
 * @return the converted decimal value
 */
static inline uint8_t u8BcdToDec(uint8_t u8BcdVal)
{
    return ((u8BcdVal >> 4) * DEC_BASE)+
            (u8BcdVal & HALF_BYTE_MSK);
}


/**
 * @brief Convert a decimal value to a BCD value
 * 
 * @param u8DecVal the decimal value to convert
 * 
 * @return the converted BCD value
 */
static inline uint8_t u8DecToBcd(uint8_t u8DecVal)
{
    return ((u8DecVal % DEC_BASE) + (uint8_t)(((u8DecVal/
             DEC_BASE) % DEC_BASE) << HALF_BYTE));
}


/**
 * @brief Get the unix time and convert it to rtc datas
 * 
 * @param u32UnixTime [in] unix time
 */
static void vUnixToRTCData(uint32_t u32UnixTime);


/**
 * @brief convert rtc datas to unix time
 * 
 * @return the converted unix time
 */
static uint32_t u32RTCDataToUnix(void);



/************************************************************************************************************
 *                           PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

bool bM41t56Init(I2C_HandleTypeDef *psI2C)
{
    CHECK_PTR(psI2C);

    psLocalI2C = psI2C;

    /* Check if target device is ready for communication */
    if ((HAL_OK == HAL_I2C_IsDeviceReady(psI2C, M41T56_ADDR, TRIALS_3, TIMEOUT_10MS)))
    {
    	bM41t56Initialized = true;
    }

    return bM41t56Initialized;
}


bool bM41t56SetTime(uint32_t u32UnixTime)
{
    /* convert unix time to rtc datas */
    vUnixToRTCData(u32UnixTime);

    return bI2CTransfertRTC(&u8WriteTime[0], RTCSEC_ADDR, WRITE);
}


uint32_t u32M41t56GetTime(void)
{
	static bool bRet   = false;
    uint32_t u32RetVal = 0;

	if (bRet == false)
	{
        /* dummy read */
		 bI2CTransfertRTC(&u8ReadTime[0], RTCSEC_ADDR, READ);
		 bRet = true;
	}

    if (true == bI2CTransfertRTC(&u8ReadTime[0], RTCSEC_ADDR, READ))
    {
        /* Set the mask and convert the rtc datas to decimal values */
        vSetRTCDataMask();
        vRTCDataToDec(&localTime);

        u32RetVal = u32RTCDataToUnix();  /* return the compute unix time */
    }
    else
    {
        u32RetVal = 0;   /* error occured */
    }

    return u32RetVal;
}



/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static bool bI2CTransfertRTC(uint8_t* u8Data, uint8_t u8CmdAddr, uint8_t u8Direction)
{
    bool bRet = false;

	if (HAL_I2C_IsDeviceReady(psLocalI2C, M41T56_ADDR, TRIALS_3, TIMEOUT_100MS) == HAL_OK)
    {
        if (READ == u8Direction)
        {
            bRet = (bool)(HAL_OK == HAL_I2C_Mem_Read(psLocalI2C, M41T56_ADDR,
            		     (u16)u8CmdAddr, (u16)1, u8Data, LENGHT7, TIMEOUT_100MS));
        }
        else
        {
            bRet = (bool)(HAL_OK == HAL_I2C_Mem_Write_IT(psLocalI2C, M41T56_ADDR,
                         (u16)u8CmdAddr, (u16)1, u8Data, LENGHT7));
        } 
    }

    return bRet;
}


static bool bIsLeapYear(uint32_t u16Year)
{
    bool bRet = false;

    if (u16Year % 400 == 0)
        bRet = true;
    else if (u16Year % 100 == 0)
        bRet = false;
    else if (u16Year % 4 == 0)
        bRet = true;
    else
        bRet = false;

    return bRet;
}


static void vRTCDataToBCD(sTime_t const *psTime)
{
    /* Convert all the RTC related data to BCD values */
    u8WriteTime[DAY]   =  u8DecToBcd((u8)psTime->u32Day  );
    u8WriteTime[SECS]  =  u8DecToBcd((u8)psTime->u32Sec  );
    u8WriteTime[MINS]  =  u8DecToBcd((u8)psTime->u32Min  );
    u8WriteTime[HOUR]  =  u8DecToBcd((u8)psTime->u32Hour );
    u8WriteTime[MNTH]  =  u8DecToBcd((u8)psTime->u32Month);
    u8WriteTime[YEAR]  =  u8DecToBcd((u8)psTime->u32Year );
    u8WriteTime[W_DAY] =  u8DecToBcd((u8)psTime->u32WkDay) + 1;
}


static void vRTCDataToDec(sTime_t *psTime)
{
    /* Convert all the RTC related data to decimal values */
    psTime->u32Day     =  u8BcdToDec(u8ReadTime[DAY]  ); 
    psTime->u32Sec     =  u8BcdToDec(u8ReadTime[SECS] );
    psTime->u32Min     =  u8BcdToDec(u8ReadTime[MINS] );
    psTime->u32Hour    =  u8BcdToDec(u8ReadTime[HOUR] );
    psTime->u32Month   =  u8BcdToDec(u8ReadTime[MNTH] );
    psTime->u32Year    =  u8BcdToDec(u8ReadTime[YEAR] );
    psTime->u32WkDay   =  u8BcdToDec(u8ReadTime[W_DAY]);
    psTime->u32Year    =  psTime->u32Year + MIL_BASE;
}


static void vSetRTCDataMask (void)
{
    /* Set the mask for seconds, hour, month, day of the week */
    u8ReadTime[SECS]   &=  SEC_MASK;
    u8ReadTime[HOUR]   &=  HOUR_MASK;
    u8ReadTime[MNTH]   &=  MONTH_MASK;
    u8ReadTime[W_DAY]  &=  DAY_MASK;
    u8ReadTime[DAY]    &=  DATE_MASK;
}


static void vUnixToRTCData(uint32_t u32UnixTime)
{
    uint32_t u16NbrDays;

    /* Calculate seconds, minutes, hours, days, months, and years from Unix time */
    localTime.u32Min  = (u32UnixTime   / SEC_MIN_BASE);
    localTime.u32Hour = (localTime.u32Min  / SEC_MIN_BASE);
    localTime.u32Day  = (localTime.u32Hour / HOUR_BASE);
    localTime.u32Year =  UNIX_YEAR;

    /* compute the year */
    while (localTime.u32Day >= 365)
    {
        u16NbrDays = (bIsLeapYear(localTime.u32Year) ? 366 : 365);

        if (localTime.u32Day >= u16NbrDays)
        {
            localTime.u32Day -= u16NbrDays;
            localTime.u32Year++;
        }
    }

    /* check if the year is a leap year */
    uint8_t isLeap = bIsLeapYear(localTime.u32Year);

    uint16_t u16DaysInMonth[12] = {31, 28 + isLeap, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    localTime.u32Month = 0;

    /* calculate the day of the month and the month */
    while (localTime.u32Day >= u16DaysInMonth[localTime.u32Month])
    {
        localTime.u32Day -= u16DaysInMonth[localTime.u32Month];
        localTime.u32Month++;
    }

    /* store the computed datas */
    localTime.u32Sec   = u32UnixTime        % SEC_MIN_BASE;
    localTime.u32Min   = localTime.u32Min   % SEC_MIN_BASE;
    localTime.u32Hour  = localTime.u32Hour  % HOUR_BASE;
    localTime.u32Day   = localTime.u32Day   + 1;
    localTime.u32Month = localTime.u32Month + 1;
    localTime.u32Year  = localTime.u32Year  % 100;

    vRTCDataToBCD(&localTime);  /* convert the datas to bcd values */
}


static uint32_t u32RTCDataToUnix(void)
{
    uint32_t u32UnixTime = 0;

    /* Calculate years since 1970 */
    for (u32 i = UNIX_YEAR; i < localTime.u32Year; i++)
    {
        u32UnixTime += (bIsLeapYear(i) ? SEC_IN_LEAP_YEAR : SEC_IN_ORDI_YEAR);
    }

    /* Calculate seconds since January 1st */
    for (u32 i = 1; i < localTime.u32Month; i++)
    {
        u32UnixTime += u32SecsInMonth[i];
    }

    u32UnixTime += (localTime.u32Day - 1) * SEC_IN_ONE_DAY;
    u32UnixTime += localTime.u32Hour * SEC_IN_ONE_HOUR;
    u32UnixTime += localTime.u32Min  * SEC_MIN_BASE;
    u32UnixTime += localTime.u32Sec;

    return u32UnixTime;
}


/************************************************************************************************************
 *                           END OF MODULE
 ***********************************************************************************************************/
