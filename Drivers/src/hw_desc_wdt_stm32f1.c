/************************************************************************************************************
* @file	hw_desc_wdt_stm32f1.c
* @date	26.07.2023
*************************************************************************************************************
* @brief This file contains some functions to handle independent watchdog timer
*
************************************************************************************************************/


#include "hw_desc_wdt_stm32f1.h"
#include "stm32f1_hw_def.h"
#include "hw_desc_timer_stm32f1.h"
#include "stm32f100xe.h"


/********************************************************************************************************************
*													                                                                *
*	                  P R I V A T E S     D E F I N I T I O N    O F    C O N S T A N T S                           *
*   													                                                            *
********************************************************************************************************************/


#define WDT_PR_BIT                             (0)
#define WDT_PRESCALER_DIV4                     (uint32_t)(0 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV8                     (uint32_t)(1 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV16                    (uint32_t)(2 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV32                    (uint32_t)(3 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV64                    (uint32_t)(4 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV128                   (uint32_t)(5 << WDT_PR_BIT)
#define WDT_PRESCALER_DIV256                   (uint32_t)(6 << WDT_PR_BIT)
#define WDT_PRESCALER                          WDT_PRESCALER_DIV16
#define ENABLE_WDT_RELOAD                      (uint32_t)(0xAAAAUL)
#define ENABLE_WDT_ACCESS                      (uint32_t)(0x5555UL)
#define START_WDT                              (uint32_t)(0xCCCCUL)
#define LSI_CLK                                (uint32_t)(40000) /* clock in Khz */
#define LSI_TIME                               (85U)  /* time in us */
#define DEFAULT_PRESCALER                      (4UL << WDT_PRESCALER)
#define WDT_TIME_OUT                           ((DEFAULT_PRESCALER*1000*6)/LSI_CLK) + (LSI_TIME/1000 + 1 )
#define WDT_SR_RVU_BIT                         (1)
#define WDT_SR_PVU_BIT                         (0)
#define CHECK_STATUS_FLAG                      ((1 << WDT_SR_RVU_BIT) | (1 << WDT_SR_PVU_BIT))
#define WDT_PERIOD_MIN                         (1)
#define WDT_PERIOD_MAX                         (1638)
#define CHECK_WDT_PERIOD(PARAM)                if (((PARAM) < WDT_PERIOD_MIN) || ((PARAM) > WDT_PERIOD_MAX)) { while(1);}
#define WDT_PERIOD(PARAM)                      (((PARAM)*LSI_CLK)/(1000*DEFAULT_PRESCALER))
#define IWDG_REG                               ((IWDG_HWREG*)IWDG_ADDRESS)


/********************************************************************************************************************
*													                                                                *
*	                          E X T E R N  V A R I A B L E                                                          *
*   													                                                            *
********************************************************************************************************************/
extern sTIMObj_t sTimerObj;


/********************************************************************************************************************
*													                                                                *
*	                          P R I V A T E  V A R I A B L E                                                        *
*   													                                                            *
********************************************************************************************************************/

static bool bWDTInitialized = false;



/********************************************************************************************************************
*													                                                                *
*	                          P R I V A T E  F U N C T I O N  D E C L A R A T I O N                                 *
*   													                                                            *
********************************************************************************************************************/
/**
 * @brief   this function initialize watchdog
 * @param   u16WdtPeriod: this is a period of watchdod.
 * @note    period min (ms) : 1,  period max (ms) : 1638 (ms) 
 * @return  Nothing
 */
static bool bInitWdt(uint16_t u16WdtPeriod);


/**
 * @brief   this function is used to refresh watchdog
 * @return  Nothing
 */
static void vResetWdt(void);
/********************************************************************************************************************
*													                                                                *
*	                          P R I V A T E  F U N C T I O N  D E F I N I T I O N                                   *
*   													                                                            *
********************************************************************************************************************/


static bool bInitWdt(uint16_t u16WdtPeriod)
{
    CHECK_WDT_PERIOD(u16WdtPeriod)

	/* Start watchdog */
	IWDG_REG->KR = START_WDT;

    /* Enable access to the watchdog prescaler register */
    IWDG_REG->KR = ENABLE_WDT_ACCESS;

    /* Set watchdog prescaler */
    IWDG_REG->PR = WDT_PRESCALER;

    /* set watchdog period */
    IWDG_REG->RLR = WDT_PERIOD(u16WdtPeriod);

    /* fetch elapse time */
    uint32_t u32ElapseTime = sTimerObj.pfu32GetTickMs();

    /* wait until all flags are clear */
    while ((((IWDG_REG->SR) & CHECK_STATUS_FLAG)) && (sTimerObj.pfbHasElapsed(u32ElapseTime, WDT_TIME_OUT) == false));

    /* check flags */
    if (((IWDG_REG->SR) & CHECK_STATUS_FLAG) == 0)
    {
        /* reload watchdog counter */
        IWDG_REG->KR = ENABLE_WDT_RELOAD;

        bWDTInitialized = true;
    }

    return bWDTInitialized;
}



static void vResetWdt(void)
{
    if (bWDTInitialized)
    {
        /* reload watchdog counter */
        IWDG_REG->KR = ENABLE_WDT_RELOAD;
    }
}


/********************************************************************************************************************
*													                                                                *
*	                          P U B L I C S  F U N C T I O N  D E F I N I T I O N                                   *
*   													                                                            *
********************************************************************************************************************/


void vWdtInitInst(sWDG_t* psWdtCfg)
{
    CHECK_PTR(psWdtCfg)

    /* initialization of watchdog */
    bInitWdt(psWdtCfg->u16WdtPeriod);

    /* set watchdog refresh function */
    psWdtCfg->pfvResetWdt  = &vResetWdt;
}

