/************************************************************************************************************
* @file	hw_desc_adc_stm32f1.c
* @date	25.06.2023
*************************************************************************************************************
* @brief   This file contains the functions to handle the adc peripheral
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "hw_desc_adc_stm32f1.h"
#include "hw_desc_gpio_stm32f1.h"
#include "hw_desc_dma_stm32f1.h"
#include "stm32f100xe.h"



/************************************************************************************************************
*	                         DEFINES 
************************************************************************************************************/
#define ADC_CLOCK_DIV2         (0x3U << 14U)
#define CLEAR_REGISTER         (u32)0
#define ADC_WAIT_CYCLES        (u8)40
#define SAMPLE_TIME_13_CYCLES  (u32)0x12
#define SEQUENCE_CONV          ((13U << 0U) | (12U << 5U) | (11U << 10U))
#define ADC_PERIOD             (u32)20
#define CALL_INDEFINITELY      (u8)0
#define ADC_NUM_OF_CHANNELS    (u8)3
#define NB_OF_CONVERSIONS      (u8)10
#define MEAN_SAMPLE            (u8)10
#define CHANNEL_1              (u8)0
#define CHANNEL_2              (u8)1
#define CHANNEL_3              (u8)2

typedef struct sAdcSample_tTag
{
	uint16_t xk[MEAN_SAMPLE];
	uint16_t xsum;
	uint16_t xmean;
}sAdcSample_t;

/************************************************************************************************************
*	                         PRIVATE VARIABLES
************************************************************************************************************/

static sGPIObj_t          sGPIObj  = {0};
static sDmaParams_t       sDmaCtrl = {0};
static volatile uint16_t  u16ADCTempRes[ADC_NUM_OF_CHANNELS];
static volatile uint16_t  xk[MEAN_SAMPLE ] = {0};
static volatile uint16_t  yk[MEAN_SAMPLE ] = {0};
static volatile uint16_t  zk[MEAN_SAMPLE ] = {0};
static volatile uint16_t  xksum = 0;
static volatile uint16_t  yksum = 0;
static volatile uint16_t  zksum = 0;
static volatile uint16_t  xmean = 0;
static volatile uint16_t  ymean = 0;
static volatile uint16_t  zmean = 0;

/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/   

/**
 * @brief start a single conversion
 */
static inline void vStartSingleConv(void)
{
    ADC1->SR   = CLEAR_REGISTER;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}


/**
 * @brief start the calibration of the adc
 */
static void vStartCalibration(void);


/**
 * @brief configure the gpio pins for the selected adc channels
 */
static void vConfigGPIOPins(void);


/**
 * @brief configure the dma for the adc
 */
static void vDmaConfig(void);


/**
 * @brief Initialize the adc registers
 */
static void vInitADCHw(void);


/**
 * @brief get the adc conversion result
 *
 * @param pu16ConvResult buffer to store the result
 */
static void vGetConvResult(uint16_t* pu16ConvResult);


/**
 * @brief handle the adc conversion results
 */
void vADCConvHandler(void);



/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static void vInitADCHw(void)
{
    /* enable the adc clock and set the clock prescaler to 2 (12 Mhz) */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CFGR    &= ~ADC_CLOCK_DIV2;

    /* clear the adc registers */
    ADC1->SR     = CLEAR_REGISTER;
    ADC1->CR1    = CLEAR_REGISTER;
    ADC1->CR2    = CLEAR_REGISTER;

    ADC1->CR1   |= ADC_CR1_SCAN;  /* enable the scan mode */
    ADC1->CR2   |= ADC_CR2_DMA;   /* enable dma request */

    /* set the sample time to 13.5 cycles */
    ADC1->SMPR1 |= SAMPLE_TIME_13_CYCLES;
    ADC1->SQR1  |= ADC_SQR1_L_1;  /* set the number of conversions to 3 */
    ADC1->SQR3  |= SEQUENCE_CONV; /* set the conversion sequence */

    /* enable and set the trigger source (software trigger) */
    ADC1->CR2   |= (ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL);

    /* Initialize the dma and gpio pins */
    vDmaConfig();
    vConfigGPIOPins();

    /* start the calibration of the adc */
    vStartCalibration();
}


static void vStartCalibration(void)
{
	/* enable the adc */
    ADC1->CR2 |= ADC_CR2_ADON;

    /* wait for the ADC to stabilize */
    uint8_t u8Calib = ADC_WAIT_CYCLES;
    while (u8Calib--);

    /* start the calibration and wait for it to complete */
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    /* start a dummy conversion */
    vStartSingleConv();
}


static void vConfigGPIOPins(void)
{
    vGPIOInit(&sGPIObj, GPIO_PORTC);
    sGPIObj.pfvCfgInput(GPIO_PORTC, GPIO_PIN1, INPUT_ANALOG);
    sGPIObj.pfvCfgInput(GPIO_PORTC, GPIO_PIN2, INPUT_ANALOG);
    sGPIObj.pfvCfgInput(GPIO_PORTC, GPIO_PIN3, INPUT_ANALOG);
}


static void vDmaConfig(void)
{
    /* Select the dma channel for the adc */
    sDmaCtrl.eDmaChanSel   = DMA_CHANNEL_1;

    /* Set the dma transfer word size to 16 bits */
    sDmaCtrl.eWordSize     = DMA_WORD_16;

    /* Configure the dma type to reception */
    sDmaCtrl.eTransferType = DMA_RECEIVE;

    /* Set adc data register as source address for the transfer */
    sDmaCtrl.u32DmaSrcReg  = (u32)&ADC1->DR;

    /* Set the dma reception size */
    sDmaCtrl.u16RcvDmaSize = (u16)ADC_NUM_OF_CHANNELS;

    /* Set the buffer address for reception */
    sDmaCtrl.u32RcvAddress = (u32)&u16ADCTempRes[0];

    /* Initialize the dma */
    vDmaInit(&sDmaCtrl);
}


static void vGetConvResult(uint16_t* pu16ConvResult)
{
    pu16ConvResult[CHANNEL_1] =  xmean;
    pu16ConvResult[CHANNEL_2] =  ymean;
    pu16ConvResult[CHANNEL_3] =  zmean;
}


void vADCConvHandler(void)
{
	/* trigger adc conversion */
	vStartSingleConv();

    for(int i = 0; i < (MEAN_SAMPLE - 1); i++)
    {
    	xk[i] = xk[i+1];
    	yk[i] = yk[i+1];
    	zk[i] = zk[i+1];
    }

    xk[MEAN_SAMPLE - 1] = u16ADCTempRes[0];
    yk[MEAN_SAMPLE - 1] = u16ADCTempRes[1];
    zk[MEAN_SAMPLE - 1] = u16ADCTempRes[2];

    xksum = 0;
    yksum = 0;
    zksum = 0;

    for(int i = 0; i < MEAN_SAMPLE; i++)
    {
       xksum = xksum + xk[i];
       yksum = yksum + yk[i];
       zksum = zksum + zk[i];
    }

    xmean = (uint16_t)(xksum / MEAN_SAMPLE);
    ymean = (uint16_t)(yksum / MEAN_SAMPLE);
    zmean = (uint16_t)(zksum / MEAN_SAMPLE);
}


/************************************************************************************************************
 *                           PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vADCInit(sADCObj_t *psADCObj, const sTIMObj_t *psTimerObj)
{
	/* check if the adc and timer objects was created */
    CHECK_PTR(psADCObj)
    CHECK_PTR(psTimerObj)

    vInitADCHw(); /* Initialize the adc hardware */

    /* assigning addresses of the callback functions */
    psADCObj->pfvStartADCConv  = vStartSingleConv;
    psADCObj->pfvGetConvResult = vGetConvResult;

    /* start ADC conversion every 20 milliseconds */
    psTimerObj->pfbRegisterCbk(vADCConvHandler, ADC_PERIOD, CALL_INDEFINITELY);
}


/************************************************************************************************************
 *                           END OF MODULE
 ***********************************************************************************************************/
