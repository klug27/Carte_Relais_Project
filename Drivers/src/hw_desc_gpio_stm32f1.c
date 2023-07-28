/************************************************************************************************************
* @file	hw_desc_gpio_stm32f1.h
* @date	22.06.2023
*************************************************************************************************************
* @brief This file contains the functions need to handle the GPIO peripheral
*
************************************************************************************************************/


/************************************************************************************************************
*	                          INCLUDES 
************************************************************************************************************/
#include "hw_desc_gpio_stm32f1.h"
#include "com_stm32f1.h"



/************************************************************************************************************
*	                          DEFINES 
************************************************************************************************************/
#define NUM_OF_GPIO             (u8)5

#define CONF_SEL(x)             (x / 8)
#define GPIO_PIN_CONF(x)        (((x % 8) * 4) + 2)
#define OUTPUT_SPEED_10MHZ(x)   (0x1U << (x % 8) * 4)

/* Reset the configuration register for the selected pin */
#define GPIO_RESET_CONFIG(x)    ((0xF << ((x % 8) * 4))| \
                                 (0xF << ((x % 8) * 4)))


/************************************************************************************************************
*	                          PRIVATE VARIABLES
************************************************************************************************************/
static GPIO_HWREG* psLocalGPIO[NUM_OF_GPIO] = {0};

/* array of GPIO ports addresses */
static GPIO_HWREG* psGPIOAddr[] =
{
   (GPIO_HWREG*)GPIOA_ADDRESS,
   (GPIO_HWREG*)GPIOB_ADDRESS,
   (GPIO_HWREG*)GPIOC_ADDRESS,
   (GPIO_HWREG*)GPIOD_ADDRESS,
   (GPIO_HWREG*)GPIOE_ADDRESS
};


/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/   

/**
 * @brief Configure the specified pin as input pin
 * 
 * @param [in] eGPIOPort gpio port
 * @param [in] eGPIOPin  pin number
 * @param [in] eInputCfg input mode (analog, floating, pull-up or pull-down)
 */
static void vCfgInput(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eInputCfg_t eInputCfg);

/**
 * @brief Configure the specified pin as output pin
 *
 * @param [in] eGPIOPin  pin number
 * @param [in] eOuputCfg output mode (push-pull or open-drain)
 */
static void vCfgOutput(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eOuputCfg_t eOuputCfg);

/**
 * @brief Configure a specified pin as alternate function pin
 * 
 * @param [in] eGPIOPort gpio port
 * @param [in] eGPIOPin pin number
 * @param [in] eAltCfg  alternate mode (push-pull, open-drain)
 */
static void vCfgAlternate(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eAltCfg_t eAltCfg);

/**
 * @brief Set the specified pin to a logic high
 *
 * @param [in] eGPIOPin pin number
 */
static inline void vSetPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin);

/**
 * @brief Reset the specified pin to a logic low
 *
 * @param [in] eGPIOPin pin number
 */
static inline void vResetPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin);

/**
 * @brief Read the state of the specified pin
 * 
 * @param [in] eGPIOPort gpio port
 * @param [in] eGPIOPin pin number
 */
static bool bReadPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin);



/************************************************************************************************************
 *                           PRIVATE FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

static void vCfgInput(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eInputCfg_t eInputCfg)
{
   /* Reset the configuration for the selected pin */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] &= ~GPIO_RESET_CONFIG(eGPIOPin);

   if (eInputCfg == INPUT_PULL_DOWN)
   {
      /* Set the pin input mode */
      psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= ((eInputCfg - (u32)1) << GPIO_PIN_CONF(eGPIOPin));

      /* Enable the weak pull-down resistor */
      psLocalGPIO[eGPIOPort]->BRR = ((u32)0x01 << eGPIOPin);
   }
   else if (eInputCfg == INPUT_PULL_UP)
   {
      /* Set the pin input mode */
      psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= (eInputCfg << GPIO_PIN_CONF(eGPIOPin));

      /* Enable the weak pull-up resistor */
      psLocalGPIO[eGPIOPort]->BSRR = ((u32)0x01 << eGPIOPin);
   }
   else
   {
      /* Set the pin input mode */
      psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= (eInputCfg << GPIO_PIN_CONF(eGPIOPin));
   }
}


static void vCfgOutput(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eOuputCfg_t eOuputCfg)
{
   /* Reset the configuration for the selected pin */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] &= ~GPIO_RESET_CONFIG(eGPIOPin);

   /* Set the output pin speed to 10 Mhz max */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= OUTPUT_SPEED_10MHZ(eGPIOPin);

   /* Set the pin output mode */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= ((eOuputCfg - (u32)1) << GPIO_PIN_CONF(eGPIOPin));
}


static void vCfgAlternate(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin, eAltCfg_t eAltCfg)
{
   /* Reset the configuration for the selected pin */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] &= ~GPIO_RESET_CONFIG(eGPIOPin);

   /* Set the output pin speed to 10 Mhz max */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= OUTPUT_SPEED_10MHZ(eGPIOPin);

   /* Set the pin output mode */
   psLocalGPIO[eGPIOPort]->CR[CONF_SEL(eGPIOPin)] |= (eAltCfg << GPIO_PIN_CONF(eGPIOPin));
}


static void vSetPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin)
{
   /* Set the specified pin to a logic high */
   psLocalGPIO[eGPIOPort]->BSRR = ((u32)0x01 << eGPIOPin);
}


static void vResetPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin)
{
   /* Reset the specified pin to a logic low */
   psLocalGPIO[eGPIOPort]->BRR = ((u32)0x01 << eGPIOPin);
}


static bool bReadPin(eGPIOPort_t eGPIOPort, eGPIOPin_t eGPIOPin)
{
   /* Return the state of the specified pin */
   return (bool)(psLocalGPIO[eGPIOPort]->IDR & (u16)(0x01U << eGPIOPin));
}



/************************************************************************************************************
 *                           PUBLIC FUNCTIONS DEFINITIONS
 ***********************************************************************************************************/

void vGPIOInit(sGPIObj_t* psGPIObj, eGPIOPort_t eGPIOPort)
{
   CHECK_PTR(psGPIObj)

   /* Select the corresponding GPIO controller */
   psLocalGPIO[eGPIOPort] = psGPIOAddr[eGPIOPort];

   if ((RCCTRL->APB2ENR & ((u32)0x1 << (eGPIOPort + (u32)2))) == 0)
   {
      /* Activate the clock for the selected port */
      RCCTRL->APB2ENR |= ((u32)0x1 << (eGPIOPort + (u32)2));
   }

   /* assigning addresses of the callback functions */
   psGPIObj->pfvCfgInput     = vCfgInput    ;
   psGPIObj->pfvCfgOutput    = vCfgOutput   ;
   psGPIObj->pfvCfgAlternate = vCfgAlternate;
   psGPIObj->pfbReadPin      = bReadPin     ;
   psGPIObj->pfvSetPin       = vSetPin      ;
   psGPIObj->pfvResetPin     = vResetPin    ;
}


/************************************************************************************************************
 *                           END OF MODULE
 ***********************************************************************************************************/
