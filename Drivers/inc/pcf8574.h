#ifndef _PCF8574_H_
#define _PCF8574_H_



/************************************************************************************************************
*	           INCLUDES
************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"



/************************************************************************************************************
*	           DEFINES
************************************************************************************************************/
#define PCF8574_CRTL_CODE                  (uint8_t)(0x04)
#define PCF8574_ADDR_OFFSET                (uint8_t)(0x03)
#define PCF8574_CS_ADDR0                   (uint8_t)(0)             
#define PCF8574_CS_ADDR1                   (uint8_t)(1)             
#define PCF8574_CS_ADDR2                   (uint8_t)(2)            
#define PCF8574_CS_ADDR3                   (uint8_t)(3)             
#define PCF8574_CS_ADDR4                   (uint8_t)(4)             
#define PCF8574_CS_ADDR5                   (uint8_t)(5)             
#define PCF8574_CS_ADDR6                   (uint8_t)(6)             
#define PCF8574_CS_ADDR7                   (uint8_t)(7)   
#define PCF8574_ADDRESS(CS_ADDR)           (uint8_t)((PCF8574_CRTL_CODE << PCF8574_ADDR_OFFSET)|(CS_ADDR)) 



/************************************************************************************************************
*	         ENUMERATIONS
************************************************************************************************************/

/**
* @enum select the address of the PCF8574 device
*/
typedef enum ePCF8574Address
{
   PCF8574_ADDR0 = PCF8574_ADDRESS(PCF8574_CS_ADDR0),
   PCF8574_ADDR1 = PCF8574_ADDRESS(PCF8574_CS_ADDR1),
   PCF8574_ADDR2 = PCF8574_ADDRESS(PCF8574_CS_ADDR2),
   PCF8574_ADDR3 = PCF8574_ADDRESS(PCF8574_CS_ADDR3),
   PCF8574_ADDR4 = PCF8574_ADDRESS(PCF8574_CS_ADDR4),
   PCF8574_ADDR5 = PCF8574_ADDRESS(PCF8574_CS_ADDR5),
   PCF8574_ADDR6 = PCF8574_ADDRESS(PCF8574_CS_ADDR6),
   PCF8574_ADDR7 = PCF8574_ADDRESS(PCF8574_CS_ADDR7),

   PCF8574_ADDR_MAX
} ePCF8574Address_t;



/************************************************************************************************************
 *           PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/** @brief This function initialize the pcf8574
  * @param [in] eSlaveAddress : address of port expander device
  * @param [in] psI2C         : pointer to I2C object
  * @return none
 **/
void vPcf8574Init(ePCF8574Address_t eSlaveAddress, I2C_HandleTypeDef *psI2C);


/** @brief This function write on a port of pcf8574
  * @param  [in]  u8PortValue  : value to write on the port
  * @return true if operation was done correctly, false otherwise
 **/
bool bPcf8574WritePort(uint8_t u8PortValue);


/** @brief This function read a port of pcf8574
  * @param  [in] pu8Buffer  : pointer to the buffer where port value would be stored
  * @return true if operation was done correctly, false otherwise
 **/
bool bPcf8574ReadPort(uint8_t *pu8Buffer);



#endif	/* _PCF8574_H_ */
