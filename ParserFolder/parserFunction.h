#ifndef PARSER_FUNCTION_H
#define PARSER_FUNCTION_H



/************************************************************************************************************
*	         INCLUDES
************************************************************************************************************/
#include "stdint.h"
#include "stm32f1_types.h"



/************************************************************************************************************
 *           PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/
/**
 * @brief This function is used to initialize GPIO pin for relays and varistors on the UPP
 */
void vUPPGPIOInit(void);


/**
 * @brief This function is used to turn relays on or off
 */
void vFunctionRelaisID1(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to get ADC values
 */
void vFunctionReadADCID4(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to reset the UPP
 */
void vFunctionUPPID25(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to switch remote relays via I2C
 */
void vFunctionSWRemoteRelaisID36(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to read the output values of remote relays
 */
void vFunctionReadAnalogInputsID37(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to test I2C
 */
void vFunctionI2CTestID39(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to set the Unix time
 */
void vFunctionSetTimeID48(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);


/**
 * @brief This function is used to get the Unix time
 */
void vFunctionGetTimeID49(const uint8_t *pu8RxBuffer, uint8_t *pu8TxBuffer);




#endif
