#ifndef _PARSER_H_
#define _PARSER_H_



/************************************************************************************************************
*	         INCLUDES
************************************************************************************************************/
#include "hw_desc_timer_stm32f1.h"
#include "hw_desc_uart_stm32f1.h"



/************************************************************************************************************
 *           PUBLIC FUNCTIONS DECLARATIONS
 ***********************************************************************************************************/

/**
 * @brief receive the message on the network
 */
void vRcvMsg(void);


/**
 * @brief This function is used to initialize the parser
 *
 * @param [in] psTimerObj : timer object
 * @param [in] sUARTObj_t : uart object
 */
void vParserInit(const sTIMObj_t * const psTimerObj, const sUARTObj_t* const  psUartObj);


#endif	/* _PARSER_H_ */
