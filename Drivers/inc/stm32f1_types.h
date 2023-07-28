/** 
 @file 	  stm32f1_comT00.h
 @date    2023.06.11
 @details data types used in the whole project 
 
 #   DATE       |  Version  | Name   | Added or changed |
 --------------------------------------------------------
 # 2023.06.11   |  1.0      | AH     |   first version  |
 
*/


#ifndef _STM32F1_COMT00_H_      /* Prevent recursive inclusions */
#define _STM32F1_COMT00_H_


#include <stdint.h>
#include <stdbool.h>

/*------------ Define constants ------------------------------ */
#define FALSE                   false
#define TRUE                    true
#define NULL_PTR                ((void*)0U)
#define _IO                     volatile
#define CHECK_PTR(x)            if (x == NULL_PTR) { while(1); }
#define CHECK_RANGE(x,y)        if (x >= y )      { while(1); }
#define CHECK_OVERRUN(x,y,z)    if (x == y) x = z;
#define START_NBR               (uint8_t)'0'
#define END_NBR                 (uint8_t)'9'
#define SEMI                    (uint8_t)';'
#define CMD_END                 (uint8_t)'\r'
#define SEC                     *1000ul
#define MSEC                    *1ul 
#define CHECK_CHAR(x)           (((x >= START_NBR) && (x <= END_NBR)) || (x == SEMI)) ? TRUE : FALSE

/*------------ provided types----------------------------------*/
typedef uint8_t      u8; 
typedef int8_t       s8; 
typedef uint16_t     u16; 
typedef int16_t      s16; 
typedef uint32_t     u32; 
typedef int32_t      s32; 
typedef uint64_t     u64; 
typedef int64_t      s64;


typedef union sLongType_tTag
{
	uint64_t u64LongWord;
	uint32_t u32LongWord[2];
	uint16_t u16Word[4];
	uint8_t  u8Byte[8];	
} sLongType_t;



#endif
