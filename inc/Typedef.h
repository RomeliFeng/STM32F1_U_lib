/*
 * Typedef.h
 *
 *  Created on: 2017��1��8��
 *      Author: Romeli
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

#include "cmsis_device.h"

typedef struct _Bit_Typedef {
	uint8_t bit0 :1;
	uint8_t bit1 :1;
	uint8_t bit2 :1;
	uint8_t bit3 :1;
	uint8_t bit4 :1;
	uint8_t bit5 :1;
	uint8_t bit6 :1;
	uint8_t bit7 :1;
} Bit_Typedef;

typedef union _BytetoBit_Typedef {
	uint8_t byte;
	Bit_Typedef bit;
} BytetoBit_Typedef;

typedef union _WordtoByte_Typedef {
	uint8_t byte[2];
	uint16_t word;
} WordtoByte_Typedef;

typedef union _WordtoByteSigned_Typedef {
	uint8_t byte[2];
	int16_t word;
} WordtoByteSigned_Typedef;

typedef union _TwoWordtoByte_Typedef {
	uint8_t byte[4];
	uint32_t twoword;
} TwoWordtoByte_Typedef;

typedef union _TwoWordtoByteSigned_Typedef {
	uint8_t byte[4];
	int32_t twoword;
} TwoWordtoByteSigned_Typedef;

typedef union _DoubletoByte_Typedef {
	double d;
	uint8_t byte[8];
} DoubletoByte_Typedef;

typedef enum _Status_Typedef {
	Status_Ok, Status_Error, Status_TimeOut
} Status_Typedef;

#endif /* TYPEDEF_H_ */
