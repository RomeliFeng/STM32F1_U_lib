/*
 * Parse.h
 *
 *  Created on: 2016��10��6��
 *      Author: Romeli
 */

#ifndef U_PARSE_H_
#define U_PARSE_H_

#include "cmsis_device.h"

class U_Parse {
public:
	static uint16_t byNumber(void *num, uint8_t base, uint8_t* str);

	static uint8_t byFloat(void *flo, uint8_t ndigit, uint8_t* str);

protected:
	static uint8_t getLen(uint32_t num, uint8_t base);
	static double pow10(uint8_t power);
	static uint8_t strcat(uint8_t* str_to, uint8_t str_to_len,
			uint8_t* str_from, uint8_t str_from_len);
};

#endif /* U_PARSE_H_ */
