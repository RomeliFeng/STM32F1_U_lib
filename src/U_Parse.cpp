/*
 * U_Parse.cpp
 *
 *  Created on: 2016��10��6��
 *      Author: Romeli
 */

#include "U_Parse.h"

uint16_t U_Parse::byNumber(void *num, uint8_t base, uint8_t* str) {
	//使用空指针传入整型数
	int32_t n = *(uint32_t *) num;
	//小于零取反加负号
	if (n < 0) {
		n = -n;
		*str++ = '-';
	}

	//按照进制取得长度
	uint8_t len = getLen((uint32_t) n, base);
	str += len;
	*str = '\0';
	do {
		int8_t c = n % base;
		n /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);
	return len;
}

uint8_t U_Parse::byFloat(void *flo, uint8_t ndigit, uint8_t* str) {
	double f = *(double *) flo;
	uint8_t len = 0, len2 = 0;
	uint32_t int_part = 0;
	double rem_part = 0;

	uint8_t dot = '.';

	uint8_t tmp[20];
	//小于零取反加负号
	if (f < 0.0) {
		str[len] = '-';
		f = -f;
		++len; //字符串长度+1 ‘-’号长度
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	float rounding = 0.5;
	for (uint8_t i = 0; i < ndigit; ++i)
		rounding /= 10.0;

	f += rounding;

	// Extract the integer part of the number and print it
	int_part = (uint32_t) f;
	rem_part = (f - (double) int_part);

	len2 = byNumber(&int_part, 10, tmp); //转换整数部分
	len = strcat(str, len, tmp, len2); //拼接整数部分到字符串
	if (ndigit > 0) { //如果有小数部分
		len = strcat(str, len, &dot, 1);
		while (ndigit--) {
			rem_part *= 10;
			int_part = (int16_t) rem_part;	//每次转换一位小数
			rem_part -= int_part;
			str[len++] = int_part + 0x30;
		}
	}
	return len;
}

uint8_t U_Parse::getLen(uint32_t num, uint8_t base) {
	uint8_t i;
	if (num == 0) {
		return 1;
	}
	for (i = 0; num != 0; ++i) {
		num /= base;
	}
	return i;
}

double U_Parse::pow10(uint8_t power) {
	if (power == 0)
		return 1.0;
	else
		return 10.0 * pow10(--power);
}

uint8_t U_Parse::strcat(uint8_t* str_to, uint8_t str_to_len, uint8_t* str_from,
		uint8_t str_from_len) {
	uint8_t i;
	for (i = 0; i < str_from_len; ++i) { //搬移数据
		str_to[str_to_len + i] = str_from[i];
	}
	str_to[str_to_len + i] = '\0'; //在字符串末尾填'\0'
	return str_to_len + str_from_len; //返回字符串长度
}
