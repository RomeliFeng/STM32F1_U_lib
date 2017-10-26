/*
 * U_Steam.cpp
 *
 *  Created on: 2017年9月29日
 *      Author: Romeli
 */

#include "U_Steam.h"

uint16_t U_Steam::Available() {
	return _RxBuf.front <= _RxBuf.tail ?
			_RxBuf.tail - _RxBuf.front :
			_RxBuf.size - _RxBuf.front + _RxBuf.tail;
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
Status_Typedef U_Steam::Write(uint8_t* data, uint16_t len) {
	return Status_Ok;
}
#pragma GCC diagnostic pop

inline Status_Typedef U_Steam::Write(uint8_t data) {
	return Write(&data, 1);
}

Status_Typedef U_Steam::Print(void* str) {
	uint16_t len = getLen((uint8_t *) str);
	return Write((uint8_t *) str, len);
}

Status_Typedef U_Steam::Print(int32_t num, uint8_t base) {
	uint8_t str[16];
	uint16_t len = byNumber(&num, base, str);
	return Write(str, len);
}

Status_Typedef U_Steam::Print(double flo, uint8_t ndigit) {
	uint8_t str[16];
	uint16_t len = byFloat(&flo, ndigit, str);
	return Write(str, len);
}

Status_Typedef U_Steam::Read(uint8_t* data, uint16_t len) {
	//循环读取
	for (uint8_t i = 0; i < len; ++i) {
		Read(data + i);
	}
	return Status_Ok;
}

Status_Typedef U_Steam::Read(uint8_t* data) {
	//读取一个数
	*data = _RxBuf.data[_RxBuf.front];
	return SpInc(&_RxBuf);
}

Status_Typedef U_Steam::Peek(uint8_t* data) {
	//偷看一个数
	*data = _RxBuf.data[_RxBuf.front];
	return Status_Ok;
}

Status_Typedef U_Steam::PeekNextDigital(uint8_t *data, uint8_t ignore,
		bool detectDecimal) {
	//偷看一个数
	Peek(data);
	//当读到的字符为 '-','0'-'9','.'（detectDecimal为true）时返回
	if ((*data == '-') || ((*data >= '0') && (*data <= '9'))
			|| ((*data == '.') && detectDecimal) || (*data == ignore)) {
	} else {
		return Status_Error;
	}
	return Status_Ok;
}

Status_Typedef U_Steam::NextInt(void *num, uint8_t ignore) {
	bool firstChar = true;
	bool isNeg = false;
	uint8_t c = 0;
	uint8_t sp = _RxBuf.front;
	int32_t n = 0;

	while (Available() > 0) {
		if (PeekNextDigital(&c, ignore, false) == Status_Ok) {
			//如果读到数字
			if (c == '-') {
				if (firstChar) {
					//检测到一个'-'
					isNeg = true;
					SpInc(&_RxBuf);
					continue;
				} else {
					//'-'不是第一个数
					break;
				}
			} else if ((c == ignore) && (ignore != 0)) {
				SpInc(&_RxBuf);
				continue;
			}
			n = n * 10 + c - '0';
			firstChar = false;
			SpInc(&_RxBuf);
		} else {
			break;
		}
	}
	if ((sp != _RxBuf.front) && (c != '-') && (c != ignore)) {
		//有读取到数
		if (isNeg) {
			n = -n;
		}
		*(int32_t *) num = n;
		return Status_Ok;
	} else {
		//没有读取到数
		*(int32_t *) num = 0;
		return Status_Error;
	}
}

Status_Typedef U_Steam::NextFloat(void* flo, uint8_t ignore) {
	double f = 0;
	double frac = 1.0;
	bool isNeg = false;
	bool isFra = false;
	bool firstChar = true;
	uint8_t sp = _RxBuf.front;
	uint8_t c = 0;

	while (Available() > 0) {
		if (PeekNextDigital(&c, ignore, true) == Status_Ok) {
			if (c == '-') {
				if (firstChar) {
					//检测到一个'-'
					isNeg = true;
					SpInc(&_RxBuf);
					continue;
				} else {
					//'-'不是第一个数
					break;
				}
			} else if ((c == ignore) && (ignore != 0)) {
				SpInc(&_RxBuf);
				continue;
			} else if (c == '.') {
				if (isFra) { //不应出现两个'-'
					break;
				} else {
					if (!firstChar) {
						SpInc(&_RxBuf);
						isFra = true;
						continue;
					} else {
						//第一个字符为点的时候
						break;
					}
				}
			}
			if (isFra) {
				frac *= 0.1;
			}
			f = f * 10 + c - '0';
			SpInc(&_RxBuf);
			firstChar = false;
		} else {
			break;
		}
	}

	if ((sp != _RxBuf.front) && (c != '-') && (c != ignore)) {
		//有读取到数
		f = isNeg ? -f : f;
		f = isFra ? f * frac : f;
		*(double *) flo = f;
		return Status_Ok;
	} else {
		//没有读取到数
		*(double *) flo = 0;
		return Status_Error;
	}
}

inline bool U_Steam::IsEmpty(DataStack_Typedef* stack) {
	//判断缓冲区是否为空
	return stack->front == stack->tail;
}

Status_Typedef U_Steam::SpInc(DataStack_Typedef *stack) {
	if (IsEmpty(stack)) {
		//缓冲区为空
		return Status_Error;
	} else {
		//缓冲区指针+1
		stack->front = (stack->front + 1) % stack->size;
		return Status_Ok;
	}
}

Status_Typedef U_Steam::SpDec(DataStack_Typedef* stack) {
	stack->front = stack->front == 0 ? stack->size : stack->front - 1;
	return Status_Ok;
}

void U_Steam::Clear() {
	_RxBuf.front = _RxBuf.tail;
}

uint16_t U_Steam::getLen(uint8_t* str) {
	uint16_t len = 0;
	for (len = 0; *(str + len) != '\0'; ++len)
		;
	return len;
}
