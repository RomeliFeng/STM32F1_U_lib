/*
 * U_Steam.h
 *
 *  Created on: 2017年9月29日
 *      Author: Romeli
 */

#ifndef U_STEAM_H_
#define U_STEAM_H_

#include "cmsis_device.h"
#include "U_Parse.h"
#include "Typedef.h"

typedef struct _DataStack_Typedef {
	uint8_t* data;
	volatile uint16_t front;
	volatile uint16_t tail;
	uint16_t size;
	volatile bool busy;
} DataStack_Typedef;

class U_Steam: public U_Parse {
public:
	U_Steam(uint16_t rxBufSize, uint16_t txBufSize) {
		_RxBuf.size = rxBufSize;
		_RxBuf.data = new uint8_t[_RxBuf.size];
		_RxBuf.front = 0;
		_RxBuf.tail = 0;
		_RxBuf.busy = false;

		_TxBuf.size = txBufSize;
		_TxBuf.data = new uint8_t[_TxBuf.size];
		_TxBuf.front = 0;
		_TxBuf.tail = 0;
		_TxBuf.busy = false;
	}

	virtual ~U_Steam() {
		delete _RxBuf.data;
		delete _TxBuf.data;
	}

	virtual Status_Typedef Write(uint8_t *data, uint16_t len);
	virtual Status_Typedef Write(uint8_t data);

	Status_Typedef Print(void *str);
	Status_Typedef Print(int32_t num, uint8_t base = 10);
	inline Status_Typedef Print(int16_t num, uint8_t base) {
		return Print((int32_t) num, base);
	}
	inline Status_Typedef Print(int8_t num, uint8_t base) {
		return Print((int32_t) num, base);
	}
	inline Status_Typedef Print(uint32_t num, uint8_t base) {
		return Print((int32_t) num, base);
	}
	inline Status_Typedef Print(uint16_t num, uint8_t base) {
		return Print((int32_t) num, base);
	}
	inline Status_Typedef Print(uint8_t num, uint8_t base) {
		return Print((int32_t) num, base);
	}
	Status_Typedef Print(double flo, uint8_t ndigit = 2);
	inline Status_Typedef Print(float flo, uint8_t ndigit = 2) {
		return Print((double) flo, ndigit);
	}

	virtual Status_Typedef Read(uint8_t *data, uint16_t len);
	virtual Status_Typedef Read(uint8_t *data);

	Status_Typedef Peek(uint8_t *data);
	Status_Typedef PeekNextDigital(uint8_t *data, uint8_t ignore,
			bool detectDecimal = false);

	virtual Status_Typedef NextInt(void *num, uint8_t ignore = 0);
	virtual Status_Typedef NextFloat(void *flo, uint8_t ignore = 0);

	virtual uint16_t Available();
	virtual bool IsEmpty(DataStack_Typedef *stack);
	void Clear();
protected:
	DataStack_Typedef _RxBuf, _TxBuf;
	Status_Typedef SpInc(DataStack_Typedef *stack);
	Status_Typedef SpDec(DataStack_Typedef *stack);
private:
	uint16_t getLen(uint8_t *str);
};

#endif /* U_STEAM_H_ */
