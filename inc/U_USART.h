/*
 * U_USART.h
 *
 *  Created on: 2017年9月27日
 *      Author: Romeli
 */

#ifndef U_USART_H_
#define U_USART_H_

#include "cmsis_device.h"
#include "U_Steam.h"
//#include "U_Parse.h"

typedef enum {
	U_USARTMode_Normal, U_USARTMode_DMA
} U_USARTMode_Typedef;

typedef enum {
	U_USART485Status_Enable, U_USART485Status_Disable
} U_USART485Status_Typedef;

typedef enum {
	U_USART485Dir_Tx, U_USART485Dir_Rx
} U_USART485Dir_Typedef;

class U_USARTClass: public U_Steam {
public:
	U_USARTClass(uint16_t rxBufSize, uint16_t txBufSize);
	virtual ~U_USARTClass();

	void Init(uint32_t baud, uint16_t USART_Parity = USART_Parity_No,
			U_USART485Status_Typedef rs485Status = U_USART485Status_Disable,
			U_USARTMode_Typedef mode = U_USARTMode_DMA);

	Status_Typedef Write(uint8_t* data, uint16_t len);

	bool CheckFrame();

	void ReceiveEvent();

	Status_Typedef USARTIRQ();
	Status_Typedef DMATxIRQ();
protected:
	USART_TypeDef *_USARTx;
	DMA_TypeDef *_DMAx;
	DMA_Channel_TypeDef *_DMAy_Channelx_Rx, *_DMAy_Channelx_Tx;
	uint32_t _DMA_IT_TC_TX;

	virtual void GPIOInit(U_USART485Status_Typedef status);
	virtual void USARTInit(uint32_t baud, uint16_t USART_Parity);
	virtual void DMAInit();
	virtual void ITInit(U_USARTMode_Typedef mode);

	virtual void RS485DirCtl(U_USART485Dir_Typedef dir);
private:
	volatile bool _DMATxBusy;
	volatile bool _newFrame;
	DataStack_Typedef _DMARxBuf;
	DataStack_Typedef _TxBuf2;

	U_USARTMode_Typedef _mode;
	U_USART485Status_Typedef _rs485Status;

	Status_Typedef DMASend(DataStack_Typedef *stack, DataStack_Typedef *txBuf);
};

#endif /* U_USART_H_ */
