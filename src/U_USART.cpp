/*
 * U_USART.cpp
 *
 *  Created on: 2017年9月27日
 *      Author: Romeli
 */

#include "U_USART.h"

#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */
#define CR1_UE_Reset              ((uint16_t)0xDFFF)  /*!< USART Disable Mask */

U_USARTClass::U_USARTClass(uint16_t rxBufSize, uint16_t txBufSize) :
		U_Steam(rxBufSize, txBufSize) {
	//默认设置
	_USARTx = USART3;
	_DMAx = DMA1;
	_DMAy_Channelx_Rx = DMA1_Channel3;
	_DMAy_Channelx_Tx = DMA1_Channel2;

	if (_DMAy_Channelx_Tx == DMA1_Channel1) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC1;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel2) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC2;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel3) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC3;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel4) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC4;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel5) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC5;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel6) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC6;
	} else if (_DMAy_Channelx_Tx == DMA1_Channel7) {
		_DMA_IT_TC_TX = (uint32_t) DMA1_IT_TC7;
	} else if (_DMAy_Channelx_Tx == DMA2_Channel1) {
		_DMA_IT_TC_TX = (uint32_t) DMA2_IT_TC1;
	} else if (_DMAy_Channelx_Tx == DMA2_Channel2) {
		_DMA_IT_TC_TX = (uint32_t) DMA2_IT_TC2;
	} else if (_DMAy_Channelx_Tx == DMA2_Channel3) {
		_DMA_IT_TC_TX = (uint32_t) DMA2_IT_TC3;
	} else if (_DMAy_Channelx_Tx == DMA2_Channel4) {
		_DMA_IT_TC_TX = (uint32_t) DMA2_IT_TC4;
	} else if (_DMAy_Channelx_Tx == DMA2_Channel5) {
		_DMA_IT_TC_TX = (uint32_t) DMA2_IT_TC5;
	}

	_newFrame = false;

	_DMARxBuf.data = 0;
	_DMARxBuf.tail = 0;
	_DMARxBuf.busy = false;

	_TxBuf2.data = 0;
	_TxBuf2.tail = 0;
	_TxBuf2.busy = false;

	_mode = U_USARTMode_DMA;
	_rs485Status = U_USART485Status_Disable;
}

U_USARTClass::~U_USARTClass() {
}

void U_USARTClass::Init(uint32_t baud, uint16_t USART_Parity,
		U_USART485Status_Typedef rs485Status, U_USARTMode_Typedef mode) {
	_mode = mode;
	_rs485Status = rs485Status;
	//GPIO初始化
	GPIOInit(_rs485Status);
	//USART外设初始化
	USARTInit(baud, USART_Parity);
	if (mode == U_USARTMode_DMA) {
		//DMA初始化
		DMAInit();
	}
	//中断初始化
	ITInit(_mode);
	//使能USART
	_USARTx->CR1 |= CR1_UE_Set;
}

Status_Typedef U_USARTClass::Write(uint8_t* data, uint16_t len) {
	DataStack_Typedef statck;
	statck.data = data;
	statck.tail = len;
	if (_mode == U_USARTMode_DMA) {
		while (statck.tail != 0) {
			if ((_DMAy_Channelx_Tx->CMAR != (uint32_t) _TxBuf.data)
					&& (_TxBuf.size - _TxBuf.tail != 0)) {
				//若缓冲区1空闲，并且有空闲空间
				DMASend(&statck, &_TxBuf);
			} else if ((_DMAy_Channelx_Tx->CMAR != (uint32_t) _TxBuf2.data)
					&& (_TxBuf2.size - _TxBuf2.tail != 0)) {
				//若缓冲区2空闲，并且有空闲空间
				DMASend(&statck, &_TxBuf2);
			} else {
				//发送繁忙，两个缓冲区均在使用或已满
				//FIXME@romeli 需要添加超时返回代码
			}
		}
	} else {
		//非DMA模式
		while (len--) {
			_USARTx->DR = (*data++ & (uint16_t) 0x01FF);
			while (!(_USARTx->SR & USART_FLAG_TXE))
				;
		}
	}
	return Status_Ok;
}

bool U_USARTClass::CheckFrame() {
	if (_newFrame) {
		//读取到帧接收标志后将其置位
		_newFrame = false;
		return true;
	} else {
		return false;
	}
}

__attribute__((weak)) void U_USARTClass::ReceiveEvent() {

}

Status_Typedef U_USARTClass::USARTIRQ() {
	//读取串口标志寄存器
	uint16_t staus = _USARTx->SR;
	if ((staus & USART_FLAG_IDLE) != 0) {
		//帧接收标志被触发
		_newFrame = true;
		if (_mode) {
			//关闭DMA接收
			_DMAy_Channelx_Rx->CCR &= (uint16_t) (~DMA_CCR1_EN);

			uint16_t len = _RxBuf.size - _DMAy_Channelx_Rx->CNDTR;
			//清除DMA标志
//			_DMAx->IFCR = DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_TE3
//					| DMA1_FLAG_HT3;
			//复位DMA接收区大小
			_DMAy_Channelx_Rx->CNDTR = _RxBuf.size;
			//循环搬运数据
			for (uint16_t i = 0; i < len; ++i) {
				_RxBuf.data[_RxBuf.tail] = _DMARxBuf.data[i];
				_RxBuf.tail = (_RxBuf.tail + 1) % _RxBuf.size;
			}
			//开启DMA接收
			_DMAy_Channelx_Rx->CCR |= DMA_CCR1_EN;
		}
		USART_ReceiveData(_USARTx);
		//串口帧接收事件
		ReceiveEvent();
	}
#ifndef USE_DMA
	//串口字节接收中断置位
	if ((_USARTx->SR & USART_FLAG_RXNE) != 0) {
		//搬运数据到缓冲区
		_RxBuf.data[_RxBuf.tail] = _USARTx->DR;
		_RxBuf.tail = (_RxBuf.tail + 1) % _RxBuf.size;
	}
#endif
	//串口帧错误中断
	if ((_USARTx->SR & USART_FLAG_ORE) != 0)
		USART_ReceiveData(USART3);
	return Status_Ok;
}

Status_Typedef U_USARTClass::DMATxIRQ() {
	//暂时关闭DMA接收
	_DMAy_Channelx_Tx->CCR &= (uint16_t) (~DMA_CCR1_EN);

	_DMAx->IFCR = _DMA_IT_TC_TX;

	//判断当前使用的缓冲通道
	if (_DMAy_Channelx_Tx->CMAR == (uint32_t) _TxBuf.data) {
		//缓冲区1发送完成，置位指针
		_TxBuf.tail = 0;
		//判断缓冲区2是否有数据，并且忙标志未置位（防止填充到一半发送出去）
		if (_TxBuf2.tail != 0 && _TxBuf2.busy == false) {
			//当前使用缓冲区切换为缓冲区2，并加载DMA发送
			_DMAy_Channelx_Tx->CMAR = (uint32_t) _TxBuf2.data;
			_DMAy_Channelx_Tx->CNDTR = _TxBuf2.tail;

			_DMAy_Channelx_Tx->CCR |= DMA_CCR1_EN;
			return Status_Ok;
		} else {
			_DMAy_Channelx_Tx->CMAR = 0;
			//无数据需要发送，清除发送队列忙标志
			_DMATxBusy = false;
		}
	} else if (_DMAy_Channelx_Tx->CMAR == (uint32_t) _TxBuf2.data) {
		//缓冲区2发送完成，置位指针
		_TxBuf2.tail = 0;
		//判断缓冲区1是否有数据，并且忙标志未置位（防止填充到一半发送出去）
		if (_TxBuf.tail != 0 && _TxBuf.busy == false) {
			//当前使用缓冲区切换为缓冲区1，并加载DMA发送
			_DMAy_Channelx_Tx->CMAR = (uint32_t) _TxBuf.data;
			_DMAy_Channelx_Tx->CNDTR = _TxBuf.tail;

			_DMAy_Channelx_Tx->CCR |= DMA_CCR1_EN;
			return Status_Ok;
		} else {
			_DMAy_Channelx_Tx->CMAR = 0;
			//无数据需要发送，清除发送队列忙标志
			_DMATxBusy = false;
		}
	} else {
		//缓冲区号错误?不应发生
		return Status_Error;
	}

	if (_rs485Status == U_USART485Status_Enable) {
		while (!(_USARTx->SR & USART_FLAG_TC))
			;
		RS485DirCtl(U_USART485Dir_Rx);
	}
	return Status_Ok;
}

void U_USARTClass::GPIOInit(U_USART485Status_Typedef status) {
	GPIO_InitTypeDef GPIO_InitStructure;
	//开启GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	//设置PC10复用输出模式（TX）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//设置PC11上拉输入模式（RX）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (status == U_USART485Status_Enable) {
		//设置PC12流控制引脚
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		RS485DirCtl(U_USART485Dir_Rx);
	}
}

void U_USARTClass::USARTInit(uint32_t baud, uint16_t USART_Parity) {
	USART_InitTypeDef USART_InitStructure;
	//开启USART3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//配置USART3 全双工 停止位1 无校验
	USART_DeInit(_USARTx);
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	if (USART_Parity == USART_Parity_No) {
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	} else {
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	}

	USART_Init(_USARTx, &USART_InitStructure);
}

void U_USARTClass::DMAInit() {
	DMA_InitTypeDef DMA_InitStructure;

	_DMARxBuf.size = _RxBuf.size;
	if (_DMARxBuf.data != 0) {
		delete _DMARxBuf.data;
	}
	_DMARxBuf.data = new uint8_t[_DMARxBuf.size];

	_TxBuf2.size = _TxBuf.size;
	if (_TxBuf2.data != 0) {
		delete _TxBuf2.data;
	}
	_TxBuf2.data = new uint8_t[_TxBuf2.size];

	//开启DMA时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(_DMAy_Channelx_Tx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&_USARTx->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;				//临时设置，无效
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 10;				//临时设置，无效
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(_DMAy_Channelx_Tx, &DMA_InitStructure);
	DMA_ITConfig(_DMAy_Channelx_Tx, DMA_IT_TC, ENABLE);
	//发送DMA不开启
//	DMA_Cmd(DMA1_Channel4, ENABLE);

	DMA_DeInit(_DMAy_Channelx_Rx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&_USARTx->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) _DMARxBuf.data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = _DMARxBuf.size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(_DMAy_Channelx_Rx, &DMA_InitStructure);
	DMA_Cmd(_DMAy_Channelx_Rx, ENABLE);
}

void U_USARTClass::ITInit(U_USARTMode_Typedef mode) {
	NVIC_InitTypeDef NVIC_InitStructure;

	if (mode == U_USARTMode_DMA) {
		//配置DMA中断
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
		NVIC_Init(&NVIC_InitStructure);

		//串口发送接收的DMA功能
		USART_DMACmd(_USARTx, USART_DMAReq_Tx, ENABLE);
		USART_DMACmd(_USARTx, USART_DMAReq_Rx, ENABLE);
	} else {
		//开启串口的字节接收中断
		USART_ITConfig(_USARTx, USART_IT_RXNE, ENABLE);
	}

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;

	NVIC_Init(&NVIC_InitStructure);

	//开启串口的帧接收中断
	USART_ITConfig(_USARTx, USART_IT_IDLE, ENABLE);
}

void U_USARTClass::RS485DirCtl(U_USART485Dir_Typedef dir) {
	if (dir == U_USART485Dir_Rx) {
		GPIOC->BRR = GPIO_Pin_12;
	} else {
		GPIOC->BSRR = GPIO_Pin_12;
	}
}

Status_Typedef U_USARTClass::DMASend(DataStack_Typedef * stack,
		DataStack_Typedef * txBuf) {
	uint16_t avaSize, end;
	if (stack->tail != 0) {
		//置位忙标志，防止计算中DMA自动加载发送缓冲
		txBuf->busy = true;
		//计算缓冲区空闲空间大小
		avaSize = txBuf->size - txBuf->tail;
		//根据空闲空间大小计算搬移结束位置
		if (stack->tail > avaSize) {
			end = stack->tail - avaSize;
		} else {
			end = 0;
		}

		//填充数据，并且偏移指针
		for (; stack->tail > end; --stack->tail) {
			txBuf->data[txBuf->tail++] = *stack->data++;
		}
		if (!_DMATxBusy) {
			//DMA发送空闲，发送新的缓冲
			_DMATxBusy = true;

			if (_rs485Status == U_USART485Status_Enable) {
				RS485DirCtl(U_USART485Dir_Tx);
			}

			//设置DMA地址
			_DMAy_Channelx_Tx->CMAR = (uint32_t) txBuf->data;
			_DMAy_Channelx_Tx->CNDTR = txBuf->tail;

			//使能DMA开始发送
			_DMAy_Channelx_Tx->CCR |= DMA_CCR1_EN;
		}
		//解除忙标志
		txBuf->busy = false;
	}
	return Status_Ok;
}

