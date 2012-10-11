/*
 * USART_int.h
 *
 *  Created on: Oct 18, 2011
 *      Author: Yonghan Ching
 */

#ifndef USART_INT_H_
#define USART_INT_H_

#include "stm32f4xx_conf.h"

/* buffer size definition */
#define USART_RING_BUFSIZE 256

/* Buf mask */
#define __BUF_MASK (USART_RING_BUFSIZE-1)
/* Check buf is full or not */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))
/* Check buf will be full in next receiving or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))
/* Check buf is empty */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))
/* Reset buf */
#define __BUF_RESET(bufidx)	(bufidx=0)
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)

/** @brief USART Ring buffer structure */
typedef struct
{
    __IO uint32_t tx_head;                /*!< USART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< USART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< USART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< USART Rx ring buffer tail index */
    __IO uint8_t  tx[USART_RING_BUFSIZE];  /*!< USART Tx data ring buffer */
    __IO uint8_t  rx[USART_RING_BUFSIZE];  /*!< USART Rx data ring buffer */
} USART_RING_BUFFER_T;

void USART1_IRQHandler(void);
void USART_IntTransmit(USART_TypeDef *USARTPort);
void USART_IntReceive(USART_TypeDef *USARTPort);
void USART_IntErr(uint8_t bLSErrType);
void USART_RingInit(USART_TypeDef *USARTPort, uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbit);
uint32_t USART_SendBlock(USART_TypeDef *USARTPort, uint8_t txbuf, uint8_t buflen);
uint32_t USARTReceive(USART_TypeDef *USARTPort, uint8_t *rxbuf, uint8_t buflen);
uint32_t USARTSend(USART_TypeDef *USARTPort, uint8_t *txbuf, uint8_t buflen);
uint32_t USARTSendString(USART_TypeDef *USARTPort, char *buff);
uint32_t USART_DataAvailable(void);
char USART_ReadChar(USART_TypeDef *USARTPort);
#endif /* USART_INT_H_ */
