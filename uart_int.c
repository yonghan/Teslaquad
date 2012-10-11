/*
 * uart_int.c - Interrupt Driven USART with ring buffer
 *
 *  Created on: Oct 18, 2011
 *      Author: Yonghan Ching
 *
 *  Sep 21, 2012 ported to STM32F4 platform
 */

 #include "uart_int.h"

// USART Ring buffer
USART_RING_BUFFER_T rb;

// Current Tx Interrupt enable state
__IO FlagStatus TxIntStat;

void USART2_IRQHandler(void)
{
    // Check if there is any error present
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) |
       USART_GetFlagStatus(USART2, USART_FLAG_FE) |
       USART_GetFlagStatus(USART2, USART_FLAG_PE) |
       USART_GetFlagStatus(USART2, USART_FLAG_NE))
    {
        while(1);
    }

    // Check if this is a receive interrupt
    if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        USART_IntReceive(USART2);
    }

    // Check if this is a transmit interrupt
    if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
    {
        USART_IntTransmit(USART2);
    }
}

void USART_IntReceive(USART_TypeDef *USARTPort)
{
    uint8_t tmpc;
    uint16_t rLen;

    while(1)
    {
        // Call USART read function in USART driver
        rLen = USART_ReceiveData(USARTPort);
        // If data received
        if (rLen)
        {
            /* Check if buffer is more space
             * If no more space, remaining character will be trimmed out
             */
            if (!__BUF_IS_FULL(rb.rx_head,rb.rx_tail))
            {
                rb.rx[rb.rx_head] = tmpc;
                __BUF_INCR(rb.rx_head);
            }
        }
        // no more data
        else
        {
            break;
        }
    }
}

void USART_IntTransmit(USART_TypeDef *USARTPort)
{
    // Disable TXE interrupt
    USART_ITConfig(USARTPort, USART_IT_TXE, DISABLE);

	/* Wait for FIFO buffer empty, transfer data or break whenever ring buffers are empty */
	/* Wait until TXE is set */
    //while (USART_GetFlagStatus(USARTPort, USART_FLAG_TXE) != SET);

    while (!__BUF_IS_EMPTY(rb.tx_head,rb.tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
        if (USART_SendBlock(USARTPort, rb.tx[rb.tx_tail], 1))
        {
            /* Update transmit ring FIFO tail pointer */
            __BUF_INCR(rb.tx_tail);
        }
        else
        {
            break;
        }
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
    if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail))
    {
        USART_ITConfig(USARTPort, USART_IT_TXE, DISABLE);
        // Reset Tx Interrupt state
        TxIntStat = RESET;
    }
    else
    {
      	// Set Tx Interrupt state
        TxIntStat = SET;
    	USART_ITConfig(USARTPort, USART_IT_TXE, ENABLE);
    }
}

uint32_t USART_SendBlock(USART_TypeDef *USARTPort, uint8_t txbuf, uint8_t buflen)
{
    uint8_t bToSend, bSent, timeOut;

    bToSend = buflen;
    bSent = 0;
    while (bToSend)
    {
/*        if(!(USARTPort->SR & USART_FLAG_TXE))
        {
            break;
        }*/
        while(USART_GetFlagStatus(USARTPort, USART_FLAG_TXE))
        {
            USART_SendData(USARTPort, txbuf++);
            bToSend--;
            bSent++;
        }
    }

    return bSent;
}

uint32_t USARTSend(USART_TypeDef *USARTPort, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *)txbuf;
    uint32_t bytes = 0;

    /* Temporarily lock out USART transmit interrupts during this
    read so the USART transmit interrupt won't cause problems
    with the index values */
    USART_ITConfig(USARTPort, USART_IT_TXE, DISABLE);

    /* Loop until transmit run buffer is full or until n_bytes
    expires */
    while ((buflen > 0) && (!__BUF_IS_FULL(rb.tx_head, rb.tx_tail)))
    {
        /* Write data from buffer into ring buffer */
        rb.tx[rb.tx_head] = *data;
        data++;

        /* Increment head pointer */
        __BUF_INCR(rb.tx_head);

        /* Increment data count and decrement buffer size count */
        bytes++;
        buflen--;
    }

    /*
    * Check if current Tx interrupt enable is reset,
    * that means the Tx interrupt must be re-enabled
    * due to call USART_IntTransmit() function to trigger
    * this interrupt type
    */
    if (TxIntStat == RESET)
    {
        USART_IntTransmit(USARTPort);
    }
    /*
    * Otherwise, re-enables Tx Interrupt
    */
    else
    {
        USART_ITConfig(USARTPort, USART_IT_TXE, ENABLE);
    }

    return bytes;
}

uint32_t USARTSendString(USART_TypeDef *USARTPort, char *buff)
{
    uint32_t i = 0;

    // Get the exact length of the buffer
    while(buff[i] != '\0') { i++; }

    return USARTSend(USARTPort, (uint8_t *)buff, i);
}

uint32_t USART_DataAvailable(void)
{
    if(rb.rx_head < rb.rx_tail)
    {
        return USART_RING_BUFSIZE - rb.rx_tail + rb.rx_head;
    }
    else
    {
        return rb.rx_head - rb.rx_tail;
    }
}

char USART_ReadChar(USART_TypeDef *USARTPort)
{
	uint8_t buf;
	USARTReceive(USARTPort, &buf, 1);
	return (char)buf;
}

uint32_t USARTReceive(USART_TypeDef *USARTPort, uint8_t *rxbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;

    /* Temporarily lock out USART receive interrupts during this
       read so the USART receive interrupt won't cause problems
       with the index values */
    USART_ITConfig(USARTPort, USART_IT_RXNE, DISABLE);

    /* Loop until receive buffer ring is empty or
            until max_bytes expires */
    while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb.rx_head, rb.rx_tail))))
    {
        /* Read data from ring buffer into user buffer */
        *data = rb.rx[rb.rx_tail];
        data++;

        /* Update tail pointer */
        __BUF_INCR(rb.rx_tail);

        /* Increment data count and decrement buffer size count */
        bytes++;
        buflen--;
    }

    /* Re-enable USART interrupts */
    USART_ITConfig(USARTPort, USART_IT_RXNE, ENABLE);

    return bytes;
}

void USART_RingInit(USART_TypeDef *USARTPort, uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    if(USARTPort == USART2)
    {
        /* enable peripheral clock for USART2 */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        /* GPIOA clock enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        /* Configure USART Tx and Rx as alternate function push-pull */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Connect USART pins to AF7 */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    }
    else
    {
        // USART Ports not implemented except port 2
        while(1);
    }

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = wordlength;
    USART_InitStructure.USART_StopBits = stopbits;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // Initialize USARTx
    USART_Init(USARTPort, &USART_InitStructure);

    USART_ITConfig(USARTPort, USART_IT_TXE, ENABLE);
    USART_ITConfig(USARTPort, USART_IT_RXNE, ENABLE);

    USART_Cmd(USARTPort, ENABLE);

}