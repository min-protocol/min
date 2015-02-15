/*
 * usart.h
 *
 * Created: 24/04/2014 12:38:50
 *  Author: ken_000
 */ 


#ifndef USART_H_
#define USART_H_

#include "fifo.h"

/* USART driver for ATmega640.
 *
 * The driver is interrupt driven. The architecture is a reader-writer FIFO (see fifo.h) as follows:
 *
 * Sending:
 * - A FIFO for sending
 * - "Character sent" interrupt reads a character from the FIFO (if there is one) and puts it in the UART data buffer
 * - A background task writes to the FIFO; if it was empty it triggers the "sent" interrupt (or writes directly to the UART)
 *
 * Receiving:
 * - A FIFO for receiving
 * - A "character received" interrupt reads the UART buffer and writes to the FIFO (writes to full are ignored and character discarded)
 * - A background task polls the FIFO to see if there are enough characters to process
 */ 

#define UART_PARITY_NONE			(0)
#define UART_PARITY_ODD				(1U)
#define UART_PARITY_EVEN			(2U)

#define UART_BAUD_9600				((F_OSC_HZ / (16UL * 9600UL)) - 1UL)
#define UART_BAUD_115200 			((F_OSC_HZ / (16UL * 115200UL)) - 1UL)		/* Quite high error on this baud rate */
#define UART_BAUD_125000 			((F_OSC_HZ / (16UL * 125000UL)) - 1UL)
#define UART_BAUD_57600				((F_OSC_HZ / (16UL * 57600UL)) - 1UL)
#define UART_BAUD_38400				((F_OSC_HZ / (16UL * 38400UL)) - 1UL)
	
/* Main UART setup function
 *
 * baud			Baud rate, defined by macros above
 * stop_bits	1 or 2
 * data_bits	5, 6, 7, 8
 * parity		None, odd or even, defined by macros above
 *
 * The UART is set up for interrupt handling on transmit and receive.
 */
void init_uart(uint16_t baud, uint8_t stop_bits, uint8_t data_bits, uint8_t parity);

/* Main functions, normally to be called by interrupt handlers, for receiving and transmitting data on a UART */
void uart_isend(void);				/* Called by USART Data Register Empty interrupt */
void uart_ireceive(void);			/* Called by USART RX Complete interrupt */

/* Functions for background task to use */
uint8_t uart_receive(uint8_t *dest, uint8_t n);
uint8_t uart_send(uint8_t *src, uint8_t n);
uint8_t uart_send_space(void);
uint8_t uart_receive_space(void);
uint8_t uart_receive_ready(void);

#define SEND_UART(src, n)								uart_send((src), (n))
#define RECEIVE_UART(dest, n)							uart_receive((dest), (n))
#define UART_SEND_SPACE()								uart_send_space()
#define UART_RECEIVE_READY()							uart_receive_ready()
#define UART_RECEIVE_SPACE()							uart_receive_space()
#endif /* USART_H_ */
