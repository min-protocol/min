/*
 * Serial drivers for ATmega640 family, with software FIFOs for
 * receive and transmit.
 * 
 *  Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */ 

#include "serial.h"

/* Sending process:
 * - A FIFO for sending
 * - "Character sent" interrupt reads a character from the FIFO (if there
 *   is one) and puts it in the UART data buffer
 * - Application layer writes to the FIFO; if it was empty it triggers the
 *   "sent" interrupt (or writes directly to the UART)
 *
 * Receiving process:
 * - A FIFO for receiving
 * - A "character received" interrupt reads the UART buffer and writes to
 *   the FIFO (writes to full are ignored and character discarded)
 * - Application layer polls the FIFO to see if there are enough characters
 *   to process
 */ 

struct fifo {
	uint8_t *buf;		/* Space allocated to FIFO */
	uint8_t size;		/* Size of FIFO */
	uint8_t head;		/* Indexes first free byte (unless full) */
	uint8_t tail;		/* Indexes last filled byte (unless empty) */
	uint8_t used;		/* Between 0..size */
	uint8_t max_used;	/* Maximum space used in the FIFO */
};

#define FIFO_EMPTY(f)	((f)->used == 0)
#define FIFO_FULL(f)	((f)->used == (f)->size)
#define FIFO_USED(f)	((f)->used)

static void fifo_init(struct fifo *f, uint8_t buf[], uint8_t size)
{
	f->used = 0;
	f->head = f->tail = 0;
	f->buf = buf;
	f->size = size;	
	f->max_used = 0;
}

static void fifo_write(struct fifo *f, uint8_t b)
{
	if(!FIFO_FULL(f)) {						/* Only proceed if there's space */
		f->used++;							/* Keep track of the number of used bytes in the FIFO */
		f->buf[f->head++] = b;				/* Add to where head indexes */
		if(f->head == f->size) {			/* If head goes off the end of the FIFO buffer then wrap it */
			f->head = 0;
		}	
	}
	if(f->used > f->max_used) {				/* For diagnostics keep a high-water mark of used space */
		f->max_used = f->used;
	}
}

static uint8_t fifo_read(struct fifo *f)
{
	uint8_t ret = 0;
	
	if(!FIFO_EMPTY(f)) {					/* Only proceed if there's something there */
		f->used--;							/* Keep track of the used space */
		ret = f->buf[f->tail++];
		if(f->tail == f->size) {			/* Wrap tail around if it goes off the end of the buffer */
			f->tail = 0;
		}
	}
	
	return ret;
}

static struct fifo tx_fifo;
static struct fifo rx_fifo;

static uint8_t tx_buf[TX_FIFO_MAXSIZE];
static uint8_t rx_buf[RX_FIFO_MAXSIZE];

#define LOCK_INTERRUPTS(i)		    		{(i) = SREG; cli();}			/* NB: SIDE-EFFECT MACRO! */
#define UNLOCK_INTERRUPTS(i)				{SREG = i;}

/* Offsets from USARTn base for each register */
#define UCSRnA(u)							(*((u)+0U))
#define UCSRnB(u)							(*((u)+1U))
#define UCSRnC(u)							(*((u)+2U))
#define UBBRnL(u)							(*((u)+4U))
#define UBBRnH(u)							(*((u)+5U))
#define UDRn(u)								(*((u)+6U))

/* Bit numbers for registers */
/* UCSRnA */
#define RXCn								(7)
#define TXCn								(6)
#define UDREn								(5)
#define FEn									(4)
#define DORn								(3)
#define UPEn								(2)
#define U2Xn								(1)
#define MPCM3n								(0)

/* UCSRnB */
#define RXCIEn								(7)
#define TXCIEn								(6)
#define UDRIEn								(5)
#define RXENn								(4)
#define TXENn								(3)
#define UCSZn2								(2)
#define RXB8n								(1)
#define TXB8n								(0)

/* UCSRnC */
#define UMSELn1								(7)
#define UMSELn0								(6)
#define UPMn1								(5)
#define UPMn0								(4)
#define	USBSn								(3)
#define UCSZn1								(2)
#define UCSZn0								(1)
#define UCPOLn								(0)

/* Main setup of ATmega640 USART
 *
 * Hardwired to asynchronous and not using clock doubler so will do 16 samples on receive (better for noise tolerance).
 * The baud parameter will be written straight into the baud rate register.
 */
void init_uart(uint16_t baud)
{
    /* Initialize FIFOs for UART */
    fifo_init(&tx_fifo, tx_buf, TX_FIFO_MAXSIZE);
    fifo_init(&rx_fifo, rx_buf, RX_FIFO_MAXSIZE);
    
	/* Clear down UMSELn1 and UMSELn0 to set USART into UART mode.
	 * Set UMPn1 and UPMn0 according to parity
	 *
	 * Parity = none, clear down UPMn bits
	 * Set USBSn to 1 stop bits
	 * UCSZn according to 8 data bits 
	 */ 
	UCSRnC(UART) = (1U << UCSZn0) | (1U << UCSZn1);

	/* Enable interrupt for device ready to receive.
	 * Also /disable/ UDRIEn interrupt for transmit buffer empty (will be enabled on next write to transmit FIFO).
	 * Also clear down RXB8n, TXB8n and UCSZn2 to disable 9-bit data frames.
	 */
	UCSRnB(UART) = (1U << RXCIEn) | (1U << RXENn) | (1U << TXENn);

	/* Make sure USART is in asynchronous normal mode (U2Xn = 0) and not in multi-processor mode (MPCMn = 0).
	 * Clear down TXCn to make sure no garbage hanging over from another session.
	 * Clear down the error flags FEn, DORn and UPEn for future USART compatibility.
	 */
	UCSRnA(UART) = 0;
	
	/* Baud rate register must be written high byte first */
	UBBRnH(UART) = (baud & 0xff00) >> 8;	/* Write top byte of baud; compilers normally smart enough to spot this is just a byte operation */
	UBBRnL(UART) = baud & 0x00ff;
}

/* Handle interrupt generated when the transmit buffer is free to write to.
 */
static void uart_isend(void)
{
	uint8_t used = tx_fifo.used;
	/* Called when the transmit buffer can be filled */
	if(used > 0) {
		uint8_t byte;
		
		byte = fifo_read(&tx_fifo);
		UDRn(UART) = byte;	/* This write also clears down the interrupt */
		/* TODO: potentially could re-enable interrupts at this point because function is reentrant - but would be to be sure next interrupt couldn't
		 * be raised before rest of this ISR had executed
		 */
		if(used == 1U) {
			/* If it was just one byte in the FIFO then it will now be empty; stop interrupts until FIFO becomes not empty */
			
		}
	}
	if(used <= 1U) {
		/* There was either nothing to send (spurious interrupt) or else there is nothing to send now so must disable the UDREn interrupt
		 * to stop interrupts - must be re-enabled when FIFO becomes not empty
		 */
		UCSRnB(UART) &= ~(1U << UDRIEn);
	}
}

/* Handle receive interrupt.
 *
 * The CPU must handle the received bytes as fast as they come in (on average): there is no flow control.
 */
static void uart_ireceive(void)
{	
	uint8_t byte;
	/* TODO handle the error flags (DORn should be logged - it indicates the interrupt handling isn't fast enough; parity error frames should be discarded) */
	
	/* This read also clears down the interrupt */
	byte = UDRn(UART);
	/* TODO: potentially could re-enable interrupts at this point because function is reentrant - but would need to be sure next interrupt couldn't
	 * be raised before rest of this ISR had executed
	 */
	/* Write byte to FIFO; if there is no room in the FIFO the byte is discarded */
	fifo_write(&rx_fifo, byte);
}

/* Send n bytes to the given USART from the given source. Returns the number of bytes actually buffered. */
uint8_t uart_send(uint8_t *src, uint8_t n)
{
	uint8_t written = 0;
	uint8_t tmp;

	/* Interrupts are locked out only through the body of the loop so that transmission can
	 * start as soon as there is data to send, which happens concurrently with filling the
	 * FIFO
	 */
	while(n--) {
		LOCK_INTERRUPTS(tmp);
		if(FIFO_FULL(&tx_fifo)) {
			n = 0;	/* No space left, terminate the sending early */
		}
		else {
			fifo_write(&tx_fifo, *(src++));
			/* The FIFO is not empty so enable 'transmit buffer ready' interrupt
			 * (may already be enabled but safe to re-enable it anyway).
			 */ 
			UCSRnB(UART) |= (1U << UDRIEn);
			written++;
		}
		UNLOCK_INTERRUPTS(tmp);
	}
	
	return written;
}

/* Read up to n bytes from the given USART into the given destination. Returns the number of bytes actually read. */
uint8_t uart_receive(uint8_t *dest, uint8_t n)
{
	uint8_t tmp;
	uint8_t read = 0;
	
	while(n--) {
		LOCK_INTERRUPTS(tmp);
		if(FIFO_EMPTY(&rx_fifo)) {
			n = 0;	/* Nothing left, terminate early */
		}
		else {
			*(dest++) = fifo_read(&rx_fifo);
			read++;	
		}
		UNLOCK_INTERRUPTS(tmp);
	}
	
	return read;
}

uint8_t uart_send_space(void)
{
	uint8_t tmp;
	uint8_t ret;
	
	LOCK_INTERRUPTS(tmp);
	ret = tx_fifo.size - tx_fifo.used;
	UNLOCK_INTERRUPTS(tmp);
	
	return ret;
}

uint8_t uart_receive_ready(void)
{
	uint8_t tmp;
	uint8_t ret;
	
	LOCK_INTERRUPTS(tmp);
	ret = rx_fifo.used;
	UNLOCK_INTERRUPTS(tmp);
	
	return ret;	
}

ISR(UART_RX_VECT)
{
	uart_ireceive();
}

ISR(UART_TX_VECT)
{
	uart_isend();
}
