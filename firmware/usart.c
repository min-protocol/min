/*
 * usart.c
 *
 * Created: 28/04/2014 16:13:57
 *  Author: ken_000
 */ 

#include "main.h"

/* Define FIFOs for receive and transmit */
#define TX_FIFO_MAXSIZE                 (70U)           /* Must be >= 33 bytes */
#define RX_FIFO_MAXSIZE                 (40U)

struct fifo tx_fifo;
struct fifo rx_fifo;

uint8_t tx_buf[TX_FIFO_MAXSIZE];
uint8_t rx_buf[RX_FIFO_MAXSIZE];


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

/* UART0; check hardware for specific ports required */
#define UART                                ((volatile uint8_t *)(0x00c0U))

/* Main setup of USART
 *
 * Hardwired to asynchronous and not using clock doubler so will do 16 samples on receive (better for noise tolerance).
 * The baud parameter will be written straight into the baud rate register.
 */
void init_uart(uint16_t baud, uint8_t stop_bits, uint8_t data_bits, uint8_t parity)
{
	uint8_t ucsrnc_tmp;
    
    /* Initialise FIFOs for UART */
    fifo_init(&tx_fifo, tx_buf, TX_FIFO_MAXSIZE);
    fifo_init(&rx_fifo, rx_buf, RX_FIFO_MAXSIZE);
    
	/* Clear down UMSELn1 and UMSELn0 to set USART into UART mode.
	 * Set UMPn1 and UPMn0 according to parity
	 */
	ucsrnc_tmp = 0;
	if(parity == UART_PARITY_EVEN) {
		ucsrnc_tmp |= (1U << UPMn1);
	}
	else if(parity == UART_PARITY_ODD) {
		ucsrnc_tmp |= (1U << UPMn1) | (1U << UPMn0);
	}
	else {
		/* Parity = none, clear down UPMn bits */
	}
	/* Set USBSn according to stop bits */
	if(stop_bits == 2U) {
		ucsrnc_tmp |= (1U << USBSn);
	}
	/* UCSZn according to data bits */ 
	if(data_bits == 6U || data_bits == 8U) {
		ucsrnc_tmp |= (1U << UCSZn0);
	}
	if(data_bits == 7U || data_bits == 8U) {
		ucsrnc_tmp |= (1U << UCSZn1);
	}

	UCSRnC(UART) = ucsrnc_tmp;

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
void uart_isend(void)
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
void uart_ireceive(void)
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

uint8_t uart_receive_space(void)
{
	uint8_t tmp;
	uint8_t ret;
	
	LOCK_INTERRUPTS(tmp);
	ret = rx_fifo.size - rx_fifo.used;
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

ISR(USART0_RX_vect)
{
	uart_ireceive();
}

ISR(USART0_UDRE_vect)
{
	uart_isend();
}
