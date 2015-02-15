/*
 * min.c
 *
 *  Created on: May 3, 2014
 *      Author: ken
 */


#include "imin.h"

uint8_t min_lowest_tx_space;		/* Lowest space (in bytes) available in transmit buffer */
uint8_t min_tx_frame_no_space;		/* Number of times frame transmit discarded due to no space in transmit buffer */
uint8_t min_tx_enabled;				/* Transmission of MIN frames is disabled or enabled */

static uint16_t tx_sum1;
static uint16_t tx_sum2;

static void fletcher16_tx_init(void)
{
	tx_sum1 = 0x00ffu;
	tx_sum2 = 0x00ffu;
}

static void fletcher16_tx_step(uint8_t byte)
{
	tx_sum2 += tx_sum1 += byte;
}

static uint16_t fletcher16_tx_finalize(void)
{
	tx_sum1 = (tx_sum1 & 0x00ffu) + (tx_sum1 >> 8);
	tx_sum2 = (tx_sum2 & 0x00ffu) + (tx_sum2 >> 8);
	return tx_sum2 << 8 | tx_sum1;
}

static uint8_t tx_header_byte_countdown;

/* Queue a byte for transmission but also insert a stuff byte of 0x55 where two 0xaa bytes
 * occur in sequence
 */
static void stuffed_tx_byte(uint8_t byte)
{
	min_tx_byte(byte);
	fletcher16_tx_step(byte);
	
	if(byte == HEADER_BYTE) {
		if(--tx_header_byte_countdown == 0) {
			min_tx_byte(STUFF_BYTE);		/* Stuff byte */
			tx_header_byte_countdown = 2U;
		}
	}
	else {
		tx_header_byte_countdown = 2U;
	}
}

/* Main transmit function. Calls out to a byte transmit function,
 * typically connected to a FIFO with a UART driver.
 *
 * payload is a pointer to a buffer containing the rest of the payload,
 * and id is the frame ID within a message set.
 *
 * control encodes the length of the frame in the bottom four bits. The
 * top four bits are reserved and must be passed through unchanged.
 *
 * Frames should not be transmitted until sending is enabled (so that
 * the system can be quiescent until other devices have initialized).
 *
 */
void min_tx_frame(uint8_t id, uint8_t payload[], uint8_t control)
{
	uint8_t n, i;
	uint16_t checksum;
	uint8_t length = control & FRAME_LENGTH_MASK;

	uint8_t tx_space;
	
	/* Don't bother trying to send if sending is disabled */
	if(!min_tx_enabled) {
		return;
	}
	
	tx_space = min_tx_space();
	
	/* Don't even bother trying to send if there's not guaranteed to be enough space for the frame */
	if(tx_space < MAX_FRAME_SIZE) {
		if(++min_tx_frame_no_space == 0) {
			/* Make counter stick at 255; TODO put some standard macros in place to do this for lots of counters */
			min_tx_frame_no_space = 0xffU;
		}
		return;
	}
	if(tx_space < min_lowest_tx_space) {
		min_lowest_tx_space = tx_space;
	}

	tx_header_byte_countdown = 2U;
	fletcher16_tx_init();

	/* Header is 3 bytes; because unstuffed will reset receiver immediately */
	min_tx_byte(HEADER_BYTE);
	min_tx_byte(HEADER_BYTE);
	min_tx_byte(HEADER_BYTE);

	stuffed_tx_byte(id);

	stuffed_tx_byte(control);

	for(i = 0, n = length; n > 0; n--, i++) {
		stuffed_tx_byte(payload[i]);
	}

	checksum = fletcher16_tx_finalize();

	/* Network order is big-endian */
	stuffed_tx_byte(checksum >> 8);
	stuffed_tx_byte(checksum &0x00ff);

	/* Ensure end-of-frame doesn't contain 0xaa and confuse search for start-of-frame */
	min_tx_byte(EOF_BYTE);
}

