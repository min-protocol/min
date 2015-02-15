/*
 * framerx.c
 *
 *  Created on: May 5, 2014
 *      Author: ken
 */

#include "imin.h"

#define SEARCHING_FOR_SOF 			(0)
#define RECEIVING_ID				(1U)
#define RECEIVING_CONTROL			(2U)
#define RECEIVING_PAYLOAD			(3U)
#define RECEIVING_CHECKSUM_HIGH		(4U)
#define RECEIVING_CHECKSUM_LOW		(5U)
#define RECEIVING_EOF				(6U)

static uint8_t rx_frame_buf[MAX_FRAME_PAYLOAD_SIZE];
uint8_t min_lowest_rx_space;		/* Lowest space (in bytes) available in receive buffer */

/* Fletcher's checksum algorithm (for 16-bit checksums)
 *
 * Declared static so a good compiler will inline these calls.
 */
static uint16_t rx_sum1;
static uint16_t rx_sum2;

static void fletcher16_rx_init(void)
{
	rx_sum1 = 0x00ffu;
	rx_sum2 = 0x00ffu;
}

static void fletcher16_rx_step(uint8_t byte)
{
	rx_sum2 += rx_sum1 += byte;
}

static uint16_t fletcher16_rx_finalize(void)
{
	rx_sum1 = (rx_sum1 & 0x00ffu) + (rx_sum1 >> 8);
	rx_sum2 = (rx_sum2 & 0x00ffu) + (rx_sum2 >> 8);
	return rx_sum2 << 8 | rx_sum1;
}

static uint8_t rx_header_bytes_seen = 0;

static uint8_t rx_frame_state = SEARCHING_FOR_SOF;	/* State of receiver */
static uint16_t rx_frame_checksum;					/* Checksum received over the wire */
static uint8_t rx_payload_bytes;					/* Length of payload received so far */
static uint8_t rx_frame_id;							/* ID of frame being received */
static uint8_t rx_frame_length;						/* Length of frame */
static uint8_t rx_control;							/* Control byte */

/* Main receive function for a byte
 *
 * Typically called from a background task that's reading a FIFO in a loop.
 */
void min_rx_byte(uint8_t byte)
{
	/* Regardless of state, three header bytes means "start of frame" and
	 * should reset the frame buffer and be ready to receive frame data
	 *
	 * Two in a row in over the frame means to expect a stuff byte.
	 */
	if(rx_header_bytes_seen == 2) {
		rx_header_bytes_seen = 0;
		if(byte == HEADER_BYTE) {
			rx_frame_state = RECEIVING_ID;
			return;
		}
		if(byte == STUFF_BYTE) {
			/* Discard this byte; carry on receiving on the next character */
			return;
		}
		else {
			/* Something has gone wrong, give up on this frame and look for header again */
			rx_frame_state = SEARCHING_FOR_SOF;
			return;
		}
	}
	
	if(byte == HEADER_BYTE) {
		rx_header_bytes_seen++;
	}
	else {
		rx_header_bytes_seen = 0;
	}
	
	switch(rx_frame_state) {
	case SEARCHING_FOR_SOF:
		break;
	case RECEIVING_ID:
		rx_frame_id = byte;
		rx_payload_bytes = 0;
		fletcher16_rx_init();
		fletcher16_rx_step(byte);
		rx_frame_state = RECEIVING_CONTROL;
		break;
	case RECEIVING_CONTROL:
		rx_frame_length = byte & FRAME_LENGTH_MASK;
		rx_control = byte;
		fletcher16_rx_step(byte);
		if(rx_frame_length > 0) {
			rx_frame_state = RECEIVING_PAYLOAD;
		}
		else {
			rx_frame_state = RECEIVING_CHECKSUM_HIGH;
		}
		break;
	case RECEIVING_PAYLOAD:
		rx_frame_buf[rx_payload_bytes++] = byte;
		fletcher16_rx_step(byte);
		if(--rx_frame_length == 0) {
			rx_frame_state = RECEIVING_CHECKSUM_HIGH;
		}
		break;
	case RECEIVING_CHECKSUM_HIGH:
		rx_frame_checksum = (uint16_t)byte << 8;
		rx_frame_state = RECEIVING_CHECKSUM_LOW;
		break;
	case RECEIVING_CHECKSUM_LOW:
		rx_frame_checksum |= byte;
		if(rx_frame_checksum != fletcher16_rx_finalize()) {
			/* Frame fails the checksum and so is dropped */
			rx_frame_state = SEARCHING_FOR_SOF;
		}
		else {
			/* Checksum passes, go on to check for the end-of-frame marker */
			rx_frame_state = RECEIVING_EOF;
		}
		break;
	case RECEIVING_EOF:
		if(byte == 0x55u) {
			/* Frame received OK, pass up data to handler */
			min_frame_received(rx_frame_buf, rx_control, rx_frame_id);
		}
		/* else discard */
		/* Look for next frame */
		rx_frame_state = SEARCHING_FOR_SOF;
		break;
	}
}



