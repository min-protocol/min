/* Reference implementation of Layer 1 (frame) of MIN 1.0
 * 
 * Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */

#include "min.h"

/* Maximum size of a MIN frame:
 *
 * 3 (header) + 1 (id) + 1 (control) + 15 (payload) + 2 (checksum) + 1 (EOF) + stuff bytes
 *
 * Stuff bytes can be inserted (in worst case) after every 2 bytes in id/control/payload/checksum
 * = (1 + 1 + 15 + 2) / 2 = 10.
 *
 * Maximum size of a frame is therefore 32 bytes.
 */
#define MAX_FRAME_SIZE					(33U)
#define FRAME_LENGTH_MASK               (0x0fU)

/* Magic bytes */
#define HEADER_BYTE						(0xaaU)
#define STUFF_BYTE						(0x55U)
#define EOF_BYTE						(0x55U)

/* Receiving state machine */
#define SEARCHING_FOR_SOF 			    (0)
#define RECEIVING_ID				    (1U)
#define RECEIVING_CONTROL			    (2U)
#define RECEIVING_PAYLOAD			    (3U)
#define RECEIVING_CHECKSUM_HIGH		    (4U)
#define RECEIVING_CHECKSUM_LOW		    (5U)
#define RECEIVING_EOF				    (6U)

static uint8_t rx_frame_buf[MAX_FRAME_PAYLOAD_SIZE];

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

void min_init_layer1(void)
{
    rx_header_bytes_seen = 0;
    rx_frame_state = SEARCHING_FOR_SOF;
}

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
	
	tx_space = min_tx_space();
	
	/* Don't even bother trying to send if there's not guaranteed to be enough space for the frame */
	if(tx_space < MAX_FRAME_SIZE) {
		return;
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
