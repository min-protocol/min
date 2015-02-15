/*
 * min.h
 *
 *  Created on: May 3, 2014
 *      Author: ken
 */

#ifndef MIN_H_
#define MIN_H_

#include <stdint.h>

/*
 * Microcontroller Interconnect Network (MIN) Version 1.0
 * 
 * Reference implementation
 */

#define MAX_FRAME_PAYLOAD_SIZE		(15U)

/* Transmit a MIN Layer 1 frame */
void min_tx_frame(uint8_t id, uint8_t payload[], uint8_t control);			

/* Called by UART handler when a byte is received */
void min_rx_byte(uint8_t byte);

/* Callback; ask application to queue a byte for given frame into its UART driver */
void min_tx_byte(uint8_t byte);										

/* Callback; indicate to application that a valid frame received */
void min_frame_received(uint8_t buf[], uint8_t control, uint8_t id);		

#endif /* MIN_H_ */
