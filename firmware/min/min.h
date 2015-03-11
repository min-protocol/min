/*
 * Microcontroller Interconnect Network (MIN) Version 1.0
 * 
 * Layer 1 C API.
 * 
 * Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */
#ifndef MIN_H_
#define MIN_H_

#include <stdint.h>

#define MAX_FRAME_PAYLOAD_SIZE		(15U)

/* Initialize Layer 1 */
void min_init_layer1(void);

/* Called by Layer 2 to transmit a MIN Layer 1 frame */
void min_tx_frame(uint8_t id, uint8_t payload[], uint8_t control);			

/* Called by Layer 2 when a byte is received from its UART handler */
void min_rx_byte(uint8_t byte);

/* Callback; ask Layer 2 to queue a byte into its UART handler */
void min_tx_byte(uint8_t byte);										

/* Callback; indicate to Layer 2 that a valid frame has been received */
void min_frame_received(uint8_t buf[], uint8_t control, uint8_t id);		

/* Callback; returns to MIN the space in the transmit FIFO */
uint8_t min_tx_space(void);

#endif /* MIN_H_ */
