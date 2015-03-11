/* Layer 2 definitions (application layer).
 * 
 * Defines all the MIN setup for the application, selecting IDs to be used
 * and how to pack and unpack signal data and commands.
 * 
 * Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */ 

#ifndef LAYER2_H_
#define LAYER2_H_

/* Configure the UART speed (see serial.h for binding to UART and FIFO buffer sizes) */
#define MIN_BAUD                        (UART_BAUD_9600)

/* Command frames from the host */
#define MIN_ID_PING						(0x02U)			/* Layer 1 frame; Ping test: returns the same frame back */
#define MIN_ID_MOTOR_REQUEST			(0x36u)			/* Layer 1 frame; Set requested motor position */

/* Report messages to the host */
#define MIN_ID_DEADBEEF					(0x0eU)			/* Layer 1 frame; Payload always 4 bytes = 0xdeadbeef */
#define MIN_ID_ENVIRONMENT				(0x23U)			/* Layer 1 frame; Temperature and humidity sensor values */
#define MIN_ID_MOTOR_STATUS				(0x24U)			/* Layer 1 frame; Report the status of the motor */

/* Motor control request from the host */
extern uint8_t motor_requested;
extern uint32_t motor_position_request;
extern uint16_t motor_speed_request;

/* Set up communications */
void init_min(void);

/* Poll the incoming uart to send bytes into MIN layer 1 */
void poll_rx_bytes(void);

/* Functions to take application data and send to host */
void report_environment(uint16_t temperature, uint16_t humidity);
void report_motor(uint8_t status, uint32_t position);
void report_deadbeef(uint32_t deadbeef);

#endif /* LAYER2_H_ */
