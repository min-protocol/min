/*
 * messages.h
 *
 * Layer 3 (application layer).
 * 
 * Defines all the MIN messages across the entire system.
 *
 * Created: 17/05/2014 13:42:41
 *  Author: ken_000
 */ 

#ifndef MESSAGES_H_
#define MESSAGES_H_

/* Configure the UART and FIFO buffering to the host connection */
#define HOST_UART	                    (USART0)

/* Command frames from the host */
#define MIN_ID_PING						(0x02U)			/* Layer 1 frame; Ping test: returns the same frame back */
#define MIN_ID_MOTOR_REQUEST			(0x36u)			/* Layer 1 frame; Set requested motor position */

/* Report messages to the host */
#define MIN_ID_DEADBEEF					(0x0eU)			/* Layer 1 frame; Payload always 4 bytes = 0xdeadbeef */
#define MIN_ID_ENVIRONMENT				(0x23U)			/* Layer 1 frame; Temperature and humidity sensor values */
#define MIN_ID_MOTOR_STATUS				(0x24U)			/* Layer 1 frame; Report the status of the motor */

void init_messages(void);
void do_incoming_messages(void);
void report_environment(uint16_t temperature, uint16_t humidity);
void report_motor(uint8_t status, uint32_t position);
void report_deadbeef(uint32_t deadbeef);
void report_test1(void);

#endif /* MESSAGES_H_ */
