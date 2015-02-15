

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* Example of motor control request from the host */
extern uint8_t motor_requested;
extern uint32_t motor_position_request;
extern uint16_t motor_speed_request;

#define F_OSC_HZ							(16000000UL)		            /* 16MHz */

#define LOCK_INTERRUPTS(i)					{(i) = SREG; cli();}			/* NB: SIDE-EFFECT MACRO! */
#define UNLOCK_INTERRUPTS(i)				{SREG = i;}
