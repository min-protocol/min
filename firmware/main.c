/*
 * main.c
 *
 * Example program to illustrate MIN handling
 * 
 * Coded for an Atmel megaAVR 8-bit device. UART interrupt handlers
 * are coupled with a receive and a transmit FIFO to decouple
 * application code from ISRs.
 */ 

#include "main.h"

uint32_t motor_position_request;
uint16_t motor_speed_request;
uint8_t motor_requested;

uint16_t t = 0;

int main(void)
{
    /* Initialise the application layer for messaging as well as MIN */
	init_messages();
	
	/* CPU starts with interrupts disabled so enable them now */
	sei();					
	
	/* Simple real-time loop, run every 100ms */
	for(;;) {
        /* Run the message handler every 10ms */
        if(t % 10 == 0) {
            do_incoming_messages();
            if(motor_requested) {
                /* Tell the motor to do something */
                motor_requested = 0;
            }
        }
        /* Every 100ms send environment report */
        if(t % 100 == 1U) {
            report_environment(2000U, 5000U);
        }
        if(t % 100 == 10U) {
            report_motor(5U, 99999U);
        }
        if(t % 100 == 15U) {
            report_deadbeef(0xdeadbeefU);
        }
	}
}
