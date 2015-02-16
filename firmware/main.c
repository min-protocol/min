/*
 * main.c
 *
 * Example program to illustrate MIN handling
 * 
 * Coded for an Atmel megaAVR 8-bit device. UART interrupt handlers
 * are coupled with a receive and a transmit FIFO to decouple
 * application code from ISRs.
 * 
 * Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "layer2.h"

uint16_t t = 0;

int main(void)
{
    /* Initialise the application layer for messaging as well as MIN */
	init_min();
	
	/* CPU starts with interrupts disabled so enable them now */
	sei();					
	
	/* Simple real-time loop, run every 100ms */
	for(;;) {
        /* Run the message handler every 10ms */
        if(t % 10 == 0) {
            /* Push any outstanding bytes into MIN handler */
            poll_rx_bytes();
            if(motor_requested) {
                /* Tell the motor to do something */
                motor_requested = 0;
            }
        }
        if(t % 100 == 10U) {
            report_environment(2000U, 5000U);
        }
        if(t % 100 == 20U) {
            report_motor(5U, 99999U);
        }
        if(t % 100 == 30U) {
            report_deadbeef(0xdeadbeefU);
        }
	}
}
