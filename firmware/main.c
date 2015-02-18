/* Example program to illustrate MIN handling
 * 
 * Coded for an Atmel megaAVR 8-bit device. UART interrupt handlers
 * are coupled with a receive and a transmit FIFO to decouple
 * application code from ISRs.
 *
 * Built with AtmelStudio 6 and run and tested on Arduino Mega 2560
 * using Dragon JTAG debugger.
 *
 * Board wired up as follows:
 *
 * Arduino <---> JTAG
 * A4 ---------- Pin 1
 * GND --------- Pin 2
 * A6 ---------- Pin 3
 * +5V --------- Pin 4
 * A5 ---------- Pin 5
 *        N/C -- Pin 6
 *        N/C -- Pin 7
 *        N/C -- Pin 8
 * A7 ---------- Pin 9
 *        N/C -- Pin 10
 *
 * Push button "dead beef" switch that when pushed connects 22 (PA6) to GND.
 * (PA6 is set to input with internal pullup resistor. Will read 0 when pushed)
 * +5 to 330 ohm resistor to LED to 24 (PA2). PA2 is set as an output; 0 will light LED.
 *
 * FTDI USB breakout board, with RX and TX wired up to TX0 and RX0 and USB connection
 * to a PC running Windows (appears as COM4 but this will vary).
 *
 *
 *
 * Author: Ken Tindell
 * Copyright (c) 2014-2015 JK Energy Ltd.
 * Licensed under MIT License.
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "layer2.h"

uint8_t porta;

uint8_t deadbeef_pressed = 0;
uint16_t environment_countup = 0;
uint16_t led_countup = 0;

#define LED_PIN			PA2
#define TEST_PIN		PA4											/* PA4 = output to use as test pin for a scope */
#define SWITCH_PIN		PA6

int main(void)
{
	init_min();														/* Initializes the application layer for messaging as well as MIN */
	
	/* Use timer/counter 0 (8-bit) to implement delays */
	TCCR0B = 0x05U;													/* Use Clock/1024; @ 16MHz clock 8ms = 125 counts */
	TCNT0 = 0;
	
	/* LED port */
	DDRA = (1U << LED_PIN) | (1U << TEST_PIN);
	PORTA = (1U << LED_PIN) | (1U << SWITCH_PIN);					/* Set LED off; the switch input pull-up enabled */
	
	/* CPU starts with interrupts disabled so enable them now */
	sei();					
	
	for(;;) {
		
		if(TCNT0 >= 125U) {											/* Succeeds every 8ms */
			PORTA ^= (1U << TEST_PIN);								/* Toggle so scope can see 8ms square wave */
			TCNT0 = 0;
			
			poll_rx_bytes();										/* Check the messages that have arrived in last 8ms */
			
			if(motor_requested) {									/* Host controller has said to do something with motor so just light an LED */
				PORTA &= ~(1U << LED_PIN);							/* Set to zero to light LED */
            }
			if(motor_requested && ++led_countup >= (1000U / 8U)) {	/* 1 second */
				PORTA |= (1U << LED_PIN);							/* Set to 1, LED off */
				motor_requested = 0;								/* No longer processing motor request */
				led_countup = 0;
			}
			if((PINA & (1U << SWITCH_PIN)) == 0) {					/* 0 = deadbeef switch pressed */
				if(!deadbeef_pressed) {								/* Edge detected (effectively debouncing switch) */
					report_deadbeef(0xdeadbeef);					/* Send message to host */
				}
				deadbeef_pressed = 1U;
			}
			else {
				deadbeef_pressed = 0;
			}
			if(++environment_countup >= (2000U / 8U)) {				/* 2 seconds */
				report_environment(1000U, 20U);						/* Report temperature and humidity values */
				environment_countup = 0U;	
			}
		}
	}
}
