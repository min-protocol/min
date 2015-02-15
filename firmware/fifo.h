/*
 * fifo.h
 *
 * Created: 28/04/2014 11:31:57
 *  Author: ken_000
 */ 


#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>

/* 
 * A module that creates a FIFO for interfacing between main loop tasks and interrupt handlers (to drive access
 * to hardware).
 *
 */
struct fifo {
	uint8_t *buf;		/* Space allocated to FIFO */
	uint8_t size;		/* Size of FIFO */
	uint8_t head;		/* Indexes first free byte (unless full) */
	uint8_t tail;		/* Indexes last filled byte (unless empty) */
	uint8_t used;		/* Between 0..size */
	uint8_t max_used;	/* Maximum space used in the FIFO */
};

void fifo_init(struct fifo *f, uint8_t buf[], uint8_t size);			/* Initialize FIFO with a buffer of a given size */
void fifo_write(struct fifo *f, uint8_t b);								/* Write a byte to the FIFO; null operation if full */
uint8_t fifo_read(struct fifo *f);										/* Read a byte from the FIFO; return value is undefined if FIFO was empty */
uint8_t fifo_max_used(struct fifo *f);									/* Maximum amount of FIFO space that's been used */

#define FIFO_EMPTY(f)	((f)->used == 0)
#define FIFO_FULL(f)	((f)->used == (f)->size)
#define FIFO_USED(f)	((f)->used)

#endif /* FIFO_H_ */
