/*
 * fifo.c
 *
 * Generic FIFO handler
 */ 

#include "fifo.h"

void fifo_init(struct fifo *f, uint8_t buf[], uint8_t size)
{
	f->used = 0;
	f->head = f->tail = 0;
	f->buf = buf;
	f->size = size;	
	f->max_used = 0;
}

void fifo_write(struct fifo *f, uint8_t b)
{
	if(!FIFO_FULL(f)) {						/* Only proceed if there's space */
		f->used++;							/* Keep track of the number of used bytes in the FIFO */
		f->buf[f->head++] = b;				/* Add to where head indexes */
		if(f->head == f->size) {			/* If head goes off the end of the FIFO buffer then wrap it */
			f->head = 0;
		}	
	}
	if(f->used > f->max_used) {				/* For diagnostics keep a high-water mark of used space */
		f->max_used = f->used;
	}
}

uint8_t fifo_read(struct fifo *f)
{
	uint8_t ret = 0;
	
	if(!FIFO_EMPTY(f)) {					/* Only proceed if there's something there */
		f->used--;							/* Keep track of the used space */
		ret = f->buf[f->tail++];
		if(f->tail == f->size) {			/* Wrap tail around if it goes off the end of the buffer */
			f->tail = 0;
		}
	}
	
	return ret;
}
