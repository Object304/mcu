/*
 * RingBuffer.h
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "stm32f3xx.h"

#define BUF_SIZE 512

typedef struct {
	uint8_t data[BUF_SIZE];
	uint16_t head;
	uint16_t tail;
} RingBuffer;
void init_buffer(RingBuffer* buf);
int get_from_head(uint8_t* byte, RingBuffer* buf);
int get_from_tail(uint8_t* byte, RingBuffer* buf);
int add_to_end(uint8_t byte, RingBuffer* buf);


#endif /* RINGBUFFER_H_ */
