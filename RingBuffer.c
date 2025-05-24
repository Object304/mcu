/*
 * RingBuffer.c
 *
 *  Created on: 22 ��� 2025 �.
 *      Author: Zhon1
 */

#include "RingBuffer.h"

void init_buffer(RingBuffer* buf) {
	buf->head = 0;
	buf->tail = 0;
}

int get_from_head(uint8_t* byte, RingBuffer* buf) {
	if (buf->head == buf->tail) {
		return 0; // ����� ����
	}
	*byte = buf->data[buf->head - 1];
	buf->head = (buf->head == 0) ? BUF_SIZE - 1 : buf->head - 1;
	return 1;
}

int get_from_tail(uint8_t* byte, RingBuffer* buf) {
	if (buf->head == buf->tail) {
		return 0; // ����� ����
	}
	*byte = buf->data[buf->tail];
	buf->tail = (buf->tail + 1) % BUF_SIZE;
	return 1;
}

int add_to_end(uint8_t byte, RingBuffer* buf) {
	if ((buf->head + 1) % BUF_SIZE == buf->tail) {
		return 0; // ������������
	}
	buf->data[buf->head] = byte;
	buf->head = (buf->head + 1) % BUF_SIZE;
	return 1;
}
