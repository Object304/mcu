/*
 * globals.h
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "RingBuffer.h"
#include "stm32f3xx.h"

extern volatile uint16_t samples_count;
extern volatile uint8_t data_ready;

#define ADC_BUF_SIZE 512
extern volatile uint16_t dma_data[ADC_BUF_SIZE];
extern volatile uint8_t usart_tx_buffer[ADC_BUF_SIZE * 2 + 4];

extern volatile RingBuffer rx_buf;
extern volatile RingBuffer command_buf;
extern volatile RingBuffer command_data_buf;
extern volatile uint8_t cmd;

extern volatile uint8_t command_ready;

extern volatile uint8_t mode_type;

extern volatile uint8_t adc_size;

#endif /* GLOBALS_H_ */
