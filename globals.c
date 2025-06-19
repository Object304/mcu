/*
 * globals.c
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#include "globals.h"

volatile uint16_t samples_count = 0;
volatile uint8_t data_ready = 0;
volatile uint16_t dma_data[ADC_BUF_SIZE];
volatile uint8_t usart_tx_buffer[ADC_BUF_SIZE * 2 + 4];

volatile RingBuffer rx_buf;
volatile RingBuffer command_buf;
volatile RingBuffer command_data_buf;
volatile uint8_t cmd = 0xFF;

volatile uint8_t command_ready = 0;

volatile uint8_t mode_type = 0;

volatile uint8_t adc_size = 0;
