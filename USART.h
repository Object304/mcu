/*
 * USART.h
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f3xx.h"
#include "globals.h"
#include "RingBuffer.h"

int process_command();
void USART2_IRQHandler();
void prepare_usart_tx_buffer(uint16_t start, uint16_t length);
void usart_dma_send(uint16_t length);
void data_convert();
void DMA1_Channel7_IRQHandler();
void init_usart_dma_tx();
void init_pll_usart();

void set_interval();

#endif /* USART_H_ */
