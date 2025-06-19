/*
 * USART.h
 *
 *  Created on: 22 ��� 2025 �.
 *      Author: Zhon1
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f3xx.h"
#include "globals.h"
#include "RingBuffer.h"

int process_command();
void USART2_IRQHandler();
uint16_t prepare_usart_tx_buffer(uint16_t length);
void usart_dma_send(uint16_t length);
void send_notification(uint8_t type);
void data_convert();
void DMA1_Channel7_IRQHandler();
void init_usart_dma_tx();
void init_pll_usart();

void init_tim3(uint16_t period_ms);
void start_tim3();
void stop_tim3();
void TIM3_IRQHandler();
void set_usart_mode(uint8_t type, uint16_t interval_ms);
void set_interval();

#endif /* USART_H_ */
