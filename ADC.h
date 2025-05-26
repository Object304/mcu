/*
 * ADC.h
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f3xx.h"
#include "globals.h"
#include "RingBuffer.h"
#include "USART.h"

void DMA1_Channel1_IRQHandler();
void init_adc_dma();
void adc_off();
void adc_on();
void adc_set_channels();
void adc_set_freq();
void adc_set_size();
void set_block_size();

#endif /* ADC_H_ */
