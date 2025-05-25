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
void adc_set_pc_channels(uint8_t *pc_pins, uint8_t count);
void adc_set_sampling_time(uint8_t smp_bits);
void adc_start_burst(uint16_t n_total_samples);
void adc_start_cycle(uint16_t n_per_cycle);
void adc_read_temp();
void adc_start();
void init_adc_dma();

#endif /* ADC_H_ */
