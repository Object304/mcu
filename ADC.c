/*
 * ADC.c
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#include "ADC.h"

#define MAX_ADC_CHANNELS 4

uint8_t active_channels[MAX_ADC_CHANNELS];
uint8_t num_channels = 0;

void DMA1_Channel1_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF1) {
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		data_ready = 1;
	}
}

void adc_set_pc_channels(uint8_t *pc_pins, uint8_t count) { // pc0-pc3
	if (count == 0 || count > MAX_ADC_CHANNELS) return;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Настройка пинов PC0–PC3 как аналоговых
	for (uint8_t i = 0; i < count; i++) {
		uint8_t pc_pin = pc_pins[i];
		if (pc_pin > 3) continue; // Только PC0–PC3 допустимы
		GPIOC->MODER |= (3 << (2 * pc_pin)); // Analog mode
		active_channels[i] = pc_pin + 6; // PC0 - ADC IN6, PC1 - IN7, ...
	}
	num_channels = count;
	// Настройка SQR1
	ADC1->SQR1 = 0;
	for (uint8_t i = 0; i < count; i++) {
		ADC1->SQR1 |= (active_channels[i] << (6 + i * 6)); // SQ1, SQ2, ...
	}
	ADC1->SQR1 |= (count - 1); // L: количество каналов - 1
}

void adc_set_sampling_time(uint8_t smp_bits) {
	if (smp_bits > 0b111) return; // допустимые значения: 0–7
	// Очистка битов SMP6–SMP9 в SMPR1 (по 3 бита на каждый канал)
	ADC1->SMPR1 &= ~(
		(0b111 << 18) | // SMP6 (PC0 - IN6)
		(0b111 << 21) | // SMP7 (PC1 - IN7)
		(0b111 << 24) | // SMP8 (PC2 - IN8)
		(0b111 << 27)   // SMP9 (PC3 - IN9)
	);
	// Установка одинакового значения smp_bits для всех 4 каналов
	ADC1->SMPR1 |=
		(smp_bits << 18) | // SMP6
		(smp_bits << 21) | // SMP7
		(smp_bits << 24) | // SMP8
		(smp_bits << 27);  // SMP9
}

void stop_adc_dma() {
	RCC->AHBRSTR |= RCC_AHBRSTR_ADC12RST;
	RCC->AHBRSTR &= ~RCC_AHBRSTR_ADC12RST;

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CCR = 0;
	DMA1_Channel1->CNDTR = 0;
	DMA1_Channel1->CPAR = 0;
	DMA1_Channel1->CMAR = 0;
}


void adc_start_burst(uint16_t n_total_samples) {
	uint16_t dma_count = n_total_samples * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;

	samples_count = dma_count;

	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = samples_count;
	DMA1_Channel1->CCR = DMA_CCR_MINC |
						 DMA_CCR_MSIZE_0 |
						 DMA_CCR_PSIZE_0 |
						 DMA_CCR_TCIE;
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	DMA1_Channel1->CCR |= DMA_CCR_EN;


	// Настройка ADC
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	ADC1->CFGR &= ~ADC_CFGR_CONT;
	ADC1->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_OVRMOD;

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));

	for (uint16_t i = 0; i < n_total_samples; i++) {
		while (ADC1->CR & ADC_CR_ADSTART);
		ADC1->CR |= ADC_CR_ADSTART;
		while (!(ADC1->ISR & ADC_ISR_EOS)); // дождаться окончания
		ADC1->ISR |= ADC_ISR_EOS;
	}
}

void adc_start_cycle(uint16_t n_per_cycle) {
	uint16_t dma_count = n_per_cycle * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;

	samples_count = dma_count;
	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = samples_count;

	DMA1_Channel1->CCR = DMA_CCR_MINC
					   | DMA_CCR_CIRC
					   | DMA_CCR_TCIE
					   | DMA_CCR_MSIZE_0
					   | DMA_CCR_PSIZE_0;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	if (ADC1->CR & ADC_CR_ADEN) {
		ADC1->CR |= ADC_CR_ADDIS;
		while (ADC1->CR & ADC_CR_ADEN);
	}

	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	for (uint16_t i = 0; i < 1000; i++);

	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	ADC1->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG);
	ADC1->CFGR |= ADC_CFGR_DMAEN
				| ADC_CFGR_DMACFG
				| ADC_CFGR_CONT
				| ADC_CFGR_OVRMOD;

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));
	ADC1->CR |= ADC_CR_ADSTART;
}

void adc1_read_temp() {
	ADC1->CR |= ADC_CR_ADSTART;
	while (!(ADC1->ISR & ADC_ISR_EOC));
	uint16_t raw = ADC1->DR;

	// float temp = ((1.43f - ((raw * 3.3f) / 4095.0f)) * 1000.0f / 4.3f) + 25.0f;
	usart_tx_buffer[0] = raw & 0xFF;
	usart_tx_buffer[1] = (raw >> 8) & 0xFF;
	usart_dma_send(2);
}

void adc1_init_temp_sensor() {
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= ADC12_CCR_TSEN; // Температурный датчик включен

	ADC1->CR &= ~ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));

	ADC1->SQR1 = (16 << 6); // Канал 16 в первый слот

	// Максимальное время выборки для канала 16
	ADC1->SMPR1 |= (7 << 18);

	ADC1->CFGR &= ~ADC_CFGR_CONT;
}

void adc_start() {
	uint8_t byte;
	get_from_tail(&byte, &command_data_buf);
	uint8_t count = byte & 0x0F;
	uint8_t channels_raw = byte >> 4;
	uint8_t channels[count];
	uint8_t counter = 0;
	for (uint8_t i = 0; i < 4; i++) {
		if ((channels_raw >> i) & 1) {
			channels[counter] = 3 - i;
			counter++;
		}
	}
	adc_set_pc_channels(channels, count);

	get_from_tail(&byte, &command_data_buf);
	uint8_t speed = byte >> 4;
	uint8_t mode = byte & 0x0F;
	adc_set_sampling_time(speed);

	get_from_tail(&byte, &command_data_buf);
	uint16_t n_samples = 0;
	n_samples |= byte << 8;
	get_from_tail(&byte, &command_data_buf);
	n_samples |= byte;
	if (mode == 0) adc_start_cycle(n_samples);
	if (mode == 1) adc_start_burst(n_samples);
}
