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

void adc_off() {
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	ADC1->CR |= ADC_CR_ADSTP;
	ADC1->CR |= ADC_CR_ADDIS;
}

void adc_on() {
	uint8_t byte;
	get_from_tail(&byte, &command_data_buf);
	uint8_t mode = byte;
	get_from_tail(&byte, &command_data_buf);
	uint16_t n_samples = 0;
	n_samples |= byte << 8;
	get_from_tail(&byte, &command_data_buf);
	n_samples |= byte;
	if (mode == 0) adc_start_cycle(n_samples);
	if (mode == 1) adc_start_burst(n_samples);

}

void adc_set_channels() { //pc0-pc3
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

	if (count == 0 || count > MAX_ADC_CHANNELS) return;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	for (uint8_t i = 0; i < count; i++) {
		uint8_t pc_pin = channels[i];
		if (pc_pin > 3) continue;
		GPIOC->MODER |= (3 << (2 * pc_pin)); // Analog mode
		active_channels[i] = pc_pin + 6; // PC0 - ADC IN6, PC1 - IN7, ...
	}
	num_channels = count;
	ADC1->SQR1 = 0;
	for (uint8_t i = 0; i < count; i++) {
		ADC1->SQR1 |= (active_channels[i] << (6 + i * 6)); // SQ1, SQ2, ...
	}
	ADC1->SQR1 |= (count - 1);
}

void adc_set_freq() {
	uint8_t byte;
	uint32_t freq = 0;
	for (uint8_t i = 4; i > 0; i--) {
		get_from_tail(&byte, &command_data_buf);
		freq |= byte << 8 * (i - 1);
	}

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = 0;
	TIM2->CNT = 0;
	TIM2->PSC = 0;
	// freq = SystemCoreClock / ((PSC + 1) * (ARR + 1))
	uint32_t arr = (SystemCoreClock / freq) - 1;
	TIM2->ARR = arr;
	TIM2->CR2 &= ~TIM_CR2_MMS;
	TIM2->CR2 |= TIM_CR2_MMS_1; // MMS = 010: Update event for TRGO
	TIM2->CR1 |= TIM_CR1_CEN;
}

void adc_set_size() {
	uint8_t byte;
	get_from_tail(&byte, &command_data_buf);
	byte &= 0b11;
	ADC1->CFGR &= ~ADC_CFGR_RES;
	ADC1->CFGR |= (byte << 3);
}

void adc_start_burst(uint16_t n_total_samples) {
	uint16_t dma_count = n_total_samples * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;
	samples_count = dma_count;
	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CNDTR = samples_count;
	DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	ADC1->CFGR &= ~ADC_CFGR_CONT;
	ADC1->CR |= ADC_CR_ADEN;
	for (uint16_t i = 0; i < 1000; i++);
	for (uint16_t i = 0; i < n_total_samples; i++) {
		ADC1->CR |= ADC_CR_ADSTART;

		uint32_t timeout = SystemCoreClock;
		while (!(ADC1->ISR & ADC_ISR_EOS) && timeout--);
		if (timeout == 0) NVIC_SystemReset();
		ADC1->ISR |= ADC_ISR_EOS;
	}
}

void adc_start_cycle(uint16_t n_per_cycle) {
	uint16_t dma_count = n_per_cycle * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;
	samples_count = dma_count;
	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CNDTR = samples_count;
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	ADC1->CFGR |= ADC_CFGR_CONT;
	ADC1->CR |= ADC_CR_ADEN;
	for (uint16_t i = 0; i < 1000; i++);
	ADC1->CR |= ADC_CR_ADSTART;
}

void adc_read_temp() {
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	ADC1->CR |= ADC_CR_ADSTP;
	ADC1->CR |= ADC_CR_ADDIS;

	ADC1->SQR1 = (16 << 6); // Канал 16 в первый слот
	ADC1->SMPR1 |= (7 << 18);
	ADC1->CFGR &= ~ADC_CFGR_CONT;

	ADC1->CR |= ADC_CR_ADEN;
	for (uint16_t i = 0; i < 1000; i++);
	ADC1->CR |= ADC_CR_ADSTART;

	uint32_t timeout = SystemCoreClock;
	while (!(ADC1->ISR & ADC_ISR_EOS) && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	uint16_t raw = ADC1->DR;
	// float temp = ((1.43f - ((raw * 3.3f) / 4095.0f)) * 1000.0f / 4.3f) + 25.0f;
	usart_tx_buffer[0] = raw & 0xFF;
	usart_tx_buffer[1] = (raw >> 8) & 0xFF;
	usart_dma_send(2);
}

void init_adc_dma() {
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CCR = DMA_CCR_MINC
					   | DMA_CCR_TCIE
					   | DMA_CCR_MSIZE_0
					   | DMA_CCR_PSIZE_0;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= ADC12_CCR_TSEN; // Температурный датчик включен
	ADC1->CR |= ADC_CR_ADDIS;
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	ADC1->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_OVRMOD | ADC_CFGR_DMACFG;
	ADC1->CFGR |= ADC_CFGR_EXTEN_0;
	ADC1->CFGR |= (0b1011 << 6); // TRGO tim2
}
