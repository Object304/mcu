/*
 * USART.c
 *
 *  Created on: 22 мая 2025 г.
 *      Author: Zhon1
 */

#include "USART.h"

volatile uint8_t dma_tx_busy = 0;
volatile uint8_t sync_byte_received = 0;
volatile uint8_t byte_counter = 0;

volatile uint8_t big_data = 0;

// USART receive

int process_command() {
	if (byte_counter < 10) return 1; // команда не сформирована
	uint8_t byte;
	uint8_t sync;
	uint8_t time1;
	uint8_t time2;
	get_from_tail(&sync, &command_buf);
	get_from_tail(&cmd, &command_buf);
	get_from_tail(&time1, &command_buf);
	get_from_tail(&time2, &command_buf);
	uint8_t xor = sync ^ cmd ^ time1 ^ time2;
	uint8_t temp_data_buf[5];
	for (uint8_t i = 0; i < 5; i++) {
		get_from_tail(&byte, &command_buf);
		temp_data_buf[i] = byte;
		xor ^= byte;
	}
	get_from_tail(&byte, &command_buf);
	if ((xor ^ byte) != 0) {
		return 2; // данные повреждены
	}

	if (cmd == 0x06) {
		big_data = 1;
		init_buffer(&command_data_buf);
	}
	else if (cmd == 0x07)
		big_data = 0; // no init
	else {
		init_buffer(&command_data_buf);
		for (uint8_t i = 0; i < 5; i++) add_to_end(temp_data_buf[i], &command_data_buf);
	}
	sync_byte_received = 0;
	byte_counter = 0;

	command_ready = 1;

	return 0;
}

void USART2_IRQHandler() {
	if (USART2->ISR & USART_ISR_RXNE) {
		uint8_t byte = USART2->RDR;
		add_to_end(byte, &rx_buf);
		if (byte == 0xAA) sync_byte_received = 1;

		if (big_data && !sync_byte_received) {
			add_to_end(byte, &command_data_buf);
		}

		if (sync_byte_received) {
			byte_counter++;
			add_to_end(byte, &command_buf);
		}
	}
}

// USART transmit

void prepare_usart_tx_buffer(uint16_t start, uint16_t length) {
	for (uint16_t i = 0; i < length; ++i) {
		uint16_t value = dma_data[start + i];
		usart_tx_buffer[2*i]     = value & 0xFF;        // LSB (LSB first)
		usart_tx_buffer[2*i + 1] = (value >> 8) & 0xFF; // MSB
	}
}

void usart_dma_send(uint16_t length) {
	DMA1_Channel7->CCR &= ~DMA_CCR_EN; // остановка
	DMA1_Channel7->CNDTR = length; // сколько байт передать
	DMA1_Channel7->CCR |= DMA_CCR_EN;  // запуск
}

void data_convert() {
	if (data_ready == 1) {
		prepare_usart_tx_buffer(0, samples_count);
		usart_dma_send(samples_count * 2);
		data_ready = 0;
	}
}

void DMA1_Channel7_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF7) {
		DMA1->IFCR |= DMA_IFCR_CTCIF7; // сброс флага
	}
}

void init_usart_dma_tx() {
	// включаем DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;
	DMA1_Channel7->CMAR = (uint32_t)usart_tx_buffer;
	DMA1_Channel7->CCR = DMA_CCR_MINC |
						 DMA_CCR_DIR |
						 DMA_CCR_TCIE;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void init_pll_usart() {
	//init clock as pll
	RCC->CR &= ~RCC_CR_PLLON;

	uint32_t timeout = SystemCoreClock;
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	RCC->CFGR |= (1 << 15);	// 8MHz
	RCC->CFGR |= RCC_CFGR_PLLMUL9;	// 8Mhz * 9 = 72MHz
	RCC->CR |= RCC_CR_PLLON;

	timeout = SystemCoreClock;
	while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1; //clock for adc12 is on
	RCC->CFGR |= RCC_CFGR_SW_PLL; // set pll as main clock

	timeout = SystemCoreClock;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	SystemCoreClockUpdate();

	//init usart
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->AFR[0] |= (7 << 8) | (7 << 12); // выбрали альтернативную функцию 7 для портов 2 и 3
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = SystemCoreClock / 921600;
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // transmitter, receiver enable
	USART2->CR1 |= USART_CR1_RXNEIE; // interrupt enable
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR3 |= USART_CR3_DMAT; // enable dma mode for transmission
	USART2->CR1 |= USART_CR1_UE; // usart enable
}

void set_interval() {

}

