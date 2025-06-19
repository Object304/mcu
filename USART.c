/*
 * USART.c
 *
 *  Created on: 22 ма€ 2025 г.
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

uint16_t prepare_usart_tx_buffer(uint16_t length) {
	uint8_t bit_width;
	if (adc_size == 00) bit_width = 12;
	if (adc_size == 01) bit_width = 10;
	if (adc_size == 02) bit_width = 8;
	if (adc_size == 03) bit_width = 6;

	uint32_t bit_buf = 0;   // буфер битов
	uint8_t bit_pos = 0;    // сколько битов уже в буфере
	uint16_t out_idx = 0;

	for (uint16_t i = 0; i < length; i++) {
		uint16_t val = dma_data[i] & ((1 << bit_width) - 1);  // обрезаем по нужной разр€дности
		bit_buf |= ((uint32_t)val << bit_pos);
		bit_pos += bit_width;

		while (bit_pos >= 8) {
			usart_tx_buffer[out_idx++] = bit_buf & 0xFF;
			bit_buf >>= 8;
			bit_pos -= 8;
		}
	}

	// записываем остаток (если остались непросмотренные биты)
	if (bit_pos > 0) {
		usart_tx_buffer[out_idx++] = bit_buf & 0xFF;
	}

	return out_idx;
}

void usart_dma_send(uint16_t length) {
	DMA1_Channel7->CCR &= ~DMA_CCR_EN; // остановка
	DMA1_Channel7->CNDTR = length; // сколько байт передать
	DMA1_Channel7->CCR |= DMA_CCR_EN;  // запуск
}

void send_notification(uint8_t type) {
	if (type == 0) {
		uint8_t xor = 0xAA;
		usart_tx_buffer[0] = 0xAA;
		usart_tx_buffer[1] = type;
		usart_tx_buffer[2] = 0x00;
		usart_tx_buffer[3] = 0x00;
		usart_tx_buffer[4] = samples_count & 0xFF; // lsb first
		usart_tx_buffer[5] = (samples_count >> 8) & 0xFF;
		usart_tx_buffer[6] = adc_size;
		usart_tx_buffer[7] = 0x00;
		usart_tx_buffer[8] = 0x00;
		for (uint8_t i = 1; i < 9; i++) {
			xor ^= usart_tx_buffer[i];
		}
		usart_tx_buffer[9] = xor;
	}
	if (type == 1) {
		uint8_t xor = 0xAA;
		usart_tx_buffer[0] = 0xAA;
		usart_tx_buffer[1] = type;
		usart_tx_buffer[2] = 0x00;
		usart_tx_buffer[3] = 0x00;
		usart_tx_buffer[4] = 0x00;
		usart_tx_buffer[5] = 0x00;
		usart_tx_buffer[6] = 0x00;
		usart_tx_buffer[7] = 0x00;
		usart_tx_buffer[8] = 0x00;
		for (uint8_t i = 1; i < 9; i++) {
			xor ^= usart_tx_buffer[i];
		}
		usart_tx_buffer[9] = xor;
	}
}

void data_convert() {
	if (data_ready == 1) {
		send_notification(0);
		usart_dma_send(10);

		usart_dma_send(prepare_usart_tx_buffer(samples_count));

		send_notification(1);
		usart_dma_send(10);
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

	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
	RCC->CFGR |= RCC_CFGR_PLLMUL16;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	RCC->CR |= RCC_CR_PLLON;

	timeout = SystemCoreClock;
	while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV12; //clock for adc12 is on
	RCC->CFGR |= RCC_CFGR_SW_PLL; // set pll as main clock

	timeout = SystemCoreClock;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL && timeout--);
	if (timeout == 0) NVIC_SystemReset();

	SystemCoreClockUpdate();

	//init usart
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->AFR[0] |= (7 << 8) | (7 << 12); // выбрали альтернативную функцию 7 дл€ портов 2 и 3
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = SystemCoreClock / 256000;
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // transmitter, receiver enable
	USART2->CR1 |= USART_CR1_RXNEIE; // interrupt enable
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR3 |= USART_CR3_DMAT; // enable dma mode for transmission
	USART2->CR1 |= USART_CR1_UE; // usart enable
}

void init_tim3(uint16_t period_ms) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = (SystemCoreClock / 1000) - 1; // частота таймера 1 к√ц (1 мс)
	TIM3->ARR = period_ms - 1;
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM3_IRQn);
}

void start_tim3() {
	TIM3->CNT = 0;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void stop_tim3() {
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void TIM3_IRQHandler() {
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF;

		if (data_ready) {
			data_convert();
		}
	}
}

void set_interval() {
	uint8_t byte;
	get_from_tail(&byte, &command_data_buf);
	mode_type = byte;
	uint16_t interval_ms = 0;
	get_from_tail(&byte, &command_data_buf);
	interval_ms |= (byte << 8);
	get_from_tail(&byte, &command_data_buf);
	interval_ms |= byte;
	if (mode_type == 0) {
		stop_tim3();
	}
	else if (mode_type == 1) {
		init_tim3(interval_ms);
		start_tim3();
	}
}
