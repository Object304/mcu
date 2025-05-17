//1 - sync
//2 - weight (0x00 - small, 0x01 - big)
//3 - cmd
//4 - time1 (min)
//5 - time2 (sec)
//6 - data0
//7 - data1
//8 - data2
//9 - data3
//10 - xor
//11 - end (0xC0)
//
//0x00 - start adc, data - set channels, speed, mode, amount of counts:
//0	x	0				0		0								0				0 0 0 0
//		0	0	0	0
//		pc0	pc1	pc2	pc3	count	000 0: 1.5 ADC clock cycles		0 - circular	amount
//		1 - on			1..4	001 1: 2.5 ADC clock cycles		1 - burst
//		0 - off					010 2: 4.5 ADC clock cycles
//								011 3: 7.5 ADC clock cycles
//								100 4: 19.5 ADC clock cycles
//								101 5: 61.5 ADC clock cycles
//								110 6: 181.5 ADC clock cycles
//								111 7: 601.5 ADC clock cycles





#include "stm32f3xx.h"

#define MAX_ADC_CHANNELS 4

uint8_t active_channels[MAX_ADC_CHANNELS];
uint8_t num_channels = 0;
volatile uint16_t samples_count = 0;

#define ADC_BUF_SIZE 512
volatile uint16_t dma_data[ADC_BUF_SIZE];
volatile uint8_t usart_tx_buffer[ADC_BUF_SIZE * 2];
volatile uint8_t data_ready = 0;

#define RX_BUF_SIZE 512
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

volatile uint8_t rx_data_buf[RX_BUF_SIZE];
volatile uint16_t rx_data_head = 0;
volatile uint16_t rx_data_tail = 0;
volatile uint8_t cmd;
volatile uint8_t cmd_received = 0;

volatile uint8_t click_processed = 0;

// USART receive

int get_from_head(uint8_t* byte) {
	if (rx_data_head == rx_data_tail) {
		return 0; // Буфер пуст
	}
	*byte = rx_data_buf[rx_data_head];
	rx_data_head = (rx_data_head == 0) ? RX_BUF_SIZE - 1 : rx_data_head - 1;
	return 1;
}

int get_from_tail(uint8_t* byte) {
	if (rx_data_head == rx_data_tail) {
		return 0; // Буфер пуст
	}
	*byte = rx_data_buf[rx_data_tail];
	rx_data_tail = (rx_data_tail + 1) % RX_BUF_SIZE;
	return 1;
}

int add_to_end(uint8_t byte) {
	if ((rx_data_head + 1) % RX_BUF_SIZE == rx_data_tail) {
		return 0; // переполнение
	}
	rx_data_buf[rx_data_head] = byte;
	rx_data_head = (rx_data_head + 1) % RX_BUF_SIZE;
	return 1;
}

int usart2_read_byte(uint8_t *data) {
    if (rx_head == rx_tail) {
        return 0; // Буфер пуст
    }
    *data = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

int process_command(){
	uint8_t byte;
	uint8_t sync = 0xAA;
	uint8_t weight;
	uint8_t time1;
	uint8_t time2;
	while (1) { // ищем байт инициализации
		if (!usart2_read_byte(&byte)) {
			return 1; // нет байта инициализации
		}
		if (byte != sync) {
			continue;
		}
		else {
			break;
		}
	}
	if (!usart2_read_byte(&weight)) return 2;
	if (!usart2_read_byte(&cmd)) return 2;
	if (!usart2_read_byte(&time1)) return 2;
	if (!usart2_read_byte(&time2)) return 2;
	uint8_t xor = sync ^ weight ^ cmd ^ time1 ^ time2;
	while (1) { // читаем что осталось
		if (!usart2_read_byte(&byte)) {
			return 2; // неполная команда
		}
		if (byte == 0xC0) {
			break;
		}
		add_to_end(byte);
		xor ^= byte;
	}
	if (xor != 0) {
		return 3; // данные повреждены
	}
	get_from_head(&byte);
	return 0;
}

void USART2_IRQHandler() {
    if (USART2->ISR & USART_ISR_RXNE) {
    	rx_buf[rx_head] = USART2->RDR;
    	if (rx_buf[rx_head] == 0xC0) cmd_received = 1;
		rx_head = (rx_head + 1) % RX_BUF_SIZE;
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
	DMA1_Channel7->CNDTR = length;     // сколько байт передать
	DMA1_Channel7->CCR |= DMA_CCR_EN;  // запуск
}

void data_convert() {
	if (data_ready == 1) {
		prepare_usart_tx_buffer(0, samples_count / 2);
		usart_dma_send(samples_count);
		data_ready = 0;
	}
	if (data_ready == 2) {
		prepare_usart_tx_buffer(samples_count / 2, samples_count / 2);
		usart_dma_send(samples_count);
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

	DMA1_Channel7->CCR =
		DMA_CCR_MINC      | // инкремент адреса памяти
		DMA_CCR_DIR       | // передача из памяти в периферию
		DMA_CCR_TCIE;       // прерывание по завершению

	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	// НЕ включаем здесь — включим при старте передачи
}

void init_pll_usart() {
	//init clock as pll
	RCC->CR &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;	// 8MHz / 2 = 4MHz
	RCC->CFGR |= RCC_CFGR_PLLMUL8;	// 4Mhz * 8 = 32MHz
	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV256; //clock for adc12 is on
	RCC->CFGR |= RCC_CFGR_SW_PLL; // set pll as main clock
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
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

// ADC

void DMA1_Channel1_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_HTIF1) {
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
		data_ready = 1;
	}
	if (DMA1->ISR & DMA_ISR_TCIF1) {
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		data_ready = 2;
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
	ADC1->CR |= ADC_CR_ADSTP; // остановить ADC
	while (ADC1->CR & ADC_CR_ADSTP); // дождаться завершения
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
}

void adc_start_burst(uint16_t n_total_samples) {
	stop_adc_dma();
	uint16_t dma_count = n_total_samples * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;

	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = dma_count;
	DMA1_Channel1->CCR = DMA_CCR_MINC |
						 DMA_CCR_MSIZE_0 |
						 DMA_CCR_PSIZE_0 |
						 DMA_CCR_TCIE |
						 DMA_CCR_HTIE;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	ADC1->CFGR &= ~ADC_CFGR_CONT;
	ADC1->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_OVRMOD;

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));
	ADC1->CR |= ADC_CR_ADSTART;
}

void adc_start_cycle(uint16_t n_per_cycle) {
	stop_adc_dma();
	uint16_t dma_count = n_per_cycle * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;

	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = dma_count;
	DMA1_Channel1->CCR = DMA_CCR_MINC |
						 DMA_CCR_CIRC |
						 DMA_CCR_TCIE |
						 DMA_CCR_HTIE |
						 DMA_CCR_MSIZE_0 |
						 DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	ADC1->CFGR |= ADC_CFGR_DMAEN | ADC_CFGR_CONT | ADC_CFGR_OVRMOD;

	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));
	ADC1->CR |= ADC_CR_ADSTART;
}

// TIM

#define SET_AF(num_pin, num_af) (num_af << (num_pin * 4))

void init_tim1_as_pwm() { // pa8 pa9
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOA->AFR[1] |= SET_AF(8 - 8, 0x6) | SET_AF(9 - 8, 0x6);

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->ARR = 999;
	TIM1->PSC = 31;

	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // pwm mode 1
	TIM1->CCMR1 |= TIM_CCMR1_OC1CE;

	TIM1->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // pwm mode 1
	TIM1->CCMR1 |= TIM_CCMR1_OC2CE;

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM1->CCR1 = 500;
	TIM1->CCR2 = 200;
	TIM1->EGR |= TIM_EGR_UG;

	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR1 |= TIM_CR1_CEN;
}

void init_tim2_as_pwm() { // pb10 pb11
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->AFR[1] |= SET_AF(10 - 8, 0x1) | SET_AF(11 - 8, 0x1);

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->ARR = 999;
	TIM2->PSC = 31;

	TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // preload

	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE; // preload

	TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CCR3 = 500;
	TIM2->CCR4 = 200;
	TIM2->EGR |= TIM_EGR_UG;

	TIM2->CR1 |= TIM_CR1_CEN;
}

// need to init tim3 tim4

// Button

void init_pa5() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_0; // output
}

void EXTI15_10_IRQHandler() {
	if (EXTI->PR & EXTI_PR_PR13) {
		EXTI->PR |= EXTI_PR_PR13;
		click_processed = 0;
	}
}

void init_button_pc13() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODER13;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_0; // pull up resistor

	EXTI->IMR |= EXTI_IMR_MR13;
	EXTI->RTSR |= EXTI_FTSR_TR13;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13);
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	NVIC_SetPriority(EXTI15_10_IRQn, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void adc_start() {
	uint8_t byte;
	get_from_tail(&byte);
	uint8_t count = byte & 0x0F;
	uint8_t channels_raw = byte >> 4;
	uint8_t channels[count];
	uint8_t counter = 0;
	for (uint8_t i = 0; i < 4; i++) {
		if ((channels_raw >> i) & 1) {
			channels[counter] = i;
			counter++;
		}
	}
	adc_set_pc_channels(channels, count);

	get_from_tail(&byte);
	uint8_t speed = byte << 4;
	uint8_t mode = byte & 0x0F;
	adc_set_sampling_time(speed);

	get_from_tail(&byte);
	uint16_t n_samples = 0;
	n_samples |= byte << 8;
	get_from_tail(&byte);
	n_samples |= byte;
	if (mode == 0) adc_start_cycle(n_samples);
	if (mode == 1) adc_start_burst(n_samples);
}

void btn_process() {
	if (GPIOC->IDR & GPIO_IDR_13 || click_processed) return;
	for (uint8_t i = 0; i < 25; i++);
	if (GPIOC->IDR & GPIO_IDR_13) return;
	click_processed = 1;
	switch (cmd) {
		case 0x00:
			adc_start();
			break;
		case 0x01:
			break;
	}
}



int main(void)
{
	init_pll_usart();
	init_usart_dma_tx();
	init_pa5();
	init_button_pc13();
	while(1) {
		data_convert();

		if (cmd_received) process_command();
		cmd_received = 0;

		btn_process();
	}
	return 0;
}
