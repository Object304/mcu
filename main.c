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
//
//ADC
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
//
//00 81 71 00 01 - 1 sample, 00 81 71 02 00 - 512, 00 81 71 01 00 - 256, 00 81 71 00 80 - 128
//00 F4 71 00 01 - 4 channels 1 sample burst
//00 81 70 00 01 - circular 1 sample, 00 F4 70 00 01 - circular 1 sample 4 channels
//
//PWM
//
//0x01 - pwm on
//0x02 - pwm off
//0x03 - set frequency, data:
//0	x	0			0			0 0 0 0 0 0 0 0
//		skip		1 - tim1	frequency
//					2 - tim2
//					3 - tim3
//
//03 01 00 00 20 00		03 02 00 00 30 00		03 03 00 00 40 00 - 16384 Hz
//
//0x04 - set duty %, data:
//0	x	0			0		0 0
//		1 - tim1	1 - ch1	duty
//		2 - tim2	2 - ch2
//		3 - tim3	3 - ch3
//					4 - ch4
//
//04 11 50		04 23 50	04 31 50
//
//Temp
//
//0x05 - send temp

#include "stm32f3xx.h"

#define MAX_ADC_CHANNELS 4

uint8_t active_channels[MAX_ADC_CHANNELS];
uint8_t num_channels = 0;
volatile uint16_t samples_count = 0;

#define ADC_BUF_SIZE 512
volatile uint16_t dma_data[ADC_BUF_SIZE];
volatile uint8_t usart_tx_buffer[ADC_BUF_SIZE * 2];
volatile uint8_t data_ready = 0;
volatile uint8_t dma_tx_busy = 0;

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
	*byte = rx_data_buf[rx_data_head - 1];
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
	rx_data_head = rx_data_tail = 0;
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
	GPIOA->ODR |= GPIO_ODR_5;
	for (uint16_t i = 0; i < length; ++i) {
		uint16_t value = dma_data[start + i];
		usart_tx_buffer[2*i]     = value & 0xFF;        // LSB (LSB first)
		usart_tx_buffer[2*i + 1] = (value >> 8) & 0xFF; // MSB
	}
}

void usart_dma_send(uint16_t length) {

	while(dma_tx_busy);
	dma_tx_busy = 1;

	DMA1_Channel7->CCR &= ~DMA_CCR_EN; // остановка
	DMA1_Channel7->CNDTR = length;     // сколько байт передать
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
		dma_tx_busy = 0;
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
	// Остановка АЦП
	if (ADC1->CR & ADC_CR_ADEN) {
		ADC1->CR |= ADC_CR_ADDIS;                  // запросить отключение
		while (ADC1->CR & ADC_CR_ADEN);            // подождать, пока отключится
	}

	// Остановка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
}


void adc_start_burst(uint16_t n_total_samples) {
	stop_adc_dma();
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
	stop_adc_dma();
	uint16_t dma_count = n_per_cycle * num_channels;
	if (dma_count > ADC_BUF_SIZE) dma_count = ADC_BUF_SIZE;

	samples_count = dma_count;

	// Настройка DMA
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = samples_count;
	DMA1_Channel1->CCR = DMA_CCR_MINC |
						 DMA_CCR_CIRC |
						 DMA_CCR_TCIE |
						 DMA_CCR_MSIZE_0 |
						 DMA_CCR_PSIZE_0;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка ADC
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;
	ADC1->CFGR |= ADC_CFGR_DMAEN |
				  ADC_CFGR_CONT |
				  ADC_CFGR_OVRMOD |
				  ADC_CFGR_DMACFG;

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

void init_tim3_as_pwm() { // PA6 (CH1), PA7 (CH2)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;     // Alternate function mode
	GPIOA->AFR[0] |= SET_AF(6, 0x2) | SET_AF(7, 0x2);               // AF2 для TIM3

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->PSC = 31;     // Предделитель
	TIM3->ARR = 999;    // Автоперезагрузка

	// Настройка CH1
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;                     // preload

	// Настройка CH2
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	TIM3->CCR1 = 500;  // 50% скважность
	TIM3->CCR2 = 200;

	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->EGR |= TIM_EGR_UG;  // Обновить регистры
	TIM3->CR1 |= TIM_CR1_CEN; // Включить таймер
}

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
			channels[counter] = 3 - i;
			counter++;
		}
	}
	adc_set_pc_channels(channels, count);

	get_from_tail(&byte);
	uint8_t speed = byte >> 4;
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

void set_pwm_freq(uint8_t timer_number, uint32_t freq_hz) {
	if (freq_hz == 0 || freq_hz > SystemCoreClock) return;  // Защита от недопустимых значений
	TIM_TypeDef* tim;
	switch (timer_number) {
		case 1: tim = TIM1; break;
		case 2: tim = TIM2; break;
		case 3: tim = TIM3; break;
		default: return;  // Неподдерживаемый номер таймера
	}
	uint32_t timer_clk = SystemCoreClock;
	// Перебираем подходящие значения предделителя и авто-перезагрузки
	uint32_t psc = 0;
	uint32_t arr = 0;
	for (psc = 0; psc < 0xFFFF; psc++) {
		arr = (timer_clk / (freq_hz * (psc + 1))) - 1;
		if (arr <= 0xFFFF) {
			break;
		}
	}
	if (arr > 0xFFFF) return;  // Не удалось подобрать значения
	tim->PSC = psc;
	tim->ARR = arr;
	tim->EGR |= TIM_EGR_UG;  // Обновление регистров
}

void set_pwm_freq_raw() {
	uint8_t byte;
	get_from_tail(&byte);
	uint8_t tim_num = byte;
	uint32_t freq = 0;
	for (uint8_t i = 4; i > 0; i--) {
		get_from_tail(&byte);
		freq |= byte << 8 * (i - 1);
	}
	set_pwm_freq(tim_num, freq);
}

void set_pwm_duty() {
	uint8_t byte;
	get_from_tail(&byte);
	uint8_t tim_num = byte >> 4;
	uint8_t ch_num = byte & 0x0F;
	get_from_tail(&byte);
	uint8_t duty = byte;
	TIM_TypeDef* tim;
	switch (tim_num) {
		case 1: tim = TIM1; break;
		case 2: tim = TIM2; break;
		case 3: tim = TIM3; break;
		default: return;  // Неподдерживаемый номер таймера
	}
	if (duty > 100) duty = 100;
	uint32_t value = ((uint32_t)(tim->ARR + 1) * duty) / 100;
	switch (ch_num) {
		case 1: tim->CCR1 = value; break;
		case 2: tim->CCR2 = value; break;
		case 3: tim->CCR3 = value; break;
		case 4: tim->CCR4 = value; break;
		default: break;
	}
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

void btn_process() {
	if (GPIOC->IDR & GPIO_IDR_13 || click_processed) return;
	for (uint8_t i = 0; i < 30; i++);
	if (GPIOC->IDR & GPIO_IDR_13) return;

	click_processed = 1;
	uint16_t rx_data_tail_temp = rx_data_tail;
	switch (cmd) {
		case 0x00:
			adc_start();
			break;
		case 0x01:
			TIM1->CCER |= TIM_CCER_CC1E;
			TIM1->CCER |= TIM_CCER_CC2E;
			TIM2->CCER |= TIM_CCER_CC3E;
			TIM2->CCER |= TIM_CCER_CC4E;
			TIM3->CCER |= TIM_CCER_CC1E;
			TIM3->CCER |= TIM_CCER_CC2E;
			break;
		case 0x02:
			TIM1->CCER &= ~TIM_CCER_CC1E;
			TIM1->CCER &= ~TIM_CCER_CC2E;
			TIM2->CCER &= ~TIM_CCER_CC3E;
			TIM2->CCER &= ~TIM_CCER_CC4E;
			TIM3->CCER &= ~TIM_CCER_CC1E;
			TIM3->CCER &= ~TIM_CCER_CC2E;
			break;
		case 0x03:
			set_pwm_freq_raw();
			break;
		case 0x04:
			set_pwm_duty();
			break;
		case 0x05:
			adc1_read_temp();
			break;
	}
	rx_data_tail = rx_data_tail_temp;
}

void adc1_init_temp_sensor() {
	// Включить тактирование ADC и внутреннего датчика
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= ADC12_CCR_TSEN; // Температурный датчик включен

	// Отключить ADC1 перед калибровкой
	ADC1->CR &= ~ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADCAL; // Запуск калибровки
	while (ADC1->CR & ADC_CR_ADCAL); // Ждать завершения

	// Включить ADC1
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD)); // Ждать готовности

	// Настроить канал 16 (температурный датчик)
	ADC1->SQR1 = (16 << 6); // Канал 16 в первый слот

	// Максимальное время выборки для канала 16
	ADC1->SMPR1 |= (7 << 18);

	ADC1->CFGR &= ~ADC_CFGR_CONT; // Одиночное преобразование
}

int main(void)
{
	init_pll_usart();
	init_usart_dma_tx();
	init_pa5();
	init_button_pc13();
	adc1_init_temp_sensor();
	init_tim1_as_pwm();
	init_tim2_as_pwm();
	init_tim3_as_pwm();
	while(1) {
		data_convert();

		if (cmd_received) process_command();
		cmd_received = 0;

		btn_process();
	}
	return 0;
}
