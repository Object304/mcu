//1 - sync
//2 - weight (0x00 - small, 0x01 - big)
//3 - cmd
//4 - time1
//5 - time2
//6 - data0
//7 - data1
//8 - data2
//9 - data3
//10 - xor
//11 - end (0xC0)
//
//0x00:
//0xAA00001212349EC0


#include "stm32f3xx.h"

uint16_t dma_data[256];
uint8_t usart_tx_buffer[256]; // 128 значений по 2 байта
uint8_t data_ready = 0;

#define RX_BUF_SIZE 512
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

volatile uint8_t rx_data_buf[RX_BUF_SIZE];
volatile uint16_t rx_data_head = 0;
volatile uint16_t rx_data_tail = 0;

volatile uint8_t cmd_received = 0;

void adc_start_once() {
	ADC1->CR |= ADC_CR_ADSTART;
}

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
	uint8_t cmd;
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
	switch (cmd) {
		case 0x00:
//			adc_start_once();
			get_from_tail(&byte);
			uint16_t data = byte << 8;
			get_from_tail(&byte);
			data |= byte;
			break;
	}
	return 0;
}

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
		prepare_usart_tx_buffer(0, 128);
		usart_dma_send(128 * 2);
		data_ready = 0;
	}
	if (data_ready == 2) {
		prepare_usart_tx_buffer(128, 128);
		usart_dma_send(128 * 2);
		data_ready = 0;
	}
}

void DMA1_Channel7_IRQHandler() {
	if (DMA1->ISR & DMA_ISR_TCIF7) {
		DMA1->IFCR |= DMA_IFCR_CTCIF7; // сброс флага
	}
}

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

void USART2_IRQHandler() {
    if (USART2->ISR & USART_ISR_RXNE) {
    	rx_buf[rx_head] = USART2->RDR;

    	if (rx_buf[rx_head] == 0xC0) cmd_received = 1;

		rx_head = (rx_head + 1) % RX_BUF_SIZE;
    }
}

void init_adc_dma_usart() {
	//init gpio pa1
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER1;	//pa1 as analog adc1_in2

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

	//init adc12
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	//enable power
	ADC1->CR &= ~ADC_CR_ADVREGEN_1;
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	ADC1->CFGR |= ADC_CFGR_OVRMOD;
	ADC1->CFGR |= ADC_CFGR_CONT;
	ADC1->SQR1 &= ~(1 << 0); //only one channel (L -> 0000 - 1 conversion)
	ADC1->SQR1 |= 2 << 6; //2nd channel

	ADC1->SMPR1 &= ~ADC_SMPR1_SMP2;
	ADC1->SMPR1 |= ADC_SMPR1_SMP2_2 | ADC_SMPR1_SMP2_1 | ADC_SMPR1_SMP2_0;

	ADC1->CFGR |= ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;	//enable DMA

	//init dma
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel1->CMAR = (uint32_t)dma_data; // записываем адрес массива в который записываем результат преобразования
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR; // записываем адрес данных ацп из которых будем считывать
	DMA1_Channel1->CNDTR = 256; // количсетво элементов которые мы записываем
	DMA1_Channel1->CCR |= DMA_CCR_CIRC; // непрерывное преобразование
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; // размер одного элемента
	DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE; // указывается двойная буферизация
	DMA1_Channel1->CCR |= DMA_CCR_MINC; // инкрементируем память

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA1_Channel1->CCR |= DMA_CCR_EN;

	//enable adc
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRD));
//	ADC1->CR |= ADC_CR_ADSTART;

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

void init_pa5() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_0; // output
}

int main(void)
{
	init_adc_dma_usart();
	init_usart_dma_tx();

	init_pa5();

	while(1) {
		data_convert();
		if (cmd_received) process_command();
		cmd_received = 0;
	}
	return 0;
}
