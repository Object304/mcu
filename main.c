//1 - sync
//2 - cmd
//3 - time1 (min)
//4 - time2 (sec)
//5 - data1
//6 - data2
//7 - data3
//8 - data4
//9 - data5
//10 - xor
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
//00 81 71 00 01 - burst 1 sample, 00 81 71 02 00 - 512, 00 81 71 01 00 - 256, 00 81 71 00 80 - 128
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
//03 01 00 00 20 00 - 8192		03 02 00 00 30 00 - 12288		03 03 00 00 40 00 - 16384 Hz
//
//0x04 - set duty %, data:
//0	x	0			0		0 0
//		1 - tim1	1 - ch1	duty
//		2 - tim2	2 - ch2
//		3 - tim3	3 - ch3
//					4 - ch4
//
//04 11 50		04 23 50		04 31 50
//
//Temp
//
//0x05 - send temp

#include "stm32f3xx.h"
#include "ADC.h"
#include "globals.h"
#include "RingBuffer.h"
#include "USART.h"
#include "TIM.h"

volatile uint8_t click_processed = 0;

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

void btn_process() {
	if (GPIOC->IDR & GPIO_IDR_13 || click_processed) return;
	for (uint8_t i = 0; i < 30; i++);
	if (GPIOC->IDR & GPIO_IDR_13) return;

	click_processed = 1;
	uint16_t rx_data_tail_temp = command_data_buf.tail;
	switch (cmd) {
		case 0x00:
			adc_start();
			break;
		case 0x01:
			tim_on();
			break;
		case 0x02:
			tim_off();
			break;
		case 0x03:
			set_pwm_freq_raw();
			break;
		case 0x04:
			set_pwm_duty();
			break;
		case 0x05:
			adc_read_temp();
			break;
	}
	command_data_buf.tail = rx_data_tail_temp;
}

int main(void)
{
	init_pll_usart();
	init_usart_dma_tx();
	init_pa5();
	init_button_pc13();
	init_tim1_as_pwm();
	init_tim2_as_pwm();
	init_tim3_as_pwm();
	init_buffer(&rx_buf);
	init_buffer(&command_buf);
	init_buffer(&command_data_buf);
	init_adc_dma();
	while(1) {
		data_convert();
		process_command();
		btn_process();

	}
	return 0;
}
