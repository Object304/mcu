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
//0x00 - adc off
//0x01 - adc on
//0	x	0 0				0 0 0 0
//		0 - circular	amount
//		1 - burst
//
//01 01 00 01 - burst 1 sample, 01 01 02 00 - 512, 01 01 01 00 - 256, 01 01 00 80 - 128
//01 00 00 01 - circular 1 sample
//
//0x02 - set channels
//0	x	0				0
//		0	0	0	0
//		pc0	pc1	pc2	pc3	count
//		1 - on			1..4
//		0 - off
//02 81
//
//0x03 - set frequency
//0	x	0 0 0 0 0 0 0 0
//		frequency
//03 00 00 00 01 - 1 Hz, 03 00 00 03 E8 - 1 kHz
//
//0x04 - set size
//0	x	00 - 12 bit
//		01 - 10 bit
//		02 - 8 bit
//		03 - 6 bit
//04 00
//
//0x05 - set interval ms
//0	x	00 - if available
//		01	0 0 0 0 - set interval
//			interval
//05 00 - available, 05 01 03 E8 - 1 sec
//
//Big data
//
//0x06 - big data transmit started
//0 x	00 00
//		data size
//
//06 00 05
//
//0x07 - big data transmit ended
//
//Temp
//
//0x08 - send temp
//
//PWM
//
//0x09 - pwm on
//0x0A - pwm off
//0x0B - set frequency, data:
//0	x	0			0			0 0 0 0 0 0 0 0
//		skip		1 - tim1	frequency
//					2 - tim2
//					3 - tim3
//
//03 01 00 00 20 00 - 8192		03 02 00 00 30 00 - 12288		03 03 00 00 40 00 - 16384 Hz
//
//0x0C - set duty %, data:
//0	x	0			0		0 0
//		1 - tim1	1 - ch1	duty
//		2 - tim2	2 - ch2
//		3 - tim3	3 - ch3
//					4 - ch4
//
//04 11 50		04 23 50		04 31 50

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
			adc_off();
			break;
		case 0x01:
			adc_on();
			break;
		case 0x02:
			adc_set_channels();
			break;
		case 0x03:
			adc_set_freq();
			break;
		case 0x04:
			adc_set_size();
			break;
		case 0x05:
			set_interval();
			break;
		case 0x08:
			adc_read_temp();
			break;
		case 0x09:
			tim_on();
			break;
		case 0x0A:
			tim_off();
			break;
		case 0x0B:
			set_pwm_freq_raw();
			break;
		case 0x0C:
			set_pwm_duty();
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
	init_buffer(&rx_buf);
	init_buffer(&command_buf);
	init_buffer(&command_data_buf);
	init_adc_dma();
	while(1) {
		if (mode_type == 0) {
			data_convert();
		}
		process_command();
		btn_process();
	}
	return 0;
}
