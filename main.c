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
//0x02 - set channels
//0	x	0				0
//		0	0	0	0
//		pc0	pc1	pc2	pc3	count
//		1 - on			1..4
//		0 - off
//0x03 - set frequency
//0	x	0 0 0 0 0 0 0 0
//		frequency
//0x04 - set size
//0	x	00 - 12 bit
//		01 - 10 bit
//		02 - 8 bit
//		03 - 6 bit
//0x05 - set interval
//0	x	00 - if available
//		01	0 0 0 0 0 0 0 0 - set interval
//			interval

#include "stm32f3xx.h"
#include "ADC.h"
#include "globals.h"
#include "RingBuffer.h"
#include "USART.h"

void init_pa5() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_0; // output
}

void command_execute() {
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
	}
	command_data_buf.tail = rx_data_tail_temp;
}

int main(void)
{
	init_pll_usart();
	init_usart_dma_tx();
	init_pa5();
	init_buffer(&rx_buf);
	init_buffer(&command_buf);
	init_buffer(&command_data_buf);
	init_adc_dma();
	while(1) {
		data_convert();
		process_command();
		if (command_ready) {
			command_execute();
			command_ready = 0;
		}
	}
	return 0;
}
