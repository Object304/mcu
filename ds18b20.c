/*
 * ds18b20.c
 *
 *  Created on: 26 апр. 2025 г.
 *      Author: Zhon1
 */

#include "ds18b20.h"

void delay(int count) {
	for (int i = 0; i < count; i++);
}

void delay_480us() {
	delay(385);
}

void delay_172us() {
	delay(133);
}

void delay_88us() {
	delay(66);
}

void delay_7us() {
	delay(1);
}

void delay_15us() {
	delay(7);
}

void delay_20us() {
	delay(11);
}

void delay_824ms() {
	delay(601562);
}

void write_bit(uint8_t bit) {
	GPIOC->ODR &= ~GPIO_ODR_8;
	if (bit) {
		delay_7us();
		GPIOC->ODR |= GPIO_ODR_8;
		delay_88us();
	}
	else {
		delay_88us();
		GPIOC->ODR |= GPIO_ODR_8;
	}
}

void write_byte(uint8_t byte) {
	GPIOC->MODER &= ~GPIO_MODER_MODER8;
	GPIOC->MODER |= GPIO_MODER_MODER8_0; //output
	for (int i = 0; i < 8; i++) {
		write_bit(byte & 0x01);
		byte >>= 1;
	}
}

uint8_t init_sensor() {
	GPIOC->MODER &= ~GPIO_MODER_MODER8;
	GPIOC->MODER |= GPIO_MODER_MODER8_0; //output
	GPIOC->ODR &= ~GPIO_ODR_8;
	delay_480us();
	GPIOC->MODER &= ~GPIO_MODER_MODER8; //input
	delay_88us();
	uint8_t state = (GPIOC->IDR & GPIO_IDR_8) ? 0 : 1; // 0 -- no connection
	delay_480us();
	return state;
}

void start_convert() {
	init_sensor();
	write_byte(0xCC);
	write_byte(0x44);
}

void wait_convert() {
	GPIOC->MODER &= ~GPIO_MODER_MODER8; //input
	while (!(GPIOC->IDR & GPIO_IDR_8));
}

float read_data() {
	init_sensor();
	write_byte(0xCC);
	write_byte(0xBE);
	delay_20us();
	uint16_t data = 0x0000;
	for (int i = 0; i < 16; i++) {
		GPIOC->MODER &= ~GPIO_MODER_MODER8;
		GPIOC->MODER |= GPIO_MODER_MODER8_0; //output
		GPIOC->ODR &= ~GPIO_ODR_8;
		delay_7us();
		GPIOC->MODER &= ~GPIO_MODER_MODER8; //input
		delay_15us();
		data |= ((GPIOC->IDR & GPIO_IDR_8) ? 1 : 0) << i;
		delay_20us();
		delay_20us();
	}
	int16_t temp_raw = (int16_t)data;
	float temp = temp_raw * 0.0625f;
	return temp;
}
