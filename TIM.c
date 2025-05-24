/*
 * TIM.c
 *
 *  Created on: 24 мая 2025 г.
 *      Author: Zhon1
 */

#include "TIM.h"

#define SET_AF(num_pin, num_af) (num_af << (num_pin * 4))

void init_tim1_as_pwm() { // pa8 pa9
	GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
	GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);

	GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4));
	GPIOA->AFR[1] |= (0x6 << 0) | (0x6 << 4);

	RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->PSC = 31;
	TIM1->ARR = 999;

	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1CE);
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);

	TIM1->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2CE);
	TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	TIM1->CCR1 = 500;
	TIM1->CCR2 = 200;

	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM1->EGR |= TIM_EGR_UG;

	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR1 |= TIM_CR1_CEN;

}

void init_tim2_as_pwm() { // pb10 pb11
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

	GPIOB->AFR[1] &= ~((0xF << 8) | (0xF << 12));
	GPIOB->AFR[1] |= (0x1 << 8) | (0x1 << 12);

	TIM2->PSC = 31;
	TIM2->ARR = 999;

	uint32_t ccmr2 = TIM2->CCMR2;
	ccmr2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE | TIM_CCMR2_CC4S | TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE);
	ccmr2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE);
	ccmr2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE);
	TIM2->CCMR2 = ccmr2;

	TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM2->CCR3 = 500;
	TIM2->CCR4 = 200;

	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->EGR |= TIM_EGR_UG;

	TIM2->CR1 |= TIM_CR1_CEN;
}

void init_tim3_as_pwm() { // PA6 (CH1), PA7 (CH2)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->AFR[0] |= SET_AF(6, 0x2) | SET_AF(7, 0x2);

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->PSC = 31;
	TIM3->ARR = 999;

	// Настройка CH1
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

	// Настройка CH2
	TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	TIM3->CCR1 = 500;
	TIM3->CCR2 = 200;

	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
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
	get_from_tail(&byte, &command_data_buf);
	uint8_t tim_num = byte;
	uint32_t freq = 0;
	for (uint8_t i = 4; i > 0; i--) {
		get_from_tail(&byte, &command_data_buf);
		freq |= byte << 8 * (i - 1);
	}
	set_pwm_freq(tim_num, freq);
}

void set_pwm_duty() {
	uint8_t byte;
	get_from_tail(&byte, &command_data_buf);
	uint8_t tim_num = byte >> 4;
	uint8_t ch_num = byte & 0x0F;
	get_from_tail(&byte, &command_data_buf);
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

void tim_on() {
	TIM1->CCER |= TIM_CCER_CC1E;
	TIM1->CCER |= TIM_CCER_CC2E;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
}

void tim_off() {
	TIM1->CCER &= ~TIM_CCER_CC1E;
	TIM1->CCER &= ~TIM_CCER_CC2E;
	TIM2->CCER &= ~TIM_CCER_CC3E;
	TIM2->CCER &= ~TIM_CCER_CC4E;
	TIM3->CCER &= ~TIM_CCER_CC1E;
	TIM3->CCER &= ~TIM_CCER_CC2E;
}
