/*
 * TIM.h
 *
 *  Created on: 24 мая 2025 г.
 *      Author: Zhon1
 */

#ifndef TIM_H_
#define TIM_H_

#include "stm32f3xx.h"
#include "RingBuffer.h"
#include "globals.h"

void init_tim1_as_pwm();
void init_tim2_as_pwm();
void set_pwm_freq(uint8_t timer_number, uint32_t freq_hz);
void set_pwm_freq_raw();
void set_pwm_duty();
void tim_on();
void tim_off();

#endif /* TIM_H_ */
