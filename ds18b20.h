/*
 * ds18b20.h
 *
 *  Created on: 26 апр. 2025 г.
 *      Author: Zhon1
 */

#include "stm32f3xx.h"

#ifndef DS18B20_H_
#define DS18B20_H_

void delay(int count);
uint8_t init_sensor();
void start_convert();
void wait_convert();
float read_data();

#endif /* DS18B20_H_ */
