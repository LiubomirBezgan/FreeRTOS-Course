/*
 * exti.h
 *
 *  Created on: May 31, 2023
 *      Author: <SKG> POTOP
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f4xx_hal.h"

void pa0_interrupt_init(void);
void gpio_init(void);
uint8_t read_digital_sensor(void);

#endif /* INC_EXTI_H_ */
