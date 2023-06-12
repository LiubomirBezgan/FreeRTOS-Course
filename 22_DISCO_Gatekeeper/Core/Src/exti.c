/*
 * exti.c
 *
 *  Created on: May 31, 2023
 *      Author: <SKG> POTOP
 */

#include "exti.h"

void pa0_interrupt_init(void)		// PA0
{
	RCC->AHB1ENR |= 1;				// Enable GPIOA clock

	RCC->APB2ENR |= 0x4000;			// Enable SYSCFG clock

	GPIOA->MODER &= ~0x3;			// Configure PA0 for push button interrupt

	SYSCFG->EXTICR[3] &= ~0x00F0;	// Clear port selection for EXTI13 and select port A

	EXTI->IMR |= 0x2000;			// unmask EXTI13

	EXTI->FTSR |= 0x2000;			// select falling edge trigger

	NVIC_SetPriority(EXTI15_10_IRQn, 6);

	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void gpio_init(void)
{
	RCC->AHB1ENR |= 1;				// Enable GPIOA clock
	GPIOA->MODER &= ~0x3;			// Configure PA0 for push button interrupt
}

uint8_t read_digital_sensor(void)
{
	if (GPIOA->IDR & 0x1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
