/*
 * adc.c
 *
 *  Created on: May 31, 2023
 *      Author: <SKG> POTOP
 */

#include "../../../19_DISCO_SomeDrivers/Core/Inc/adc.h"

void adc_init(void)
{
	RCC->AHB1ENR |= (1U<<0);		// shift '1' to bit 0 // Enable clock for GPIOA
	RCC->APB2ENR |= (1U<<8);		// Enable clock for ADC1
	GPIOA->MODER |= (3U<<2);		// or 0xC

	ADC1->CR2 = 0;					// software trigger
	ADC1->SQR3 = 1;					// conversion sequence starts at ch 1
	ADC1->SQR1 = 0;					// conversion sequence length 1
	ADC1->CR2 |= 1;					// enable ADC1
}

uint32_t read_analog_sensor(void)
{
	ADC1->CR2 |= (1U<<30);			// Start conversion

	while((ADC1->SR & 2)){}			// Wait for conversion to complete
	return ADC1->DR;				// Return the resultat
}
