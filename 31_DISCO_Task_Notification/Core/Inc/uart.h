/*
 * uart.h
 *
 *  Created on: May 31, 2023
 *      Author: <SKG> POTOP
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"

///**
//  * @brief USART1 Transmission Initialization Function
//  * @param None
//  * @retval None
//  */
//void USART1_UART_Tx_Init(void);
//
///**
//  * @brief USART1 Receiving Initialization Function
//  * @param None
//  * @retval None
//  */
//void USART1_UART_Rx_Init(void);

/**
  * @brief  USART1 writing function
  * @param  ch describes a sending character
  * @retval ch describes the sending character
  */
int uart1_write(int ch);

/**
  * @brief  Sending function which use USART1 instead of a console
  * @param  ch describes a sending character
  * @retval ch describes the sending character
  */
int __io_putchar(int ch);

#endif /* INC_UART_H_ */
