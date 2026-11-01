/*
 * motor_link_uart.h
 *
 *  Created on: Oct 29, 2026
 *      Author: pk
 */

#ifndef MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_
#define MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_

#include <stdint.h>
#include <stdio.h>
#include <stm32f1xx_hal.h>
#include "MOTOR_Link_Types.h"
ML_Status ML_UARTWrite(UART_HandleTypeDef *huart, uint8_t *isSent, uint8_t *pData, size_t data_len);
ML_Status ML_UARTRead(UART_HandleTypeDef *huart, uint8_t *pData, size_t data_len);

#endif /* MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_ */
