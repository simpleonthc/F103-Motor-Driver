/*
 * motor_link.c
 *
 *  Created on: Oct 26, 2026
 *      Author: pk
 */

#include "motor_link_uart.h"


ML_Status ML_UARTWrite(UART_HandleTypeDef *huart, uint8_t *isSent, uint8_t *pData, size_t data_len) {
	if (*isSent == 1) {
		HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(huart, pData, data_len);
		*isSent = 0;
		if (result != HAL_OK) {
			return ML_TX_ERROR;
		}


	}


	return ML_OK;
}

ML_Status ML_UARTRead(UART_HandleTypeDef *huart, uint8_t *pData, size_t data_len) {
	HAL_StatusTypeDef result = HAL_UART_Receive_DMA(huart, pData, data_len);
	if (result != HAL_OK) {
		return ML_TX_ERROR;
	}
	return ML_OK;
}

