/*
 * Motor_Link.c
 *
 *  Created on: Oct 26, 2026
 *      Author: pk
 */
#include "MOTOR_Link.h"
/*
 * Functions for Main controller side
 * */
ML_Status ML_sendVelocity(ML_MotorLink *self, int16_t leftVelocity, int16_t rightVelocity) {
	const size_t sof_len = 2;
	const size_t frame_len = 5;
	uint8_t *tx_buffer = self->uart.tx_buffer;
	tx_buffer[0] = SOF1;
	tx_buffer[1] = SOF2;
	tx_buffer[2] = ML_SET_VELOCITY;
	tx_buffer[3] = (uint8_t)(leftVelocity & 0xFF);
	tx_buffer[4] = (uint8_t)(leftVelocity >> 8); ;
	tx_buffer[5] = (uint8_t)(rightVelocity & 0xFF);
	tx_buffer[6] = (uint8_t)(rightVelocity >> 8);

	ML_UARTWrite(self->uart.huart, &self->uart.tx_flag, self->uart.tx_buffer, frame_len + sof_len);
	return ML_OK;
}

/*
 * Functions for Motor controller side
 * */
ML_Status ML_Receive(ML_MotorLink *self) {
	ML_UARTRead(self->uart.huart, self->uart.rx_buffer, ML_TX_BUFFER_SIZE);
	uint8_t *rx_buffer = self->uart.rx_buffer;
	bool valid_frame = false;

	for(int i= 0; i < (ML_RX_BUFFER_SIZE - ML_TX_BUFFER_SIZE); i++) {
		if (rx_buffer[i] == SOF1 && rx_buffer[i+1] == SOF2) {
			valid_frame = true;
		}
	}
	if (valid_frame == false) {
		return ML_FAULT;
	}
	valid_frame = !valid_frame;
	switch(rx_buffer[2]){
	case ML_SET_VELOCITY:

	default:
		return ML_OK;
	}
	return ML_OK;
}

ML_Status ML_ReceiveVelocity(ML_MotorLink *self) {
	self->command
	return ML_OK;
}
