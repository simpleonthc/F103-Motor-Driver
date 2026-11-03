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
ML_Status ML_setVelocity(ML_Master *self, int16_t leftVelocity, int16_t rightVelocity) {
//	const size_t sof_len = 2;
//	const size_t frame_len = 5;
//	uint8_t *send_buffer = self->uart.tx_buffer;
//	tx_buffer[0] = SOF1;
//	tx_buffer[1] = SOF2;
//	tx_buffer[2] = ML_SET_VELOCITY;
//	tx_buffer[3] = (uint8_t)(leftVelocity & 0xFF);
//	tx_buffer[4] = (uint8_t)(leftVelocity >> 8); ;
//	tx_buffer[5] = (uint8_t)(rightVelocity & 0xFF);
//	tx_buffer[6] = (uint8_t)(rightVelocity >> 8);

//	size_t out_size = encodeCOBS(inptr, inputlen, outptr, outlen, add_trailing_zero)

//	ML_UARTWrite(self->uart.huart, &self->uart.tx_flag, self->uart.tx_buffer, frame_len + sof_len);
	return ML_OK;
}

ML_Status ML_decodeTelemetry(ML_Master *self) {
	size_t telemetry_buf_len = 7;
	uint8_t telemetry_buf[7] = {0x01, 0x01, 0x02, 0x04, 0x02, 0xFC, 0x00};
	//ML_UARTRead(telemetry_buf, telemetry_buf_len);

	size_t decoded_size = decodeCOBS_inplace(telemetry_buf, telemetry_buf_len);
	if (decoded_size == 0) {
		return ML_FAULT;
	}
	//self->telemetry.ML_Left_Velocity
	int16_t output = (int16_t)((telemetry_buf[2] << 8) | telemetry_buf[1]);
	return ML_OK;
}
/*
 * Functions for Motor controller side
 * */

ML_Status ML_sendTelemetry(ML_Driver *self,
							int16_t leftVelocity,
							int16_t rightVelocity,
							uint8_t VBAT, uint8_t FAULT, uint8_t STATUS) {
	size_t databuf_len = 7;
	uint8_t databuf[databuf_len];
	uint8_t encodedbuf[databuf_len + 1];

	databuf[0] = (uint8_t)(leftVelocity & 0xFF);
	databuf[1] = (uint8_t)(leftVelocity >> 8); ;
	databuf[2] = (uint8_t)(rightVelocity & 0xFF);
	databuf[3] = (uint8_t)(rightVelocity >> 8);
	databuf[4] = VBAT;
	databuf[5] = FAULT;
	databuf[6] = STATUS;

	size_t encodedSize = encodeCOBS(databuf,
									databuf_len,
									encodedbuf,
									databuf_len + 1,
									true);

	return ML_UARTWrite(encodedbuf, encodedSize);
}
