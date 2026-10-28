/*
 * Motor_Link.c
 *
 *  Created on: Oct 26, 2026
 *      Author: pk
 */
#include "MOTOR_Link.h"

//ML_Status ML_Serialize( uint8_t    *buffer,
//						ML_Payload *payload,
//						size_t      payload_len)
//{
//	for (int i = 0; i < payload_len; i++) {
//			buffer[i + 1] = (uint8_t)(payload->data->ML_Acceleration >> (i * 8));
//		}
//}

ML_Status ML_EncodeCommand( uint8_t    *buffer,
							ML_Payload *payload,
							size_t      buffer_len,
							size_t     *encoded_len )
{
	if (buffer == NULL || payload == NULL || encoded_len == NULL) {
		return ML_INVALID_PARAM;
	}

	if (buffer_len < 1U) {
		return ML_FAULT;
	}

	buffer[0] = payload->ML_CommandID;
	size_t payload_len;
	switch(payload->ML_CommandID) {
		case ML_SET_VELOCITY:
			payload_len = 4U;
			if (buffer_len < payload_len + 1U) {
				return ML_FAULT;
			}
//			for (int i = 0; i < payload_len; i++) {
//				buffer[i+1] = ( (( (uint8_t)(payload->data.ML_Velocity)) >> (i * 8)) & 0xFFU );
//			}
			buffer[1] = (uint8_t)(payload->ML_Velocity->ML_Left_Velocity) & 0xFFU;
			buffer[2] = (uint8_t)(payload->ML_Velocity->ML_Left_Velocity) >> 8;
			buffer[3] = (uint8_t)(payload->ML_Velocity->ML_Right_Velocity) & 0xFFU;
			buffer[4] = (uint8_t)(payload->ML_Velocity->ML_Right_Velocity) >> 8;
			break;
		case ML_SET_ACCELERATION:
			payload_len = 4U;
			if (buffer_len < payload_len + 1U) {
				return ML_FAULT;
			}
//			for (uint8_t i = 0; i < payload_len; i++) {
//				buffer[i+1] = (uint8_t)((payload->data->ML_Acceleration >> (i*8U)) & 0xFFU);
//			}
			break;
		case ML_SET_DRV_MODE:
			payload_len = 2U;
			if (buffer_len < payload_len + 1U) {
				return ML_FAULT;
			}
//			for (uint8_t i = 0; i < payload_len; i++) {
//				buffer[i+1] = (uint8_t)((payload->data->ML_DriveMode >> (i*8U)) & 0xFFU);
//			}
 			break;
		case ML_INITIALIZATION:
			payload_len = 0U;
			break;
		case ML_DRIVE_START:
			payload_len = 0U;
			break;
		case ML_DRIVE_STOP:
			payload_len = 0U;
			break;
		default:
			payload_len = 0U;
	}

	*encoded_len = payload_len + 1U;
	return ML_OK;
}

