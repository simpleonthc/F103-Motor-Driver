/*
 * MOTOR_Link.h
 *
 *  Created on: Oct 26, 2026
 *      Author: pk
 */

#ifndef MOTOR_LINK_CORE_INC_MOTOR_LINK_H_
#define MOTOR_LINK_CORE_INC_MOTOR_LINK_H_

#include <stdint.h>
#include <stdio.h>

typedef enum {
	ML_OK            = 0x00,
	ML_ERROR         = 0x01,
	ML_INVALID_PARAM = 0x02,
	ML_FAULT         = 0x03
} ML_Status;

typedef enum {
	ML_MODE_DISABLE = 0x00,
	ML_MODE_COAST   = 0x01,
	ML_MODE_BRAKE   = 0x02
} ML_DriveMode;

typedef enum {
	ML_SET_VELOCITY     = 0x06,
	ML_SET_ACCELERATION = 0x01,
	ML_SET_DRV_MODE     = 0x02,
	ML_INITIALIZATION   = 0x03,
	ML_DRIVE_START      = 0x04,
	ML_DRIVE_STOP       = 0x05,
} ML_CommandID;

typedef struct {
	int16_t ML_Left_Velocity;
	int16_t ML_Right_Velocity;
} ML_CommandVelocity;

typedef struct {
	uint16_t ML_Left_Acceleration;
	uint16_t ML_Right_Acceleration;
} ML_CommandAcceleration;

typedef struct {
	uint8_t ML_Left_DriveMode;
	uint8_t ML_Right_DriveMode;
} ML_CommandDriveMode;


typedef struct {
	ML_CommandID ML_CommandID;
	union {
		ML_CommandVelocity *ML_Velocity;
		ML_CommandAcceleration *ML_Acceleration;
		ML_CommandDriveMode *ML_DriveMode;
	};
} ML_Payload;

/*
 * @brief:
 * @Param:
 * @retval:
 * */
ML_Status ML_EncodeCommand(uint8_t *buffer, ML_Payload *payload, size_t buffer_len, size_t *encoded_len);

/*
 * @brief:
 * @Param:
 * @retval:
 * */
ML_Status ML_DecodeCommand(ML_CommandID *CommandID);

#endif /* MOTOR_LINK_CORE_INC_MOTOR_LINK_H_ */
