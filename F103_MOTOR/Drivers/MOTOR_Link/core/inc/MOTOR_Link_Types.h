/*
 * MOTOR_Link_Types.h
 *
 *  Created on: Oct 29, 2026
 *      Author: pk
 */

#ifndef MOTOR_LINK_CORE_INC_MOTOR_LINK_TYPES_H_
#define MOTOR_LINK_CORE_INC_MOTOR_LINK_TYPES_H_

#include <stdint.h>
#include <stdio.h>
#include <stm32f1xx_hal.h>

#define ML_TX_BUFFER_SIZE 16
#define ML_RX_BUFFER_SIZE 32

#define SOF1 0xA5
#define SOF2 0x5A

typedef enum {
	ML_OK            = 0x00,
	ML_ERROR         = 0x01,
	ML_INVALID_PARAM = 0x02,
	ML_FAULT         = 0x03,
	ML_TX_BUSY       = 0x04,
	ML_TX_ERROR       = 0x05
} ML_Status;

typedef enum {
	ML_MODE_DISABLE = 0x00,
	ML_MODE_COAST   = 0x01,
	ML_MODE_BRAKE   = 0x02
} ML_DriveMode;

typedef enum {
	ML_SET_VELOCITY     = 0x00,
	ML_SET_ACCELERATION = 0x01,
	ML_SET_DRV_MODE     = 0x02,
	ML_INITIALIZATION   = 0x03,
	ML_DRIVE_START      = 0x04,
	ML_DRIVE_STOP       = 0x05,
} ML_CommandID;


typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t tx_flag;
	uint8_t tx_buffer[ML_TX_BUFFER_SIZE];
	uint8_t rx_buffer[ML_RX_BUFFER_SIZE];
}ML_UARTInterface;

typedef struct {
	int16_t ML_Left_Velocity;
	int16_t ML_Right_Velocity;

	uint16_t ML_Left_Acceleration;
	uint16_t ML_Right_Acceleration;

	uint8_t ML_Left_DriveMode;
	uint8_t ML_Right_DriveMode;
}ML_Command;

typedef struct {
	int16_t ML_Left_Velocity;
	int16_t ML_Right_Velocity;

	uint8_t VBAT;
	uint8_t FAULT;
	uint8_t STATUS;
}ML_Feedback;

typedef struct{
	ML_UARTInterface uart;
	ML_Command command;
	ML_Feedback feedback;
}ML_MotorLink;


#endif /* MOTOR_LINK_CORE_INC_MOTOR_LINK_TYPES_H_ */
