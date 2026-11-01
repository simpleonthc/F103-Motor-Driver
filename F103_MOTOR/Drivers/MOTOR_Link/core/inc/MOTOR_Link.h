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
#include <stdbool.h>
#include "MOTOR_Link_Types.h"
#include "motor_link_uart.h"

ML_Status ML_sendVelocity(ML_MotorLink *self, int16_t leftVelocity, int16_t rightVelocity);
ML_Status ML_receiveVelocity(ML_MotorLink *self, int16_t *leftVelocity, int16_t *rightvelocity);
/*
 * @brief:
 * @Param:
 * @retval:
 * */

#endif /* MOTOR_LINK_CORE_INC_MOTOR_LINK_H_ */
