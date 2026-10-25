/*
 * DRV8848.h
 *
 *  Created on: Oct 24, 2026
 *      Author: lmaoPkk
 */

#ifndef DRV8848_DRV8848_H_
#define DRV8848_DRV8848_H_

#include "DRV8848_types.h"
#include "stm32f1xx_hal.h"

#define dutyCycle_cutoff 0.1f

typedef enum {
	DIR_FWD,
	DIR_REV
} DRV8848_DIR;

typedef enum {
    DRV8848_OK = 0,
    DRV8848_ERROR,
    DRV8848_INVALID_PARAM,
    DRV8848_FAULT
} DRV8848_Status;

typedef enum {
	DRV8848_COAST,
	DRV8848_BRAKE
} DRV8848_Mode;

typedef struct DRV8848{
	GPIO_TypeDef *nSleep_Port;
	uint16_t nSleep_Pin;

	TIM_HandleTypeDef *htim;
	uint32_t ain1_Channel;
	uint32_t ain2_Channel;

	GPIO_TypeDef *nFault_Port;
	uint16_t nFault_Pin;

	DRV8848_Mode decayMode;
} DRV8848;

/*
 * @brief   Initializes the driver
 * @param   self: pointer to DRV8848 struct
 * @retval  DRV8848_Status
 * */
DRV8848_Status DRV8848_Init(DRV8848 *self);

/*
 * @brief   Enables the driver
 * @param   self: pointer to DRV8848 struct
 * @retval  DRV8848_Status
 * */
DRV8848_Status DRV8848_Enable(DRV8848 *self);

/*
 * @brief   Disables the driver
 * @param   self: pointer to DRV8848 struct
 * @retval  DRV8848_Status
 * */
DRV8848_Status DRV8848_Disable(DRV8848 *self);

/*
 * @brief   Drives single motor
 * @param   self: pointer to DRV8848 struct
 * @param   duty: Duty cycle from 0.0f to 1.0f
 * @param   dir: Direction of the motor:
 *          	 DRV8848_FWD
 *          	 DRV8848_REV
 *
 * @retval  DRV8848_Status
 * */
DRV8848_Status DRV8848_setDrive(DRV8848 *self, float duty, DRV8848_DIR dir);

/*
 * Error detection
 * */
uint8_t DRV8848_IsFaultActive(const DRV8848 *self);

/*
 * @brief   Handles error
 * @param   self: pointer to DRV8848 struct
 * @retval  DRV8848_Status
 * */
DRV8848_Status DRV8848_ErrHandler(DRV8848 *self);

#endif /* DRV8848_DRV8848_H_ */
