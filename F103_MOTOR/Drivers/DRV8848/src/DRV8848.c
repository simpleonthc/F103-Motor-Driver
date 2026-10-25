/*
 * DRV8848.c
 *
 *  Created on: Oct 24, 2026
 *      Author: pk
 */
#include "DRV8848.h"

DRV8848_Status DRV8848_Init(DRV8848 *self) {
	DRV8848_Disable(self);
	HAL_TIM_PWM_Start(self->htim, self->ain1_Channel);
	HAL_TIM_PWM_Start(self->htim, self->ain2_Channel);
	return DRV8848_OK;
}

DRV8848_Status DRV8848_Enable(DRV8848 *self) {
	HAL_GPIO_WritePin(self->nSleep_Port, self->nSleep_Pin, GPIO_PIN_SET);
	return DRV8848_OK;
}

DRV8848_Status DRV8848_Disable(DRV8848 *self) {
	HAL_GPIO_WritePin(self->nSleep_Port, self->nSleep_Pin, GPIO_PIN_RESET);
	return DRV8848_OK;
}

DRV8848_Status DRV8848_setDrive(DRV8848 *self, float duty, DRV8848_DIR DIR) {
	if (duty <= 0.0f || duty >= 1.0f) {
		return DRV8848_INVALID_PARAM;
	}

	uint16_t Period_p1 = (uint16_t)(self->htim->Init.Period) + 1;
	uint16_t Decay;
	if (duty < dutyCycle_cutoff) {
		if (self->decayMode == DRV8848_COAST) {
			Decay = Period_p1;
		}
		else {
			Decay = 0;
		}
		__HAL_TIM_SET_COMPARE(self->htim,self->ain1_Channel, Decay);
		__HAL_TIM_SET_COMPARE(self->htim,self->ain2_Channel, Decay);
	}
	else {
		uint16_t CCR = (uint16_t)(Period_p1 * duty);
		if (DIR) {
			__HAL_TIM_SET_COMPARE(self->htim,self->ain1_Channel, CCR);
			__HAL_TIM_SET_COMPARE(self->htim,self->ain2_Channel, 0);
		}
		else {
			__HAL_TIM_SET_COMPARE(self->htim,self->ain1_Channel, 0);
			__HAL_TIM_SET_COMPARE(self->htim,self->ain2_Channel, CCR);
		}
	}

	return DRV8848_OK;
}

uint8_t DRV8848_IsFaultActive(const DRV8848 *self) {
	return 0;
}

DRV8848_Status DRV8848_ErrHandler(DRV8848 *self) {
	DRV8848_Disable(self);
	HAL_TIM_PWM_Stop(self->htim, self->ain1_Channel);
	HAL_TIM_PWM_Stop(self->htim, self->ain2_Channel);
	__HAL_TIM_SET_COMPARE(self->htim,self->ain1_Channel, 0);
	__HAL_TIM_SET_COMPARE(self->htim,self->ain2_Channel, 0);

	return DRV8848_OK;
}

