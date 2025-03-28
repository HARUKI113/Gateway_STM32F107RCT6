/*
 * eth_to_can.h
 *
 *  Created on: Aug 6, 2021
 *      Author: tsuna
 */

#ifndef INC_ROBOMAS_MOTOR_H_
#define INC_ROBOMAS_MOTOR_H_

#include "main.h"
#include "stm32f1xx_hal_tim.h"

void RobomasMotor_init(CAN_HandleTypeDef* can_handle, TIM_HandleTypeDef* htim);

void RobomasMotor_Process(void);

void RobomasMotor_timCallback(void);

#endif /* INC_ROBOMAS_MOTOR_H_ */
