/*
 * can_to_eth.h
 *
 *  Created on: Aug 6, 2021
 *      Author: tsuna
 */

#ifndef SRC_CAN_TO_ETH_H_
#define SRC_CAN_TO_ETH_H_

#include "main.h"

void CanToEth_init(CAN_HandleTypeDef* can_handle);

void CanToEth_Process(void);


#endif /* SRC_CAN_TO_ETH_H_ */
