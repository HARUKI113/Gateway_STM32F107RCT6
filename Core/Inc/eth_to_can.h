/*
 * eth_to_can.h
 *
 *  Created on: Aug 6, 2021
 *      Author: tsuna
 */

#ifndef SRC_ETH_TO_CAN_H_
#define SRC_ETH_TO_CAN_H_

#include "main.h"

void EthToCan_init(CAN_HandleTypeDef* can_handle);

void EthToCan_Process(void);

#endif /* SRC_ETH_TO_CAN_H_ */
