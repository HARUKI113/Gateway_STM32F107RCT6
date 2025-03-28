/*
 * udp_debug.h
 *
 *  Created on: Mar 26, 2021
 *      Author: sena2
 */

#ifndef INC_UDP_DEBUG_H_
#define INC_UDP_DEBUG_H_

#include "main.h"

void UdpDebug_initializer(void);

void UdpDebug_print(const char* str);

void UdpDebug_error(const char* str);

#endif /* INC_UDP_DEBUG_H_ */
