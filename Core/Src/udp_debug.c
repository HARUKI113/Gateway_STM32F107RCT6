/*
 * udp_debug.c
 *
 *  Created on: Mar 26, 2021
 *      Author: sena2
 */

#include "udp_debug.h"
#include "lwip.h"

#include "cs_type.h"
#include <string.h>

struct UdpDebug_t
{
	struct udp_pcb *pcb;
	ip4_addr_t send_addr;
	unsigned short send_port;
};

static struct UdpDebug_t g_root;


static void UdpDebug_errorLedOn(void)
{
//	HAL_GPIO_TogglePin(CAN_ERROR_LED_GPIO_Port, CAN_ERROR_LED_Pin);
}


void UdpDebug_initializer(void)
{
	IP4_ADDR(&g_root.send_addr, HOST_IP_ADDR1, HOST_IP_ADDR2, HOST_IP_ADDR3, HOST_IP_ADDR4);
	g_root.send_port = ETH_ERROR_SEND_PORT;

	g_root.pcb = udp_new();
}


void UdpDebug_print(const char* str)
{
	size_t len = strlen(str);
	struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len + 1, PBUF_RAM);
	memcpy(p->payload, str, len + 1);
	if(udp_sendto(g_root.pcb, p, &g_root.send_addr, g_root.send_port) != ERR_OK)
	{
//		HAL_GPIO_WritePin(CAN_ERROR_LED_GPIO_Port, CAN_ERROR_LED_Pin, GPIO_PIN_RESET);
	}

	pbuf_free(p);
}


void UdpDebug_error(const char* str)
{
	UdpDebug_errorLedOn();
	UdpDebug_print(str);
}
