/*
 * udp_ping.c
 *
 *  Created on: Aug 7, 2021
 *      Author: tsuna
 */

#include "udp_ping.h"

#include "cs_type.h"
#include "udp.h"
#include "udp_debug.h"
#include "lwip.h"
#include "ip_addr.h"
#include <string.h>

#define PING_STRING	"Hello"

static uint32_t g_next_sendtim;

static struct udp_pcb *g_send_pcb;
static ip4_addr_t g_send_addr;

extern ETH_HandleTypeDef heth;


void UdpPing_init(void)
{
	g_send_pcb = udp_new();
	IP4_ADDR(&g_send_addr, HOST_IP_ADDR1, HOST_IP_ADDR2, HOST_IP_ADDR3, HOST_IP_ADDR4);

	g_next_sendtim = HAL_GetTick() + 400;
}

void UdpPing_Process(void)
{
	if(g_next_sendtim < HAL_GetTick())
	{
		struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(PING_STRING), PBUF_RAM);
		memcpy(p->payload, PING_STRING, sizeof(PING_STRING));
		udp_sendto(g_send_pcb, p, &g_send_addr, ETH_PING_SEND_PORT);
		pbuf_free(p);

		g_next_sendtim = HAL_GetTick() + 400;
	}
}
