/*
 * eth_to_can.c
 *
 *  Created on: Aug 6, 2021
 *      Author: tsuna
 */
#include "eth_to_can.h"

#include "basic_queue.h"
#include "cs_type.h"
#include "udp.h"
#include "udp_debug.h"
#include "lwip.h"
#include "ip_addr.h"

#define CSTYPE_BROADCAST_ADDR	(0x001)
#define CSTYPE_SAFETY_TIMEOUT 	(500) // 500ms

typedef enum
{
	CSType_brcReg_Safety = 0x00,
	CSType_brcReg_Unsafe = 0x01,
	CSType_brcReg_Arp = 0x10,
	CSType_brcReg_ChipInit = 0x20
} CSType_brcReg_t;


#define SEQUENCE_MAP_SIZE 	(4)

static CAN_HandleTypeDef* g_can_handle;

static struct BasicQueue_t g_queue;
static struct CSIo_dataFrame_t g_staticBuffer[CAN_PACKET_BUFFER_COUNT];

static struct udp_pcb *g_recv_pcb;

static uint8_t g_next_map_index;
static uint8_t g_sequence_map[SEQUENCE_MAP_SIZE];

static uint8_t 	g_is_safety_start;
static uint32_t g_safety_time;


void EthToCan_recvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	if(p->len == sizeof(struct CSIo_ethFrame_t))
	{
		struct CSIo_ethFrame_t* eth_frame = p->payload;

		uint8_t is_effective = 1;
		for(size_t i = 0; i < SEQUENCE_MAP_SIZE; i++)
		{
			if(g_sequence_map[i] == eth_frame->sequence)
			{
				is_effective = 0;
				break;
			}
		}

		if(is_effective)
		{
			for(size_t i = 0; i < eth_frame->data_count; i++)
			{
				BasicQueue_add(&g_queue, &eth_frame->data_frame[i]);
			}

			g_sequence_map[g_next_map_index % SEQUENCE_MAP_SIZE] = eth_frame->sequence;
			g_next_map_index++;

			HAL_GPIO_TogglePin(ETH_RX_LED_GPIO_Port, ETH_RX_LED_Pin);
		}
	}
	pbuf_free(p);
}


void EthToCan_init(CAN_HandleTypeDef* can_handle)
{
	BasicQueue_init(&g_queue, sizeof(struct CSIo_dataFrame_t), CAN_PACKET_BUFFER_COUNT, g_staticBuffer);

	g_can_handle = can_handle;

	g_recv_pcb = udp_new();
	if(udp_bind(g_recv_pcb, IP_ADDR_ANY, ETH_DATA_RECV_PORT) != ERR_OK)
	{
		UdpDebug_error("recv bind failed.");
	}
	udp_recv(g_recv_pcb, EthToCan_recvCallback, NULL);

	g_next_map_index = 0;
	memset(g_sequence_map, UINT8_MAX, sizeof(uint8_t) * SEQUENCE_MAP_SIZE);

	g_is_safety_start = 0;
	g_safety_time = 0;
}


void EthToCan_Process(void)
{
	if(0 < BasicQueue_count(&g_queue))
	{
		if(0 < HAL_CAN_GetTxMailboxesFreeLevel(g_can_handle))
		{
			struct CSIo_dataFrame_t recv_data;
			BasicQueue_get(&g_queue, &recv_data);
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t TxData[8];

			TxHeader.StdId = recv_data.id; 		// CAN ID
			TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
			TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
			TxHeader.DLC = recv_data.len + 1;
			TxHeader.TransmitGlobalTime = DISABLE;  // ???
			TxData[0] = recv_data.reg;
			TxData[1] = recv_data.data[0];
			TxData[2] = recv_data.data[1];
			TxData[3] = recv_data.data[2];
			TxData[4] = recv_data.data[3];

			HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(g_can_handle, &TxHeader, TxData, &TxMailbox);

			if(recv_data.id == CSTYPE_BROADCAST_ADDR)
			{
				switch(recv_data.reg)
				{
				case CSType_brcReg_Safety:
					g_is_safety_start = 1;
					g_safety_time = HAL_GetTick() + CSTYPE_SAFETY_TIMEOUT;
					break;
				case CSType_brcReg_Unsafe:
					if(recv_data.len == 4)
					{
						if(recv_data.data[0] == 'U' && recv_data.data[1] == 'N' && recv_data.data[2] == 'S' && recv_data.data[3] == 'A')
						{
							g_safety_time = 0;
						}
					}
					break;
				default:
					break;
				}
			}

			if(result == HAL_OK)
			{
				HAL_GPIO_TogglePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin);
			}else{
//				HAL_GPIO_TogglePin(CAN_ERROR_LED_GPIO_Port, CAN_ERROR_LED_Pin);
				UdpDebug_error("can send failed");
			}
		}else{
//			HAL_GPIO_TogglePin(CAN_ERROR_LED_GPIO_Port, CAN_ERROR_LED_Pin);
			UdpDebug_error("Send queue overflowed");
		}
	}

	if(g_is_safety_start == 1 && g_safety_time < HAL_GetTick())
	{
//		HAL_GPIO_WritePin(SAFETY_PIN_GPIO_Port, SAFETY_PIN_Pin, GPIO_PIN_RESET);
	}else{
//		HAL_GPIO_WritePin(SAFETY_PIN_GPIO_Port, SAFETY_PIN_Pin, GPIO_PIN_SET);
	}
}



