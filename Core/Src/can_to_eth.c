/*
 * can_to_eth.c
 *
 *  Created on: 2021/08/07
 *      Author: tsuna
 */

#include "can_to_eth.h"

#include "basic_queue.h"
#include "cs_type.h"
#include "udp.h"
#include "udp_debug.h"
#include "lwip.h"
#include "ip_addr.h"

static CAN_HandleTypeDef* g_can_handle;

static struct BasicQueue_t g_queue;
static struct CSIo_dataFrame_t g_staticBuffer[CAN_PACKET_BUFFER_COUNT];

static struct udp_pcb *g_send_pcb;
static ip4_addr_t g_send_addr;
static uint8_t g_sequence_counter;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		struct CSIo_dataFrame_t data_frame;
		data_frame.id = (uint16_t)RxHeader.StdId;
		data_frame.reg = (uint8_t)RxData[0];
		data_frame.len = (uint8_t)RxHeader.DLC - 1;
		data_frame.data[0] = (uint8_t)RxData[1];
		data_frame.data[1] = (uint8_t)RxData[2];
		data_frame.data[2] = (uint8_t)RxData[3];
		data_frame.data[3] = (uint8_t)RxData[4];

		//		char buff[256];
		//		static size_t count = 0;
		//		sprintf(buff, "can recv %d\n", count);
		//		UdpDebug_print(buff);
		//		count++;

		BasicQueue_add(&g_queue, &data_frame);

		HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
	}
}



void CanToEth_init(CAN_HandleTypeDef* can_handle)
{
	BasicQueue_init(&g_queue, sizeof(struct CSIo_dataFrame_t), CAN_PACKET_BUFFER_COUNT, g_staticBuffer);

	g_can_handle = can_handle;
	if(HAL_CAN_ActivateNotification(g_can_handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		UdpDebug_error("can_to_eth can set interrupt failed\n");
	}

	g_send_pcb = udp_new();
	IP4_ADDR(&g_send_addr, HOST_IP_ADDR1, HOST_IP_ADDR2, HOST_IP_ADDR3, HOST_IP_ADDR4);
	g_sequence_counter = 0;
}

void CanToEth_Process(void)
{
	size_t queue_count = BasicQueue_count(&g_queue);
	if(0 < queue_count)
	{
		if(4 < queue_count)
		{
			queue_count = 4;
		}

		struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct CSIo_ethFrame_t), PBUF_RAM);
		struct CSIo_ethFrame_t* eth_frame = (struct CSIo_ethFrame_t*)p->payload;

		eth_frame->sequence = g_sequence_counter;
		g_sequence_counter++;
		eth_frame->data_count = queue_count;

		for(size_t i = 0; i < queue_count; i++)
		{
			BasicQueue_get(&g_queue, &eth_frame->data_frame[i]);
		}

		  err_t err = udp_sendto(g_send_pcb, p, &g_send_addr, ETH_DATA_SEND_PORT);

		  if(err == ERR_OK)
		  {
			  pbuf_free(p);
				HAL_GPIO_TogglePin(ETH_TX_LED_GPIO_Port, ETH_TX_LED_Pin);
		  }else{
			  HAL_GPIO_TogglePin(ETH_ERROR_LED_GPIO_Port, ETH_ERROR_LED_Pin);
		  }
	}
}
