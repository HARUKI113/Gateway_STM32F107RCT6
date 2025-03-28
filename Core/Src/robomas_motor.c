/*
 * robomanster.h
 *
 *  Created on: Oct 20, 2021
 *      Author: tsuna
 */

#include "robomas_motor.h"

#include <string.h>
#include "udp.h"
#include "udp_debug.h"
#include "lwip.h"
#include "ip_addr.h"
#include "cs_type.h"

// RobotRMMのrobomas_motorのライブラリと対応
#define ROBOMAS_ETH_RPM_RECV_PORT	(20520)
#define ROBOMAS_ETH_CONF_RECV_PORT	(20521)
#define ROBOMAS_ETH_SEND_PORT		(20522)


// RobotRMMのrobomas_motorのライブラリと対応
struct RM_ethRpmFrame_t
{
	uint8_t mot_id;
	int16_t rpm;
} __attribute__((__packed__));


// 新しい出力送信フレーム互換性を維持するために，前のも残してる．
struct RM_ethRpmSyncFrame_t
{
	uint8_t mot_id1;
	uint8_t mot_id2;
	int16_t rpm;
} __attribute__((__packed__));


// 新しい出力送信フレーム互換性を維持するために，前のも残してる．
struct RM_ethPitchFrame_t
{
	uint8_t mot_id;
	uint16_t base_power;
	int64_t target_rota;
} __attribute__((__packed__));


// 新しい出力送信フレーム互換性を維持するために，前のも残してる．
struct RM_ethRpmNewFrame_t
{
	uint8_t mot_id;
    uint8_t power_mode; // 0 : rpm, 1 : current.
	int16_t value;
	uint8_t mot_type; // 0 : m3508, 1 : m2006
} __attribute__((__packed__));


// RobotRMMのrobomas_motorのライブラリと対応
struct RM_ethCoefFrame_t
{
	uint8_t mot_id;
	uint32_t p;
	uint32_t i;
	uint32_t d;
} __attribute__((__packed__));

struct RM_ethAdvancedCoefFrame_t
{
	uint8_t mot_id;
	uint32_t p;
	uint32_t i;
	uint32_t d;
    int8_t  direction;
} __attribute__((__packed__));

// RobotRMMのrobomas_motorのライブラリと対応
struct RM_ethSensData_t
{
	int16_t rpm;
	int16_t cur;
	int16_t set_cur;
	int64_t rota; // [2PI / 10]
	uint8_t status;
} __attribute__((__packed__));


#define SEQUENCE_MAP_SIZE 	(4) // 使ってない

// グローバル変数の宣言
static CAN_HandleTypeDef* g_can_handle;

static struct udp_pcb* g_recv_pcb;
static struct udp_pcb* g_recv_coef_pcb;
static struct udp_pcb* g_send_pcb;
static ip4_addr_t g_send_addr;

// 送受信データ格納用
static uint8_t g_power_mode[8];
static int16_t g_power_value[8];
static int64_t g_target_theta[8];
static int16_t g_max_current[8];
static int16_t g_min_current[8];
static struct RM_ethSensData_t g_sens_data[8];

// ロボますモータの回転数計測用変数
static uint16_t g_ang_befo[8];
static int32_t g_rota_count[8];

// PID用の変数等
static int32_t g_pid_u_i[8];
static int32_t g_pid_u_befo[8];
static const float g_lp_u_d_conf = 0.01;
static float g_lp_u_d[8];
static float g_pid_conf[8][3]; // PIDのパラメータ
static int8_t g_direction[8];
//static int64_t g_offset_theta[8];

// Safetyの管理用変数
static uint32_t g_next_timeout;
static uint8_t 	g_is_safety;

static uint32_t g_next_sendtim;

// 割り込み用
static TIM_HandleTypeDef* 	g_tim_handle;

/*
	RobotRMMから送られてくるRPMの受信（コールバック関数）
*/
void RoboMas_ethRpmRecvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	if(p->len == sizeof(struct RM_ethRpmFrame_t))
	{
		struct RM_ethRpmFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id < 8)
		{
			g_power_mode[eth_frame->mot_id] = 0;
			g_power_value[eth_frame->mot_id] = eth_frame->rpm;
			g_max_current[eth_frame->mot_id] = 16384;
			g_min_current[eth_frame->mot_id] = -16384;
		}
	}
	else if(p->len == sizeof(struct RM_ethRpmNewFrame_t))
	{
		struct RM_ethRpmNewFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id < 8)
		{
			g_power_mode[eth_frame->mot_id] = eth_frame->power_mode;
			g_power_value[eth_frame->mot_id] = eth_frame->value;
			if(eth_frame->mot_type == 0)
			{
				g_max_current[eth_frame->mot_id] = 16384;
				g_min_current[eth_frame->mot_id] = -16384;
			}else{
				g_max_current[eth_frame->mot_id] = 10000;
				g_min_current[eth_frame->mot_id] = -10000;
			}
		}
	}
	else if(p->len == sizeof(struct RM_ethRpmSyncFrame_t))
	{
		struct RM_ethRpmSyncFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id1 < 8 && eth_frame->mot_id2 < 8)
		{
			g_power_mode[eth_frame->mot_id1] = (eth_frame->mot_id2 << 4) | 2;
			g_power_value[eth_frame->mot_id1] = eth_frame->rpm;

			g_max_current[eth_frame->mot_id1] = 16384;
			g_min_current[eth_frame->mot_id1] = -16384;


			g_power_mode[eth_frame->mot_id2] = 3;
			g_power_value[eth_frame->mot_id2] = eth_frame->mot_id1;

			g_max_current[eth_frame->mot_id2] = 16384;
			g_min_current[eth_frame->mot_id2] = -16384;
		}
	}
	else if(p->len == sizeof(struct RM_ethPitchFrame_t))
	{
		struct RM_ethPitchFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id < 8)
		{
			g_power_mode[eth_frame->mot_id] = 4;
			g_power_value[eth_frame->mot_id] = eth_frame->base_power;
			g_target_theta[eth_frame->mot_id] = eth_frame->target_rota;
		}
	}
	pbuf_free(p);
}

/*
	RobotRMMから送られてくるPIDパラメータの受信（コールバック関数）
*/
void RoboMas_ethConfRecvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	if(p->len == sizeof(struct RM_ethCoefFrame_t))
	{
		struct RM_ethCoefFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id < 8)
		{
			g_pid_conf[eth_frame->mot_id][0] = (float)eth_frame->p / 10000;
			g_pid_conf[eth_frame->mot_id][1] = (float)eth_frame->i / 10000;
			g_pid_conf[eth_frame->mot_id][2] = (float)eth_frame->d / 10000;
		}

		struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct RM_ethCoefFrame_t), PBUF_RAM);
		memcpy(p->payload, (const void*)eth_frame, sizeof(struct RM_ethCoefFrame_t));

		err_t err = udp_sendto(pcb, p, addr, port);
		if(err != ERR_OK)
		{
			UdpDebug_error("send failed.");
		}
		pbuf_free(p);
	}
	else if(p->len == sizeof(uint8_t))
	{
		uint8_t* flg = p->payload;
		if(*flg == 0)
		{
			g_next_timeout = HAL_GetTick() + 500;
		}
	}
	else if(p->len == sizeof(struct RM_ethAdvancedCoefFrame_t))
	{
		struct RM_ethAdvancedCoefFrame_t* eth_frame = p->payload;
		if(eth_frame->mot_id < 8)
		{
			g_pid_conf[eth_frame->mot_id][0] = (float)eth_frame->p / 10000;
			g_pid_conf[eth_frame->mot_id][1] = (float)eth_frame->i / 10000;
			g_pid_conf[eth_frame->mot_id][2] = (float)eth_frame->d / 10000;

			g_direction[eth_frame->mot_id] = eth_frame->direction;
		}

		struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct RM_ethAdvancedCoefFrame_t), PBUF_RAM);
		memcpy(p->payload, (const void*)eth_frame, sizeof(struct RM_ethAdvancedCoefFrame_t));

		err_t err = udp_sendto(pcb, p, addr, port);
		if(err != ERR_OK)
		{
			UdpDebug_error("send failed.");
		}
		pbuf_free(p);
	}
	pbuf_free(p);
}

/*
	ロボますモータからの受信（コールバック関数）
	割り込み関数なので，ここまで行数が長いのは良くない．要改良
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
	{
		uint16_t ang;

		uint16_t can_id = (uint16_t)RxHeader.StdId;
		uint8_t mot_id = (can_id & 0x00F); // 受信したIDから，ロボますモータのIDを求める．

		if((can_id & 0xFF0) == 0x200 &&  1 <= mot_id && mot_id <= 8)
		{
			mot_id = mot_id - 1; // ロボますモータIDの正規化

			g_sens_data[mot_id].status = 1; // ロボマスモータが繋がっているかどうかの識別用のフラグ...よく覚えてない

			ang = ((int16_t)RxData[0] << 8) | RxData[1];
			g_sens_data[mot_id].rpm = (((int16_t)RxData[2] << 8) | RxData[3]) * g_direction[mot_id];
			g_sens_data[mot_id].cur = (((int16_t)RxData[4] << 8) | RxData[5]) * g_direction[mot_id];

			// ロボますから得られる角度は，シリが一回転すると振り切れちゃうので，振り切れを検知して計測する
			if(g_ang_befo[mot_id] < ang)
			{
				if(ang - g_ang_befo[mot_id] < 5191)
				{
				}else{
					g_rota_count[mot_id]--;
				}
			}else{
				if(g_ang_befo[mot_id] - ang < 5191)
				{
				}else{
					g_rota_count[mot_id]++;
				}
			}
		  g_sens_data[mot_id].rota = ((g_rota_count[mot_id] * 1000) + (((int64_t)ang * 1000) / 8191)) * g_direction[mot_id];
		  g_ang_befo[mot_id] = ang;
		}
	}
}


/*
	初期化
*/
void RobomasMotor_init(CAN_HandleTypeDef* can_handle, TIM_HandleTypeDef* htim)
{
	// CAN通信の初期化，データを受け取ったら，HAL_CAN_RxFifo1MsgPendingCallbackが送られるように，設定している.
	g_can_handle = can_handle;
	if(HAL_CAN_ActivateNotification(g_can_handle, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		UdpDebug_error("robomas can set interrupt failed\n");
	}

	for(size_t i = 0; i < 8; i++)
	{
		g_power_mode[i] = 0;
		g_power_value[i] = 0;
		g_max_current[i] = 16384;
		g_min_current[i] = -16384;

		g_ang_befo[i] = 0;
		g_rota_count[i] = 0;

		g_sens_data[i].rpm = 0;
		g_sens_data[i].cur = 0;
		g_sens_data[i].set_cur = 0;
		g_sens_data[i].rota = 0;
		g_sens_data[i].status = 0;

		g_pid_u_i[i] = 0;
		g_pid_u_befo[i] = 0;
		g_lp_u_d[i] = 0;

		g_pid_conf[i][0] = 5;
		g_pid_conf[i][1] = 0.1;
		g_pid_conf[i][2] = 0.05;

		g_direction[i] = 1;
	}

	// UDPの初期化，コールバック関数も設定している．
	g_recv_pcb = udp_new();
	if(udp_bind(g_recv_pcb, IP_ADDR_ANY, ROBOMAS_ETH_RPM_RECV_PORT) != ERR_OK)
	{
		UdpDebug_error("recv bind failed.");
	}
	udp_recv(g_recv_pcb, RoboMas_ethRpmRecvCallback, NULL);

	g_recv_coef_pcb = udp_new();
	if(udp_bind(g_recv_coef_pcb, IP_ADDR_ANY, ROBOMAS_ETH_CONF_RECV_PORT) != ERR_OK)
	{
		UdpDebug_error("recv bind failed.");
	}
	udp_recv(g_recv_coef_pcb, RoboMas_ethConfRecvCallback, NULL);

	g_send_pcb = udp_new();
	IP4_ADDR(&g_send_addr, HOST_IP_ADDR1, HOST_IP_ADDR2, HOST_IP_ADDR3, HOST_IP_ADDR4);

	g_next_timeout = 0;
	g_next_sendtim = HAL_GetTick() + 10;

	g_tim_handle = htim;
	HAL_TIM_Base_Start_IT(g_tim_handle);
	__HAL_TIM_SET_COUNTER(g_tim_handle, 0);
}


/*
	main側から，定期的に呼ばれる処理
	ロボますモータからのフィードバックをRobotRMMに送信している
	セーフティタイムアウトの処理も行なっている．
*/
void RobomasMotor_Process(void)
{
	if(1 < __HAL_TIM_GET_COUNTER(g_tim_handle))
	{
		RobomasMotor_timCallback();
	}

	if(g_next_sendtim < HAL_GetTick())
	{
		struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct RM_ethSensData_t) * 8, PBUF_RAM);
		memcpy(p->payload, g_sens_data, sizeof(struct RM_ethSensData_t) * 8);

		err_t err = udp_sendto(g_send_pcb, p, &g_send_addr, ROBOMAS_ETH_SEND_PORT);

		if(err != ERR_OK)
		{
			UdpDebug_error("send failed.");
		}
		pbuf_free(p);
		g_next_sendtim = HAL_GetTick() + 10;
	}

	if(g_next_timeout < HAL_GetTick())
	{
		g_is_safety = 1;
	}else{
		g_is_safety = 0;
	}
}

void RobomasMotor_timCallback(void)
{
	__HAL_TIM_SET_COUNTER(g_tim_handle, 0);

	// PIDで算出した出力値を，ロボますモータにCANで送信する処理
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(g_can_handle))
	{
		// Safetyがオンになってたら，モータを止める
		if(g_is_safety == 1)
		{
			for(size_t mot_id = 0; mot_id < 8; mot_id++)
			{
				g_sens_data[mot_id].set_cur = 0;
				g_pid_u_i[mot_id] = 0;
				g_pid_u_befo[mot_id] = 0;
				g_lp_u_d[mot_id] = 0;
			}
		}
		else
		{
			for(size_t mot_id = 0; mot_id < 8; mot_id++)
			{
				if(g_sens_data[mot_id].status == 1)
				{
					if(g_power_mode[mot_id] == 0)
					{
						// PID制御の開始
						int32_t u = (g_power_value[mot_id] - g_sens_data[mot_id].rpm);
						float p = g_pid_conf[mot_id][0];
						float i = g_pid_conf[mot_id][1];
						float d = g_pid_conf[mot_id][2];

						g_pid_u_i[mot_id] += u;
						if(g_pid_u_i[mot_id] < g_min_current[mot_id] || g_max_current[mot_id] < g_pid_u_i[mot_id])
						{
							g_pid_u_i[mot_id] -= u;
						}

						int32_t u_d = u - g_pid_u_befo[mot_id];
						g_pid_u_befo[mot_id] = u;
						g_lp_u_d[mot_id] = u_d * (g_lp_u_d_conf) + g_lp_u_d[mot_id] * (1 - g_lp_u_d_conf);

						int32_t set_cur = (int32_t)(p * u + i * g_pid_u_i[mot_id] + d * g_lp_u_d[mot_id]) * g_direction[mot_id];
						if(g_max_current[mot_id] < set_cur)
						{
							g_sens_data[mot_id].set_cur = g_max_current[mot_id];
						}
						else if(set_cur < g_min_current[mot_id])
						{
							g_sens_data[mot_id].set_cur = g_min_current[mot_id];
						}else{
							g_sens_data[mot_id].set_cur = set_cur;
						}
					}
					else if(g_power_mode[mot_id] == 1)
					{
						g_sens_data[mot_id].set_cur = g_power_value[mot_id] * g_direction[mot_id];
						g_pid_u_i[mot_id] = 0;
						g_pid_u_befo[mot_id] = 0;
						g_lp_u_d[mot_id] = 0;
					}
					else if((g_power_mode[mot_id] & 0x0F) == 2)
					{
						// PID制御の開始
						int16_t avg_rpm = (g_sens_data[mot_id].rpm + (g_sens_data[g_power_mode[mot_id] >> 4].rpm * -1)) / 2;
						int32_t u = (g_power_value[mot_id] - avg_rpm);
						float p = g_pid_conf[mot_id][0];
						float i = g_pid_conf[mot_id][1];
						float d = g_pid_conf[mot_id][2];

						g_pid_u_i[mot_id] += u;
						if(g_pid_u_i[mot_id] < g_min_current[mot_id] || g_max_current[mot_id] < g_pid_u_i[mot_id])
						{
							g_pid_u_i[mot_id] -= u;
						}

						int32_t u_d = u - g_pid_u_befo[mot_id];
						g_pid_u_befo[mot_id] = u;
						g_lp_u_d[mot_id] = u_d * (g_lp_u_d_conf) + g_lp_u_d[mot_id] * (1 - g_lp_u_d_conf);

						int32_t set_cur = (int32_t)(p * u + i * g_pid_u_i[mot_id] + d * g_lp_u_d[mot_id]);
						if(g_max_current[mot_id] < set_cur)
						{
							g_sens_data[mot_id].set_cur = g_max_current[mot_id];
						}
						else if(set_cur < g_min_current[mot_id])
						{
							g_sens_data[mot_id].set_cur = g_min_current[mot_id];
						}else{
							g_sens_data[mot_id].set_cur = set_cur;
						}
					}
					else if(g_power_mode[mot_id] == 3)
					{
						g_sens_data[mot_id].set_cur = g_sens_data[g_power_value[mot_id]].set_cur * -1;
						g_pid_u_i[mot_id] = 0;
						g_pid_u_befo[mot_id] = 0;
						g_lp_u_d[mot_id] = 0;
					}
					else if(g_power_mode[mot_id] == 4)
					{
						// PID制御の開始
						int32_t u = (g_sens_data[mot_id].rota - g_target_theta[mot_id]);
						u = u * g_direction[mot_id] * -1;
						if(200 < u)
						{
							u = 200;
						}else if(u < -200){
							u = -200;
						}
						float p = g_pid_conf[mot_id][0];
						float i = g_pid_conf[mot_id][1];
						float d = g_pid_conf[mot_id][2];

						g_pid_u_i[mot_id] += u;
						if(g_pid_u_i[mot_id] < (g_min_current[mot_id] / 3) || (g_max_current[mot_id] / 3) < g_pid_u_i[mot_id])
						{
							g_pid_u_i[mot_id] -= u;
						}

						int32_t u_d = u - g_pid_u_befo[mot_id];
						g_pid_u_befo[mot_id] = u;
						g_lp_u_d[mot_id] = u_d * (0.8) + g_lp_u_d[mot_id] * (1 - 0.8);

						int32_t set_cur = (g_power_value[mot_id] * g_direction[mot_id]) + (int32_t)(p * u + i * g_pid_u_i[mot_id] + d * g_lp_u_d[mot_id]);
						if(g_max_current[mot_id] < set_cur)
						{
							g_sens_data[mot_id].set_cur = g_max_current[mot_id];
						}
						else if(set_cur < g_min_current[mot_id])
						{
							g_sens_data[mot_id].set_cur = g_min_current[mot_id];
						}else{
							g_sens_data[mot_id].set_cur = set_cur;
						}
					}
				}
			}
		}

		uint8_t sum;
		sum = g_sens_data[0].status + g_sens_data[1].status + g_sens_data[2].status + g_sens_data[3].status;
		if(0 < sum)
		{
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t TxData[8];

			TxHeader.StdId = 0x200; 		// CAN ID
			TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
			TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
			TxHeader.DLC = 8;
			TxHeader.TransmitGlobalTime = DISABLE;  // ???
			TxData[0] = (uint8_t)(g_sens_data[0].set_cur >> 8);
			TxData[1] = (uint8_t)g_sens_data[0].set_cur;
			TxData[2] = (uint8_t)(g_sens_data[1].set_cur >> 8);
			TxData[3] = (uint8_t)g_sens_data[1].set_cur;
			TxData[4] = (uint8_t)(g_sens_data[2].set_cur >> 8);
			TxData[5] = (uint8_t)g_sens_data[2].set_cur;
			TxData[6] = (uint8_t)(g_sens_data[3].set_cur >> 8);
			TxData[7] = (uint8_t)g_sens_data[3].set_cur;

			HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(g_can_handle, &TxHeader, TxData, &TxMailbox);

			if(result != HAL_OK)
			{
				UdpDebug_error("robomas can send failed\n");
			}
		}

		sum = g_sens_data[4].status + g_sens_data[5].status + g_sens_data[6].status + g_sens_data[7].status;
		if(0 < sum)
		{
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t TxData[8];

			TxHeader.StdId = 0x1FF; 		// CAN ID
			TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
			TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
			TxHeader.DLC = 8;
			TxHeader.TransmitGlobalTime = DISABLE;  // ???
			TxData[0] = (uint8_t)(g_sens_data[4].set_cur >> 8);
			TxData[1] = (uint8_t)g_sens_data[4].set_cur;
			TxData[2] = (uint8_t)(g_sens_data[5].set_cur >> 8);
			TxData[3] = (uint8_t)g_sens_data[5].set_cur;
			TxData[4] = (uint8_t)(g_sens_data[6].set_cur >> 8);
			TxData[5] = (uint8_t)g_sens_data[6].set_cur;
			TxData[6] = (uint8_t)(g_sens_data[7].set_cur >> 8);
			TxData[7] = (uint8_t)g_sens_data[7].set_cur;

			HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(g_can_handle, &TxHeader, TxData, &TxMailbox);

			if(result != HAL_OK)
			{
				UdpDebug_error("robomas can send failed\n");
			}
		}
	}

	__HAL_TIM_SET_COUNTER(g_tim_handle, 0);
}

