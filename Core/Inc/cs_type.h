/*
 * cs_type.h
 *
 *  Created on: Aug 6, 2021
 *      Author: tsuna
 */

#ifndef INC_CS_TYPE_H_
#define INC_CS_TYPE_H_

#define CSTYPE_TRUE 	(1)
#define CSTYPE_FALSE	(0)
typedef uint8_t 	CSType_bool_t;

#define HOST_IP_ADDR1	(172)
#define HOST_IP_ADDR2	(22)
#define HOST_IP_ADDR3	(4)
#define HOST_IP_ADDR4	(129)

#define ETH_DATA_RECV_PORT 	(20100)
#define ETH_DATA_SEND_PORT 	(20101)
#define ETH_ERROR_SEND_PORT 	(20111)
#define ETH_PING_SEND_PORT 	(20121)

#define CSIO_DATAPACK_COUNT (4)

#define CAN_PACKET_BUFFER_COUNT (CSIO_DATAPACK_COUNT * 2)

typedef uint16_t CanId_t;
typedef uint8_t 	CanReg_t;

struct CSIo_canFrame_t{
	uint8_t reg;
	uint8_t data[4];
}__attribute__((__packed__));


struct CSIo_dataFrame_t
{
    uint16_t id;
    uint8_t reg;
    uint8_t len;
    uint8_t data[4];
} __attribute__((__packed__));

struct CSIo_ethFrame_t
{
    uint8_t sequence;
    uint8_t data_count;
    struct CSIo_dataFrame_t data_frame[CSIO_DATAPACK_COUNT];
} __attribute__((__packed__));

struct CSIo_dataPack_t
{
    uint8_t data_count;
    struct CSIo_dataFrame_t data[CSIO_DATAPACK_COUNT];
};



#endif /* INC_CS_TYPE_H_ */
