/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file    gs_can_types.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji type
 *
 ******************************************************************************/ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GS_CAN_DJI_PROTOCOL_H
#define __GS_CAN_DJI_PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
/* Private typedef -----------------------------------------------------------*/
/**
 * @brief state of packet
 *
 * Contains state related to packet
 */
typedef enum __gs_can_parse_status_t{
	GS_CAN_PARSE_STATE_IDLE,
	GS_CAN_PARSE_STATE_DELIMITER,
	GS_CAN_PARSE_STATE_LEN,
	GS_CAN_PARSE_STATE_VERSION,
	GS_CAN_PARSE_STATE_CRC8,
	GS_CAN_PARSE_STATE_SENDER_ID,
	GS_CAN_PARSE_STATE_RECEIVER_ID,
	GS_CAN_PARSE_STATE_SEQ1,
	GS_CAN_PARSE_STATE_SEQ2,
	GS_CAN_PARSE_STATE_ACK,
	GS_CAN_PARSE_STATE_CMDSET,
	GS_CAN_PARSE_STATE_CMD,
	GS_CAN_PARSE_STATE_PAYLOAD,
	GS_CAN_PARSE_STATE_CRC16_1,
	GS_CAN_PARSE_STATE_CRC16_2,
	GS_CAN_PARSE_STATE_COUNT,
}gs_can_parse_status_t;

/**
 * @brief gs_can_src_dest_id_t
 *
 * source send data to destination ID device 
 */
typedef enum __gs_can_src_dest_t
{
	SRC_DEST_INVALID	= 0,
	SRC_DEST_CAMERA		= 1,
	SRC_DEST_APP		= 2,
	SRC_DEST_FC			= 3,	/** Flight Controller*/
	SRC_DEST_GIMBAL		= 4,
	SRC_DEST_CENTER_BOARD = 5,
	SRC_DEST_REMOTE_CONTROL = 6,
	SRC_DEST_HD_LINK		= 9,	/**LB2*/
	SRC_DEST_BATT			= 11,
	SRC_DEST_ESC			= 12,
	
	
	
	
	
	SRC_DEST_IMU				= 25,
	SRC_DEST_GPS_RTK			= 26,
	
	
	SRC_DEST_UNK				= 30,
	SRC_DEST_LAST				= 31
}gs_can_src_dest_id_t;

/**
 * @brief gs_can_cmd_set_t
 *
 * command set to specify devices
 */
typedef enum __gs_can_cmd_set_t
{
	CAN_DJI_CMD_SET_GENERAL = 0,
	CAN_DJI_CMD_SET_SPECIAL = 1,
	CAN_DJI_CMD_SET_CAMERA	= 2,
	CAN_DJI_CMD_SET_FC		= 3,
	CAN_DJI_CMD_SET_GIMBAL	= 4,
	CAN_DJI_CMD_SET_CENTER_BOARD,
	CAN_DJI_CMD_SET_REMOTE_CONTROL,
	CAN_DJI_CMD_SET_WIFI,
	CAN_DJI_CMD_SET_DM36X,
	CAN_DJI_CMD_SET_HD_LINK = 9,
	CAN_DJI_CMD_SET_MONO_BINOCULAR,
	CAN_DJI_CMD_SET_SIMULATOR,
	CAN_DJI_CMD_SET_ESC,
	CAN_DJI_CMD_SET_BATTERY,
	CAN_DJI_CMD_SET_DATA_LOGGER,
	CAN_DJI_CMD_SET_RTK,
	CAN_DJI_CMD_SET_AUTOMATION = 16,
}gs_can_cmd_set_t;

/**
 * @brief gs_can_ack_t
 */
typedef enum __gs_can_ack 
{
	RESPONSE,
	CMD_SEND,
	CMD_RECV,
	UNKNOW
} gs_can_ack_t;

/**
 * @brief can_dji_freq
 */
typedef enum _can_dji_freq
{
    CAN_DJI_FREQ_1HZ = 1,
    CAN_DJI_FREQ_5HZ = 5,
    CAN_DJI_FREQ_10HZ = 10,
    CAN_DJI_FREQ_20HZ = 20,
    CAN_DJI_FREQ_30HZ = 30,
    CAN_DJI_FREQ_40HZ = 40,
    CAN_DJI_FREQ_50HZ = 50
} can_dji_freq_t;
/**
 * @brief gs_can_data_t
 *
 * Header can protocol
 */
typedef struct __gs_can_data_t{
	uint32_t 	StdId;		/** CAN ID*/
	uint32_t 	DLC;		/** Length of CAN */
	uint8_t		*Data;		/** Packet*/
}gs_can_data_t;

/**
 * @brief gs_can_status_private_t
 *
 */
typedef struct __gs_can_status_private_t{
	uint8_t			packetIdx;
	uint16_t		currentTxSeq;
	uint16_t		crc16;
}gs_can_status_private_t;

/**
 * @brief gs_can_status_t
 *
 */
typedef struct __gs_can_status_t{
	gs_can_parse_status_t status;
	
	gs_can_data_t	value;
	
	gs_can_status_private_t zPrivate;
}gs_can_status_t;


/**
 * @brief can protocol struct 
 *
 * Contains data
 */
typedef struct __gs_can_message {
	uint16_t 	crc16;					/** [B+Payload] CRC*/
	uint8_t		delimiter;				/**	[0]  Start of Pkt, always 0x55*/
	uint8_t		length;					/**	[1]  Length of Pkt */
	uint8_t		protocol_version;		/** [2]  Protocol version*/
	uint8_t 	crc8;					/**	[3]  Header CRC*/ 
	uint8_t		senderId;				/** [4]  Sender */
	uint8_t		receiverId;				/** [5]  Receiver */
	uint16_t	seq;					/** [6-7]  Sequence Ctr */
	uint8_t		ack;					/** [8] Encryption/ACK */
	uint8_t		cmdSet;					/** [9] Cmd Set*/
	uint8_t		cmd;					/** [A] Cmd*/
	uint8_t		payload[256];			/** [B] Payload (optional)*/
} gs_can_message_t;

/* Private define ------------------------------------------------------------*/

/* 3 bytes include header + len + ver*/
#define GS_CAN_HEADER_STX2VER_LEN	(uint8_t)3

/* 8 bytes include header + len + ver + crc8 + sender + src + seq + ack + cmdSet + cmd*/
#define GS_CAN_HEADER_STX2SEQ_LEN	(uint8_t)11

	/* 2 byte checksum */
#define GS_CAN_FOOTER_CRC_LEN		(uint8_t)2

/* 8 bytes include header + len + ver + crc8 + sender + src + seq(2byte) + ack + cmdSet + cmd + crc16(2byte) */
#define GS_CAN_SYSTEM_LEN			(uint8_t)(GS_CAN_HEADER_STX2SEQ_LEN + GS_CAN_FOOTER_CRC_LEN)

/** Maximum payload length*/
#define GS_CAN_MAX_PAYLOAD_LEN 	255

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
* @brief @brief gs_can_coppy
* @param  buff: pointer to a buffer contains data from src
* @param  buffSrc: pointer to a srouce buffer
* @param  count: number of bytes
* @return None
*/
void gs_can_coppy(void* buff, void* buffSrc, int count);

/**
* @brief @brief gs_can_compare
* @param  buff: pointer to a buffer contains data 1
* @param  buffSrc: pointer to a buffer contains data  2
* @param  count: number of bytes
* @return True/false
*/
uint8_t gs_can_compare(void* buff, void* buffSrc, int count);

/**
* @brief @brief gs_can_crc8
* @param  len: length of packet which wants to check sum
* @return True/false
*/
uint8_t gs_can_crc8(uint8_t len);

/**
* @brief @brief gs_can_msg_pack
* @param  buff: pointer to a buffer contains data 1
* @param  buffSrc: pointer to a buffer contains data  2
* @param  count: number of bytes
* @return True/false
*/
uint16_t gs_can_msg_pack(gs_can_message_t* msg, gs_can_status_t* state, uint8_t sender_id, uint8_t receiver_id, uint8_t length, uint8_t smdSet, uint8_t cmd);

/**
* @brief gs_can_parse
* @param  canid: Specific canID to parse
* @param  msg: packet after parse data from can system
* @param  state: pointer to data
* @return True/false
*/
uint8_t gs_can_parse(uint32_t canid, gs_can_message_t *msg, gs_can_status_t* state);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GS_CAN_DJI_PROTOCOL_H */
/************************ GREMSY Co., Ltd *****END OF FILE****/
