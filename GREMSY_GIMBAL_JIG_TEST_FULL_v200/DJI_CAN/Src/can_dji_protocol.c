/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file    can_dji.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji protocol
 *
 ******************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include <inttypes.h>
#include <string.h>
#include "can_dji_protocol.h"

/** Support CRC*/
#include "can_dji_crc_8.h"
#include "can_dji_crc_16.h"

/* Private define-------------------------------------------------------------*/

#define GS_CAN_DELIMITER					0x55
#define GS_CAN_VERSION						0x04
/* Private Typedef------------------------------------------------------------*/
/* Exported Global variables------------------------------------------------- */
/* Private variable- ---------------------------------------------------------*/
/* Private function- ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
* @brief @brief gs_can_coppy
* @param  buff: pointer to a buffer contains data from src
* @param  buffSrc: pointer to a srouce buffer
* @param  count: number of bytes
* @return None
*/
void gs_can_coppy(void* buff, void* buffSrc, int count)
{
	int i = 0;
	uint8_t* ptr 		= buff;
	uint8_t* ptrSrc 	= buffSrc;
	
	for(i = 0; i < count; i++){
		ptr[i] = ptrSrc[i];
	}
}

/**
* @brief @brief gs_can_compare
* @param  buff: pointer to a buffer contains data 1
* @param  buffSrc: pointer to a buffer contains data  2
* @param  count: number of bytes
* @return True/false
*/
uint8_t gs_can_compare(void* buff, void* buffSrc, int count)
{
	int i = 0;
	uint8_t* ptr 		= buff;
	uint8_t* ptrSrc 	= buffSrc;
	
	for(i = 0; i < count; i++){
		if(ptr[i] != ptrSrc[i]){
			return 0;
		}
	}
	
	return 1;
}

/**
* @brief @brief gs_can_crc8
* @param  len: length of packet which wants to check sum
* @return True/false
*/
uint8_t gs_can_crc8(uint8_t len)
{
	uint8_t crc8 = gs_can_crc_8_init();
	uint8_t buff[GS_CAN_HEADER_STX2VER_LEN] = {GS_CAN_DELIMITER, len, GS_CAN_VERSION};
	
	crc8 = gs_can_crc_8_update(crc8, buff, GS_CAN_HEADER_STX2VER_LEN);
	crc8 = gs_can_crc_8_finalize(crc8);
	
	return crc8;
}

uint16_t gs_can_msg_pack(gs_can_message_t* msg, gs_can_status_t* state, uint8_t sender_id, uint8_t receiver_id, uint8_t length, uint8_t cmdSet, uint8_t cmd)
{
	msg->delimiter 			= GS_CAN_DELIMITER;
	msg->length 			= length + GS_CAN_SYSTEM_LEN;
	msg->protocol_version 	= GS_CAN_VERSION;
	
	msg->crc8 = gs_can_crc_8_init();
	msg->crc8 = gs_can_crc_8_update(msg->crc8, &msg->delimiter, GS_CAN_HEADER_STX2VER_LEN);
	msg->crc8 = gs_can_crc_8_finalize(msg->crc8);
	
	msg->senderId	= sender_id;
	msg->receiverId	= receiver_id;
	msg->seq = state->zPrivate.currentTxSeq++;
	
	msg->crc16 = gs_can_crc_16_init();
	msg->crc16 = gs_can_crc_16_update(msg->crc16, &msg->delimiter, msg->length - GS_CAN_FOOTER_CRC_LEN);
	msg->crc16 = gs_can_crc_16_finalize(msg->crc16);
	
  msg->cmdSet = cmdSet;
  msg->cmd    = cmd;
	//checksum
	msg->payload[length+1] =((uint8_t)(msg->crc16 >> 8) & 0xFF);
	msg->payload[length] = 	((uint8_t)msg->crc16 & 0xFF);
	
	return msg->length;
}

/**
* @brief gs_can_parse
* @param  canid: Specific canID to parse
* @param  msg: packet after parse data from can system
* @param  state: pointer to data
* @return True/false
*/
uint8_t gs_can_parse(uint32_t canid, gs_can_message_t *msg, gs_can_status_t* state)
{
//	if(canid == state->value.StdId)
//	{
		uint8_t i = 0;
		
		for(i = 0; i < state->value.DLC; i++)
		{
			uint8_t c = state->value.Data[i];
			
			if(state->status == GS_CAN_PARSE_STATE_IDLE)
			{
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_DELIMITER)
			{
				if(c == GS_CAN_DELIMITER){
					state->status++;
					msg->delimiter = c;
					msg->length = 0;
					state->status = GS_CAN_PARSE_STATE_LEN;
					
//					gConsole.fWrite("stx\n");
				}
			}
			else if(state->status == GS_CAN_PARSE_STATE_LEN)
			{
				if(c > GS_CAN_MAX_PAYLOAD_LEN)
				{
					state->status = GS_CAN_PARSE_STATE_DELIMITER;
				}
				else
				{
//					gConsole.fWrite("len\n");
					msg->length 				= c;
					state->zPrivate.packetIdx 	= 0;
					state->status++;
				}
			}
			else if(state->status == GS_CAN_PARSE_STATE_VERSION)
			{
//				gConsole.fWrite("ver\n");
				msg->protocol_version = c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_CRC8)
			{
				msg->crc8 = gs_can_crc_8_init();
				msg->crc8 = gs_can_crc_8_update(msg->crc8, &msg->delimiter, GS_CAN_HEADER_STX2VER_LEN);
				msg->crc8 = gs_can_crc_8_finalize(msg->crc8);
				
				/** Check CRC of packet is true*/
				if(msg->crc8 == c){
//					sprintf(CanbusBuffTest,"crc: %d, c: %d\n", msg->crc8, c);
//					gConsole.fWrite(CanbusBuffTest);
					state->status++;
				}
				else
				{
					state->status = GS_CAN_PARSE_STATE_DELIMITER;
//					gConsole.fWrite("crc8 error\n");
				}
			}
			else if(state->status == GS_CAN_PARSE_STATE_SENDER_ID)
			{
				//gConsole.fWrite("sender id\n");
				msg->senderId 			= c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_RECEIVER_ID)
			{
				//gConsole.fWrite("receiver\n");
				msg->receiverId 		= c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_SEQ1){
				//gConsole.fWrite("seq1\n");
				msg->seq = c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_SEQ2){
				//gConsole.fWrite("seq2\n");
				msg->seq += c*256;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_ACK)
			{
				/** DJI_P3_FLIGHT_CONTROL_UART_ENCRYPT_TYPE | DJI_P3_FLIGHT_CONTROL_UART_ACK_POLL*/
				msg->ack = c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_CMDSET)
			{
				msg->cmdSet 	= c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_CMD)
			{
				msg->cmd 	= c;
				state->status++;
			}
			else if(state->status == GS_CAN_PARSE_STATE_PAYLOAD)
			{
				msg->payload[state->zPrivate.packetIdx++] = c;
				
				if(state->zPrivate.packetIdx >= (msg->length - GS_CAN_SYSTEM_LEN)){
//					//gConsole.fWrite("pay\n");
					state->status++;
				}
			}
			else if(state->status == GS_CAN_PARSE_STATE_CRC16_1)
			{
				state->zPrivate.crc16 = c;
				
				msg->crc16 = gs_can_crc_16_init();
				msg->crc16 = gs_can_crc_16_update(msg->crc16, &msg->delimiter, msg->length - GS_CAN_FOOTER_CRC_LEN);
				msg->crc16 = gs_can_crc_16_finalize(msg->crc16);
				
				state->status++;
			}

			else if(state->status == GS_CAN_PARSE_STATE_CRC16_2){
				state->status = GS_CAN_PARSE_STATE_DELIMITER;
				
				state->zPrivate.crc16 += c*256;
				
//				sprintf(CanbusBuffTest,"c2: %x, c: %x\n", msg->crc16, state->zPrivate.crc16);
//				gConsole.fWrite(CanbusBuffTest);
				
				if(state->zPrivate.crc16 == msg->crc16)
				{
					return 1;
				}
			}
			else
			{
				state->status = GS_CAN_PARSE_STATE_IDLE;
			}
		}
//	}
	
	return 0;
}
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.********END OF FILE*******/
