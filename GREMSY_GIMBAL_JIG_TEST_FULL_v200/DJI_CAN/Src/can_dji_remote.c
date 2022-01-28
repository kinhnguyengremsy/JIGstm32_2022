/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file   	can_dji_remote.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji protocol
 *
 ******************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_remote.h"
#include "can_dji.h"

//#include "gGlobalData.h"

#include "string.h"
/* Private define-------------------------------------------------------------*/
#define CAN_DJI_CMD_REMOTE_BUTTON			80
#define CAN_DJI_CMD_REMOTE_BUTTON_LEN		19

#define REMOTE_DJI_CMD_NBR		(sizeof(remote_cmd_list)/sizeof(remote_cmd_list[0]))

/* Private Typedef------------------------------------------------------------*/
/**/
/*
 * camera_command_handle_t
 */
typedef struct __can_dji_cmd_t
{
	const char*			name;
	uint8_t				cmd;
	void				(*func)(gs_can_message_t* in, can_dji_remote_t* out);
}can_dji_cmd_t;

#define ADD_CAN_DJI_CMD(c_name, c_cmd, c_func) \
					{ \
						.name 		= c_name,\
						.cmd		= c_cmd,\
						.func		= c_func\
					}

/* Exported Global variables------------------------------------------------- */
/* Private function- ---------------------------------------------------------*/
void debug_printf_remote(gs_can_message_t *in);

static void can_dji_cmd_remote_button(gs_can_message_t* in, can_dji_remote_t *rc);

/* Private variable- ---------------------------------------------------------*/
static const can_dji_cmd_t	remote_cmd_list[] = 
{
	ADD_CAN_DJI_CMD("remote_button",	CAN_DJI_CMD_REMOTE_BUTTON, can_dji_cmd_remote_button)
};

/* Exported functions --------------------------------------------------------*/
/**
* @brief @brief can_handle_gimbal
* @param  msg: pointer to a packet after parsing data the following protocol
* @param  gimbal: pointer to buffer which stores data relate to gimbal
* @return None
*/
void can_handle_remote_button(gs_can_message_t* msg, can_dji_remote_t *rc)
{
    uint16_t	scan_idx = 0;
    uint8_t		cam_received = 0x00;

    /**Get camera command */
    cam_received = msg->cmd;

    //	debug_printf_1(msg);

    /* TODO: optimize the search method */
    /* Scan the matching command in task list */
    for (scan_idx = 0; scan_idx < REMOTE_DJI_CMD_NBR; scan_idx++)
    {
        if(remote_cmd_list[scan_idx].cmd == cam_received)
        {
            /**Execute function correspond with the command from can system*/
            (*remote_cmd_list[scan_idx].func)(msg, rc);
        }
    }
}


static void can_dji_cmd_remote_button(gs_can_message_t* in, can_dji_remote_t *rc)
{
    uint8_t		cal_crc8 = 0x00;

    /** Calculate crc8 */
    cal_crc8 = gs_can_crc8(CAN_DJI_CMD_REMOTE_BUTTON_LEN);

    /** Check crc8 */
    if(in->crc8 == cal_crc8)
    {
        if(in->payload[0] == RC_BUT_C1)
        {
            rc->but.c1_but = RC_BUT_C1;
        }
        else if(in->payload[0] == RC_BUT_C2)
        {
            rc->but.c2_but = RC_BUT_C2;
        }
        else if(in->payload[0] == RC_BUT_RECORD)
        {
            rc->but.record_but = RC_BUT_RECORD;
        }
        else if(in->payload[0] == RC_BUT_PLAYBACK)
        {
            rc->but.playback_but = RC_BUT_PLAYBACK;
        }
        else if(in->payload[0] == RC_BUT_SHUTTER)
        {
            rc->but.shutter_but = RC_BUT_SHUTTER;
        }
        else
        {
            memset(&rc->but, 0x00, sizeof(rc->but));
        }

        /** Button Switch mode */
        rc->switch_mode = in->payload[2];
        
        if(in->payload[4] == RC_BUT_CAM_DIAL)
        {
            rc->dial_but = RC_BUT_CAM_DIAL;
        }
        else 
        {
            rc->dial_but = 0;
        }
    }
}
uint16_t can_dji_cmd_remote_control_gimbal_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, uint8_t remoteButton[8])
{
    uint8_t len = 0;
    
    len = CAN_DJI_CMD_REMOTE_BUTTON_LEN - GS_CAN_SYSTEM_LEN;
    gs_can_coppy(msg->payload, remoteButton, len);
    
    return gs_can_msg_pack(msg, state,  senderId, receiverId, len, 6, 80);
}
void debug_printf_remote(gs_can_message_t *in)
{
    printf("l:%d, c:%d, s:%d, r:%d, ,a:%d, set:%d, cmd:%d,	%d,	%d,	%d,	%d,	%d,	%d,	%d,	%d	%d\n",
        in->length, in->crc8, in->senderId, in->receiverId, in->ack, in->cmdSet, in->cmd,
        in->payload[0], in->payload[1], in->payload[2], in->payload[3], in->payload[4],
        in->payload[5], in->payload[6], in->payload[7], in->payload[8]
    );
}

/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.********END OF FILE*******/
