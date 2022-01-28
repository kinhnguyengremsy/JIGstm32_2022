/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file    gs_can_dji.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji protocol
 *
 ******************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "can_dji.h"
/* Private define-------------------------------------------------------------*/

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
void can_dji_handle(gs_can_message_t* msg, can_dji_t *dji_system)
{
//    if(msg->cmdSet == CAN_DJI_CMD_SET_CAMERA)
//    {
//        printf("C: Send:%d, r:%d, cmdSet:%d, cmd:%d, len:%d, crc:%d, p0:%d\n",msg->senderId, msg->receiverId, msg->cmdSet, msg->cmd, msg->length, msg->crc8, msg->payload[0]);
//        gs_can_handle_camera(msg, &dji_system->cam);
//    }

//    printf("C: Send:%02X, r:%02X, cmdSet:%02X, cmd:%02X, len:%02X, crc:%02X\n",msg->senderId, msg->receiverId, msg->cmdSet, msg->cmd, msg->length, msg->crc8);

	if(msg->cmdSet == CAN_DJI_CMD_SET_GIMBAL || msg->cmdSet == CAN_DJI_CMD_SET_SPECIAL || msg->cmdSet == CAN_DJI_CMD_SET_GENERAL || msg->cmdSet == CAN_DJI_CMD_SET_HD_LINK)
	{
//		if(msg->length == 0x17)
//		{
//			printf("%d\t	%d\t	%d\t	%d\t	%d\t	%d\t	%d\t	%d	%d\t	%d\n",
//			msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3], msg->payload[4],
//			msg->payload[5], msg->payload[6], msg->payload[7], msg->payload[8], msg->payload[9]);
//		}
            
		can_gimbal_handle(msg, &dji_system->gimbal);
	}

	if(msg->cmdSet  == CAN_DJI_CMD_SET_REMOTE_CONTROL)
	{
		can_handle_remote_button(msg, &dji_system->rc_but);
	}

}
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.********END OF FILE*******/
