/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file    gs_can_camera.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji protocol
 *
 ******************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_camera.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "can_dji.h"

//#include "gGlobalData.h"
/* Private define-------------------------------------------------------------*/

#define CAN_DJI_CMD_CAMERA_SHUTTER			0x7c
#define CAN_DJI_CMD_CAMERA_PLAYBACK_PARAMS	0x82

#define CAN_DJI_CMD_CAMERA_SHUTTER_CMD          0X7C
#define CAN_DJI_CMD_CAMERA_STATE_INFO           0X80
#define CAN_DJI_CMD_CAMERA_SHOT_PARAMS          0X81
#define CAN_DJI_CMD_CAMERA_PLAY_BACK_PARAMS     0X82
#define CAN_DJI_CMD_CAMERA_CHART_INFO           0X83
#define CAN_DJI_CMD_CAMERA_RECORDING_NAME       0X84
#define CAN_DJI_CMD_CAMERA_RAW_PARAMS           0X85
#define CAN_DJI_CMD_CAMERA_CUR_PANO_FILE_NAME   0X86
#define CAN_DJI_CMD_CAMERA_SHOT_INFO            0X87
#define CAN_DJI_CMD_CAMERA_TIMELAPSE_PARMS      0X88
#define CAN_DJI_CMD_CAMERA_TRACKING_STATUS      0X89
#define CAN_DJI_CMD_CAMERA_FOV_PARAM            0X8A
#define CAN_DJI_CMD_CAMERA_PREPARE_OPEN_FAN     0XB4
#define CAN_DJI_CMD_CAMERA_OPTICS_ZOOM_MODE     0XB8
#define CAN_DJI_CMD_CAMERA_TAP_ZOOM_STATE_INFO  0XC7
#define CAN_DJI_CMD_CAMERA_TAU_PARAM            0XF2

/** switch mode auto, S, M*/
#define CAN_DJI_CMD_CAMERA_MODE				0x1E
/**Camera state infor 0x80 */
/** ISO CMD */
#define CAM_DJI_CMD_ISO						42
/** Photo cmd*/
#define CAM_DJI_CMD_PHOTO					106
/** Cmd shutter */
#define CAM_DJI_CMD_SHUTTER					40

#define CAM_DJI_CMD_SHOT_PARAMS				0x81
#define CAM_DJI_CMD_SHOT_PARAMS_LEN			104



#define CAM_DJI_CMD_NBR		(sizeof(cam_cmd_list)/sizeof(cam_cmd_list[0]))

char CanbusBuffTest[255] = {0};
/* Private Typedef------------------------------------------------------------*/

/**/
/*
 * camera_command_handle_t
 */
typedef struct __can_dji_camera_cmd_t
{
	const char*			name;
	uint8_t				cmd;
	void				(*func)(gs_can_message_t* in, gs_can_dji_cam_t* out);
}can_dji_camera_cmd_t;

#define ADD_CAM_CMD(c_name, c_cmd, c_func) \
					{ \
						.name 		= c_name,\
						.cmd		= c_cmd,\
						.func		= c_func\
					}

/* Exported Global variables------------------------------------------------- */

/* Private function- ---------------------------------------------------------*/
//static void can_dji_cam_cmd_iso(uint8_t* payload, void *cam_iso);
static void can_dji_cam_cmd_shutter(gs_can_message_t *in, gs_can_dji_cam_t *cam);
static void can_dji_cam_cmd_shot_params(gs_can_message_t *in, gs_can_dji_cam_t *cam);


void debug_printf_camera(gs_can_message_t *in);

/* Private variable- ---------------------------------------------------------*/
/**********************CCB COMMAND HANDLE**************************************/
static const can_dji_camera_cmd_t cam_cmd_list [] = 
{
	ADD_CAM_CMD("cmd_camera_shutter", CAM_DJI_CMD_SHUTTER, can_dji_cam_cmd_shutter),
	ADD_CAM_CMD("cmd_shot params", CAM_DJI_CMD_SHOT_PARAMS, can_dji_cam_cmd_shot_params),
};


/* Exported functions --------------------------------------------------------*/
/**
* @brief @brief can_dji_cam_cmd_iso
* @param  payload: pointer to a buffer contains data free after parsing header 
* @param  cam_iso: pointer to data camera iso
* @return None
*/
//static void can_dji_cam_cmd_iso(uint8_t* payload, void *data_out)
//{
//}

/**
* @brief @brief can_dji_cam_cmd_shutter
* @param  payload: pointer to a buffer contains data free after parsing header 
* @param  cam_iso: pointer to data camera iso
* @return None
*/
static void can_dji_cam_cmd_shutter(gs_can_message_t *in, gs_can_dji_cam_t *cam)
{
	uint8_t			len;
	
	/** Calculate length of data */
	len = in->length - GS_CAN_SYSTEM_LEN;
	
	/** Check CRC8 of the header */
	if(in->crc8 == gs_can_crc8(len))
	{
		float delta = (float)in->payload[3] / (float)10.0;

		if(delta > 1)
		{
			delta = delta / (float)10.0;
		}
	
		/** Get data */
		*(cam->cam_shutter) = (float)in->payload[1] + (float)((in->payload[2]&0x7F)*256) + delta;
		
		printf("1:%d, 2:%d, 3:%d, 4:%d, shutter:%3.3f\n",in->payload[0], in->payload[1],in->payload[2],in->payload[3], *(cam->cam_shutter));
	}
}

/* Public functions ----------------------------------------------------------*/
void gs_can_handle_camera(gs_can_message_t* msg, gs_can_dji_cam_t *cam)
{
    uint16_t	scan_idx = 0;
    uint8_t		cam_received;

    /**Get camera command */
    cam_received = msg->cmd;
    
    /* TODO: optimize the search method */
    /* Scan the matching command in task list */
    for (scan_idx = 0; scan_idx < CAM_DJI_CMD_NBR; scan_idx++)
    {
            if(cam_cmd_list[scan_idx].cmd == cam_received)
        {
            /**Execute function correspond with the command from can system*/
            (*cam_cmd_list[scan_idx].func)(msg, cam);
        }
    }
}

//void gs_can_handle_camera(gs_can_message_t* packet, gs_can_dji_cam_t *cam)
//{
//	if(packet->senderId == SRC_DEST_APP && packet->receiverId == SRC_DEST_CAMERA)
//	{
////		printf("Cam: cmdSet:%d, cmd:%d, len:%d\n", packet->cmdSet, packet->cmd, packet->length);
//		
//		if(packet->cmdSet == CAN_DJI_CMD_SET_CAMERA)
//		{
//				/**Decode camera mode auto, S, M*/
//			if(packet->cmd == CAN_DJI_CMD_CAMERA_MODE)
//			{
//				gs_can_camera_mode(packet->payload, cam->camera_mode);
//	//			printf("mode:%d, cmd:%d, len:%d\n", *cam->camera_mode, packet->cmd, packet->length);
//			}
//			
//			if(packet->cmd == CAN_DJI_CMD_CAMERA_SHOT_PARAMS)
//			{
//				gs_can_camera_shot_params_msg_decode(packet->payload, cam->shot_params);
//				
////				printf("mode:%d, cmd:%d, len:%d\n", *cam->camera_mode, packet->cmd, packet->length);
//			}
//			
//			if(packet->cmd == CAN_DJI_CMD_CAMERA_STATE_INFO)
//			{
//				gs_can_camera_camera_state_info_msg_decode(packet->payload, cam->cam_state_info);
//				
////				printf("pState:%d, cmd:%d, len:%d\n", cam->cam_state_info->photo_state, cam->cam_state_info->sd_card_state, cam->cam_state_info->sd_card_total_size);
//			}
//			
//			if(packet->cmd == CAM_DJI_CMD_ISO)
//			{
//				can_dji_cam_cmd_iso(packet->payload, cam->cam_iso);
//				printf("iso:%d\n",  *cam->cam_iso);
//			}
//			
//			if(packet->cmd == CAM_DJI_CMD_SHUTTER)
//			{
//			
//				float shutter;
//				float delta = (float)packet->payload[3]/10.0;
//				
//				uint8_t	min;
//				
//				if(delta > 1)
//				{
//					delta = delta / 10.0;
//				}
//				
//				shutter = (float)packet->payload[1] + (float)((packet->payload[2]&0x7F)*256) + delta;
//				
//				printf("1:%d, 2:%d, 3:%d, 4:%d, s:%3.3f\n",packet->payload[0], packet->payload[1],packet->payload[2],packet->payload[3], shutter);

//				if(packet->crc8 == can_dji_cam_cmd_shutter_crc8())
//				{
//					printf("crc8:%d\n",packet->crc8);
//				}
//				else
//				{
//					printf("crc8:%d, cal:%d\n",packet->crc8, can_dji_cam_cmd_shutter_crc8());
//				}
//			}
//			
//		}
//	}
//	else if(packet->senderId == SRC_DEST_APP && packet->receiverId == SRC_DEST_GIMABL)
//	{
//		printf("Gimbal: cmdSet:%d, cmd:%d, len:%d\n", packet->cmdSet, packet->cmd, packet->length);
//	}
//}

static void can_dji_cam_cmd_shot_params(gs_can_message_t *in, gs_can_dji_cam_t *cam)
{
    uint8_t		cal_crc8 = 0x00;

    /** Calculate crc8 */
    cal_crc8 = gs_can_crc8(CAM_DJI_CMD_SHOT_PARAMS_LEN);

    if(in->crc8 == cal_crc8)
    {
        gs_can_coppy(&cam->shot_params.aperture_size, &in->payload[0], CAM_DJI_CMD_SHOT_PARAMS_LEN - GS_CAN_SYSTEM_LEN);
        
            printf("aperture_size:%d\nuser_shutter:%d\nshutter_speed_decimal:%d\niso:%d\nexposure_compensation:%d\nctr_object_for_one:%d\nctr_object_for_two:%d\nimage_size:%d\nimage_ratio:%d\nimage_quality:%d\nimage_format:%d\nvideo_format:%d\nvideo_fps:%d\nvideo_fov:%d\nvideo_second_open:%d\nvideo_second_ratio:%d\nvideo_quality:%d\nvideo_store_format:%d\nexposure_mode:%d\nscene_mode:%d\nmetering:%d\nwhite_balance:%d\ncolor_temp:%d\nmctf_enable:%d\nmctf_strength:%d\nsharpe:%d\ncontrast:%d\nsaturation:%d\ntonal:%d\ndigital_filter:%d\nanti_flicker:%d\ncontinuous:%d\ntime_params_type:%d\ntime_params_num:%d\ntime_params_period:%d\nreal_aperture_size:%d\nreal_shutter:%d\nrel_shutter_speed_decimal:%d\nrel_iso:%d\nrel_exposure_compensation:%d\ntime_countdown:%d\ncap_min_shutter:%d\ncap_min_shutter_decimal:%d\ncap_max_shutter:%d\ncap_max_shutter_decimal:%d\nvideo_standard:%d\nae_lock:%d\nphoto_type:%d\nspot_area_bottom_right_pos:%d\nunknown3b:%d\naeb_number:%d\npano_mode:%d\ncap_min_aperture:%d\ncap_max_aperture:%d\nauto_turn_off_fore_led:%d\nexposure_status:%d\nlocked_gimbal_when_shot:%d\nencode_types:%d\nnot_auto_ae_unlock:%d\nunknown47:%d\nconstrast_ehance:%d\nvideo_record_mode:%d\ntimelapse_save_type:%d\nvideo_record_interval_time:%d\ntimelapse_duration:%d\ntimelapse_time_count_down:%d\ntimelapse_recorded_frame:%d\noptics_scale:%d\ndigital_zoom_scale:%d\n",
            cam->shot_params.aperture_size,
            cam->shot_params.user_shutter,
            cam->shot_params.shutter_speed_decimal,
            cam->shot_params.iso,
            cam->shot_params.exposure_compensation,
            cam->shot_params.ctr_object_for_one,
            cam->shot_params.ctr_object_for_two,
            cam->shot_params.image_size,
            cam->shot_params.image_ratio,
            cam->shot_params.image_quality,
            cam->shot_params.image_format,
            cam->shot_params.video_format,
            cam->shot_params.video_fps,
            cam->shot_params.video_fov,
            cam->shot_params.video_second_open,
            cam->shot_params.video_second_ratio,
            cam->shot_params.video_quality,
            cam->shot_params.video_store_format,
            cam->shot_params.exposure_mode,
            cam->shot_params.scene_mode,
            cam->shot_params.metering,
            cam->shot_params.white_balance,
            cam->shot_params.color_temp,
            cam->shot_params.mctf_enable,
            cam->shot_params.mctf_strength,
            cam->shot_params.sharpe,
            cam->shot_params.contrast,
            cam->shot_params.saturation,
            cam->shot_params.tonal,
            cam->shot_params.digital_filter,
            cam->shot_params.anti_flicker,
            cam->shot_params.continuous,
            cam->shot_params.time_params_type,
            cam->shot_params.time_params_num,
            cam->shot_params.time_params_period,
            cam->shot_params.real_aperture_size,
            cam->shot_params.real_shutter,
            cam->shot_params.rel_shutter_speed_decimal,
            cam->shot_params.rel_iso,
            cam->shot_params.rel_exposure_compensation,
            cam->shot_params.time_countdown,
            cam->shot_params.cap_min_shutter,
            cam->shot_params.cap_min_shutter_decimal,
            cam->shot_params.cap_max_shutter,
            cam->shot_params.cap_max_shutter_decimal,
            cam->shot_params.video_standard,
            cam->shot_params.ae_lock,
            cam->shot_params.photo_type,
            cam->shot_params.spot_area_bottom_right_pos,
            cam->shot_params.unknown3b,
            cam->shot_params.aeb_number,
            cam->shot_params.pano_mode,
            cam->shot_params.cap_min_aperture,
            cam->shot_params.cap_max_aperture,
            cam->shot_params.auto_turn_off_fore_led,
            cam->shot_params.exposure_status,
            cam->shot_params.locked_gimbal_when_shot,
            cam->shot_params.encode_types,
            cam->shot_params.not_auto_ae_unlock,
            cam->shot_params.unknown47,
            cam->shot_params.constrast_ehance,
            cam->shot_params.video_record_mode,
            cam->shot_params.timelapse_save_type,
            cam->shot_params.video_record_interval_time,
            cam->shot_params.timelapse_duration,
            cam->shot_params.timelapse_time_count_down,
            cam->shot_params.timelapse_recorded_frame,
            cam->shot_params.optics_scale,
            cam->shot_params.digital_zoom_scale);
    }
}
void debug_printf_camera(gs_can_message_t *in)
{
	uint8_t	scan_idx = 0;
	uint8_t len = 0;
	
	char *str 		= malloc(1024*sizeof(char));
	char *str_byte	= malloc(4*sizeof(char));
	
	len = in->length - GS_CAN_SYSTEM_LEN;
	
	if(!str || !str_byte)
	{
		printf("Malloc Failed!\n");
		
		free(str);
		free(str_byte);
	}
	else
	{
		memset(str, 0, 1024);
		memset(str_byte, 0, 4);
		
		while(scan_idx < len)
		{
			sprintf(str_byte, "%d	", in->payload[scan_idx]);
			strcat(str, str_byte);
			memset(str_byte, 0, 4);
			scan_idx++;
		}
		int i = 0;
		for(i = 0; i < len; i++)
		{
			printf("%d ",str[i]);
		}
		
		printf("\n");
		
		free(str);
		free(str_byte);
	}
}

/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.********END OF FILE*******/
