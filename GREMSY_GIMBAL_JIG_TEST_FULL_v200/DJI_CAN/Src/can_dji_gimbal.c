/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 * 
 * @file    gs_can_gimbal.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    May-12-2018
 * @brief   This file contains expand of the can dji protocol
 *
 ******************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_gimbal.h"
#include "can_dji.h"

#include "string.h"
/* Private define-------------------------------------------------------------*/
/*
 *Gimbal command is set from App **********************************************
 */

/*C: Send:02, r:04, cmdSet:04, cmd:0F, len:10, crc:56  0, 1,  1,  63, 112,    63, 158,    59  122*/
/**
 [0] = 0 configuration 
 [0] = 11 enable synchronized gimbal pan follow 
 [1] = 1 default 
 [2] = 0 1 2 of the configuration tabs
*/
#define CAN_DJI_CMD_ENABLE_PAN_FOLLOW			0x0F
#define CAN_DJI_CMD_ENABLE_PAN_FOLLOW_LEN		0x10

/*C: Send:02, r:04, cmdSet:04, cmd:10, len:10, crc:56  25,    34, 35, 63, 153,    203,    172,    59  177
       Message after switch from advanced settings to gimbal settings
       Send every 1s 
    */
#define CAN_DJI_CMD_GIMBAL_SETTINGS						0x10
#define CAN_DJI_CMD_GIMBAL_SETTINGS_LEN					0x10

/*C: Send:09, r:04, cmdSet:01, cmd:01, len:17, crc:38  0, 0,  0,  0,  0,  0,  1,  0   0
 * Command to specific mode of gimbal
*/
#define CAN_DJI_CMD_SPECIAL							0x01
#define CAN_DJI_CMD_SPECIAL_LEN						0x17

/*C: Send:02, r:04, cmdSet:04, cmd:0F, len:11, crc:92  8, 2,  26, 0,  78, 146,    182,    59  80*/
#define CAN_DJI_CMD_GIMBAL_ADVANCED_SETTINGS		15
#define CAN_DJI_CMD_GIMBAL_ADVANCED_SETTINGS_LEN	17

/*C: Send:02, r:04, cmdSet:04, cmd:07, len:0E, crc:66  1, 0,  0,  1,  23, 0,  4,  1   234*/
#define CAN_DJI_CMD_GIMBAL_ADJUST					7
#define CAN_DJI_CMD_GIMBAL_ADJUST_LEN				14

/*C: Send:09, r:04, cmdSet:04, cmd:01, len:15, crc:A9  0, 4,  0,  4,  0,  4,  0,  4   165*/
#define CAN_DJI_CMD_GIMBAL_RC					1
/*C: Send:09, r:04, cmdSet:04, cmd:06, len:15, crc:A9  0, 4,  0,  4,  0,  4,  0,  4   230*/
#define CAN_DJI_CMD_FC_RC						6
#define CAN_DJI_CMD_GIMBAL_RC_LEN				21

/*
 *Gimbal command is set from App **********************************************
 */

/**G: Send:4, r:2, cmdSet:4, cmd:5, len:25, crc:228*/
#define CAN_DJI_CMD_GIMBAL_PARAMS				0x05
#define CAN_DJI_CMD_GIMBAL_PARAMS_LEN			25

/**l:16, c:86, s:2, r:4, ,a:64, set:4, cmd:16*/
#define CAN_DJI_CMD_GIMBAL_TEST2           0x10
#define CAN_DJI_CMD_GIMBAL_TEST2_LEN       0x10

/**
    [0] = 0
    [1] = 1
    [2] = configuration 1 | 2 | 3
    switch from configuration 1 | 2 | 3 to gimbal setting tab
    [0] = 25
    [1] = 34
    [2] = 35
**/
//static void can_dji_cmd_gimbal_get2(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/** l:16, c:86, s:2, r:4, ,a:64, set:4, cmd:15
*/
#define CAN_DJI_CMD_GIMBAL_TEST3           0x0F
#define CAN_DJI_CMD_GIMBAL_TEST3_LEN       0x10

/**
    [0] = 11
    [1] = 1
    [2] = 0 | 1 (enale synchronized gimbal Pan follow)
**/
//static void can_dji_cmd_gimbal_get3(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);


/** Gimbal confirm that applied for setting from app of the Advanced settings (speed, smooth, ...)

*/
#define CAN_DJI_CMD_GIMBAL_SET1           0x0F
#define CAN_DJI_CMD_GIMBAL_SET1_LEN       0x0E

/**
    Gimbal confirm that applied for setting from app of the Advanced settings
**/
//static void can_dji_cmd_gimbal_set1(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);


/**C: Send:04, r:02, cmdSet:04, cmd:10, len:0E, crc:66  227  34  35  4 
    Message after switch from advanced settings to gimbal settings
    Send every 1s 
*/
#define CAN_DJI_CMD_GIMBAL_SET2           0x10
#define CAN_DJI_CMD_GIMBAL_SET2_LEN       0x0E

/**
    Maybe send data all of the advanced settings to gimbal every time switch from configuration 1 2 3 
*/
#define CAN_DJI_CMD_GIMBAL_SET3           0x10
#define CAN_DJI_CMD_GIMBAL_SET3_LEN       0x27

/*Advanced setting reset settings is accept
 [0] = 4
*/
#define CAN_DJI_CMD_GIMBAL_SET4           0x13
#define CAN_DJI_CMD_GIMBAL_SET4_LEN       0x0E

#define CAN_DJI_CMD_GIMBAL_TYPE             0x1C
#define CAN_DJI_CMD_GIMBAL_TYPE_LEN         0x0E

#define GIMBAL_DJI_CMD_GET_NBR		(sizeof(gimbal_cmd_get)/sizeof(gimbal_cmd_get[0]))
#define GIMBAL_DJI_CMD_SET_NBR		(sizeof(gimbal_cmd_set)/sizeof(gimbal_cmd_set[0]))

#define GIMBAL_DEBUG 



/**20/7/2018 Test command joystick*/
/**C: Send:09, r:04, cmdSet:09, cmd:03, len:12, crc:C7
*/

#define CAN_DJI_CMD_GIMBAL_DIAL           0x03
#define CAN_DJI_CMD_GIMBAL_DIAL_LEN       0x12
//static void can_dji_cmd_gimbal_dial(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/* Private Typedef------------------------------------------------------------*/
extern can_dji_gimbal_get_t gimbal;
/*
 * camera_command_handle_t
 */
typedef struct __can_dji_cmd_t
{
	const char*			name;
	uint8_t				cmd;
	void				(*func)(gs_can_message_t* in, can_dji_gimbal_get_t* out);
}can_dji_cmd_t;

#define ADD_CAN_DJI_CMD_GET(c_name, c_cmd, c_func) \
					{ \
						.name 		= c_name,\
						.cmd		= c_cmd,\
						.func		= c_func\
					}

/*
 * camera_command_transmit_t
 */
typedef struct __can_dji_cmd_set_t
{
	const char*			name;
    uint8_t             send_id;
    uint8_t             receive_id;
    uint8_t             cmd_set;
	uint8_t				cmd;
    uint8_t             hz;
	void				(*func)(gs_can_message_t* in, can_dji_gimbal_set_t* out);
}can_dji_cmd_set_t;

#define ADD_CAN_DJI_CMD_SET(c_name, c_send, c_receive, c_cmd_set, c_cmd, c_hz, c_func) \
					{ \
						.name 		= c_name,\
                        .send_id    = c_send,\
                        .receive_id = c_receive,\
                        .cmd_set    = c_cmd_set,\
						.cmd		= c_cmd,\
                        .hz         = c_hz,\
						.func		= c_func\
					}
/* Exported Global variables------------------------------------------------- */
/* Private function- ---------------------------------------------------------*/
/**
 * Gimbal get data from system
*/
/** G: Send:2, cmdSet:4, cmd:15, len:17, crc:146*/
//static void can_dji_cmd_gimbal_advanced_settings(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/** function handle cmd from hd link G: Send:9, cmdSet:4, cmd:1, len:21, crc:169 */
static void can_dji_cmd_gimbal_rc(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/**G: Send:2, cmdSet:4, cmd:7, len:14, crc:102 +-0.1*/
//static void can_dji_cmd_gimbal_adjust(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/** Function handle cmd set special
 * G: Send:9, cmdSet:1, cmd:1, len:23, crc:56
 */
//static void can_dji_cmd_special_setting(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

/**    
 * @brief  This is command to enable pan follow or switch configuration tabs
 * @return none
 */
//static void can_dji_cmd_enable_pan_follow(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal);

void debug_printf(gs_can_message_t *in);
void debug_printf_1(gs_can_message_t *in);

/* Private variable- ---------------------------------------------------------*/
/**
 * list of command send to gimbal
 */
static const can_dji_cmd_t	gimbal_cmd_get[] = 
{
    /** Command setting to gimbal */
//    ADD_CAN_DJI_CMD_GET("gimbal_advanced_settings",	CAN_DJI_CMD_GIMBAL_ADVANCED_SETTINGS, can_dji_cmd_gimbal_advanced_settings),
//    
//    /* Control gimbal move */
    ADD_CAN_DJI_CMD_GET("gimbal_rc", CAN_DJI_CMD_GIMBAL_RC, can_dji_cmd_gimbal_rc),
    
    /**Adjust roll angles */
//    ADD_CAN_DJI_CMD_GET("gimbal_adjust_roll", CAN_DJI_CMD_GIMBAL_ADJUST, can_dji_cmd_gimbal_adjust),
    
    /**Gimbal mode */
//    ADD_CAN_DJI_CMD_GET("gimbal_mode", CAN_DJI_CMD_SPECIAL, can_dji_cmd_special_setting),
    
    /** Gimbal settings*/
//    ADD_CAN_DJI_CMD_GET("enable_pan_follow", CAN_DJI_CMD_ENABLE_PAN_FOLLOW, can_dji_cmd_enable_pan_follow),
//    
//    ADD_CAN_DJI_CMD_GET("can_dji_cmd_gimbal_dial", CAN_DJI_CMD_GIMBAL_DIAL, can_dji_cmd_gimbal_dial),
};

/* Exported functions --------------------------------------------------------*/
/**
* @brief @brief can_gimbal_handle
* @param  msg: pointer to a packet after parsing data the following protocol
* @param  gimbal: pointer to buffer which stores data relate to gimbal
* @return None
*/
void can_gimbal_handle(gs_can_message_t *msg, can_dji_gimbal_get_t *gimbal)
{
    uint16_t	scan_idx = 0;
    uint8_t		cam_received = 0x00;

    /**Get camera command */
    cam_received = msg->cmd;

    /** Command send to gimbal */
    if(msg->receiverId == SRC_DEST_GIMBAL)
    {
        if(msg->senderId == SRC_DEST_APP || msg->senderId == SRC_DEST_HD_LINK)
        {
            /* TODO: optimize the search method */
            /* Scan the matching command in task list */
            for (scan_idx = 0; scan_idx < GIMBAL_DJI_CMD_GET_NBR; scan_idx++)
            {
                if(gimbal_cmd_get[scan_idx].cmd == cam_received)
                {
                    /**Execute function correspond with the command from can system*/
                    (*gimbal_cmd_get[scan_idx].func)(msg, gimbal);
                }
            }
        }
    }
        printf("C: Send:%02X, r:%02X, cmdSet:%02X, cmd:%02X, len:%02X, crc:%02X %02X %02X %02X %02X \n",msg->senderId, msg->receiverId, msg->cmdSet, msg->cmd, msg->length, msg->crc8, msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3]);
}

/* Private function- ---------------------------------------------------------*/
/**
* @brief @brief can_dji_cmd_gimbal_advanced_settings
* @param  in: pointer to a packet after parsing data the following protocol
* @param  gimbal: pointer to buffer which stores data relate to gimbal
* @return None
*/
//static void can_dji_cmd_gimbal_advanced_settings(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
//{
//    uint8_t		cal_crc8 = 0x00;

//    /** Calculate crc8 */
//    cal_crc8 = gs_can_crc8(CAN_DJI_CMD_GIMBAL_ADVANCED_SETTINGS_LEN);

//    /** Check crc8 */
//    if(in->crc8 == cal_crc8)
//    {
//        /**Selecting configuration */
//        if(in->payload[1] == TYPE_CHECKBOX)
//        {
//            if(in->payload[0] == 0)
//            {
//                gimbal->advanced_settings.config_select = in->payload[2];
//            }
//            else if(in->payload[0] == 11)
//            {
//                gimbal->advanced_settings.enable_pan_follow = in->payload[2];
//            }
//        }
//        else if(in->payload[1] == TYPE_SLIDER)
//        {
//            if(in->payload[0] == TYPE_YAW_SPEED)
//            {
//                gimbal->advanced_settings.yaw_speed = in->payload[2];
//            }
//            else if(in->payload[0] == TYPE_PITCH_SPEED)
//            {
//                gimbal->advanced_settings.pitch_speed= in->payload[2];
//            }
//            else if(in->payload[0] == TYPE_YAW_SMOOTHNESS)
//            {
//                gimbal->advanced_settings.yaw_smoothness = in->payload[2];
//            }
//            else if(in->payload[0] == TYPE_PITCH_SMOOTHNESS)
//            {
//                gimbal->advanced_settings.pitch_smoothness = in->payload[2];
//            }
//        }
//        
//        printf("config:%d, p_s:%d, y_s:%d, y_sm:%d, p_sm:%d, en:%d\n",
//                gimbal->advanced_settings.config_select,
//                gimbal->advanced_settings.pitch_speed,
//                gimbal->advanced_settings.yaw_speed,
//                gimbal->advanced_settings.yaw_smoothness,
//                gimbal->advanced_settings.pitch_smoothness,
//                gimbal->advanced_settings.enable_pan_follow);
//    }
//}

/**
* @brief @brief can_dji_cmd_gimbal_adjust
* @param  in: pointer to a packet after parsing data the following protocol
* @param  gimbal: pointer to buffer which stores data relate to gimbal
* @return None
*/
//static void can_dji_cmd_gimbal_adjust(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
//{
//	uint8_t		cal_crc8 = 0x00;
//	
//	/** Calculate crc8 */
//	cal_crc8 = gs_can_crc8(CAN_DJI_CMD_GIMBAL_ADJUST_LEN);
//	
//	if(in->crc8 == cal_crc8)
//	{
//		/** Get value roll adjust*/
//		gimbal->roll_adjust = in->payload[0];
//		
////		printf("roll_adj:%d\n", gimbal->roll_adjust);
//	}
//}

static void can_dji_cmd_gimbal_rc(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
{
	uint8_t		cal_crc8 = 0x00;
	
	/** Calculate crc8 */
	cal_crc8 = gs_can_crc8(CAN_DJI_CMD_GIMBAL_RC_LEN);
	
	/**Check CRC8*/
	if(in->crc8 == cal_crc8)
	{
		gimbal->rc.channel[LB2_RC_TILT]	= (*(int16_t*)&in->payload[0]);
		gimbal->rc.channel[LB2_RC_ROLL]	= (*(int16_t*)&in->payload[2]);
		gimbal->rc.channel[LB2_RC_PAN]	= (*(int16_t*)&in->payload[4]);
		printf("t:%d,   	r:%d,   	p:%d\n",
				gimbal->rc.channel[LB2_RC_TILT],
				gimbal->rc.channel[LB2_RC_ROLL],
				gimbal->rc.channel[LB2_RC_PAN]);

		printf("1:%d\t	2:%d\t	3:%d\t	4:%d\t	5:%d\t	6:%d\t	7:%d\t	8:%d\n", 
        in->payload[0], in->payload[1], in->payload[2], in->payload[3], in->payload[4],
        in->payload[5], in->payload[6], in->payload[7]);

	}
}

//static void can_dji_cmd_special_setting(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
//{
//    uint8_t		cal_crc8	= 0x00;
//    uint8_t		subtract	= 0;
//    uint8_t		flag		= 0;

//    /** Calculate crc8 */
//    cal_crc8 = gs_can_crc8(CAN_DJI_CMD_SPECIAL_LEN);

//    /** Check CRC8*/
//    if(in->crc8 == cal_crc8)
//    {
//        /**Calculate value to detect the app */
//        subtract = in->payload[6] - in->payload[5] - in->payload[4];
//        flag = in->payload[9];
//        
//        if(subtract == flag )
//        {
//            if(in->payload[5] == IS_DETECT_APP)
//            {
//                gimbal->gimbal_mode = (gimbal_mode_t)in->payload[4];
//                
//    //				printf("mode:%d \n", gimbal->gimbal_mode);
//            }
//        }
//        
//    }
//}

/**    
 * @brief  This is command to enable pan follow or switch configuration tabs
 * @return none
 */
//static void can_dji_cmd_enable_pan_follow(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
//{
//    uint8_t		cal_crc8 = 0x00;

//    /** Calculate crc8 */
//    cal_crc8 = gs_can_crc8(CAN_DJI_CMD_ENABLE_PAN_FOLLOW);

//    /**Check CRC8*/
//    if(in->crc8 == cal_crc8)
//    {
//        /** This byte always is one*/
//        if(in->payload[1] == 1)
//        {
//            if(in->payload[0] == 0)
//            {
//                gimbal->gimbal_settings.detect.switch_tabs = in->payload[2];
//            }
//            else if(in->payload[0] == 11)
//            {
//                gimbal->gimbal_settings.detect.is_enable_pan_follow = in->payload[2];
//            }
//        }
//    }
//}
/*
 * Gimbal send data to app **************************************************
*/
void can_dji_cmd_params_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, gimbal_params_t *gimbal_params)
{
    /** coppy data to payload*/
    memcpy(msg->payload, gimbal_params, CAN_DJI_CMD_GIMBAL_PARAMS_LEN);
   
   /** Packing protocol and data to send */
//   gs_can_msg_pack(msg, state, senderId, receiverId, CAN_DJI_CMD_GIMBAL_PARAMS_LEN); // chua biet cmdSet va cmd
}

uint16_t can_dji_cmd_gimbal_type_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, uint8_t type)
{
    uint8_t len = 0;
    
    len = CAN_DJI_CMD_GIMBAL_TYPE_LEN - GS_CAN_SYSTEM_LEN;
    gs_can_coppy(msg->payload, &type, len);
	
//	return gs_can_msg_pack(msg, state,  senderId, receiverId, len);// chua biet cmdSet va cmd
}

uint16_t can_dji_cmd_gimbal_control_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, int16_t rc_channel[3])
{
    uint8_t len = 0;
    
    len = CAN_DJI_CMD_GIMBAL_RC_LEN - GS_CAN_SYSTEM_LEN; // GIMBAL_RC  
    gs_can_coppy(msg->payload, rc_channel, len);
	
	return gs_can_msg_pack(msg, state,  senderId, receiverId, len, 4, 1);
}
//static void can_dji_cmd_gimbal_dial(gs_can_message_t *in, can_dji_gimbal_get_t *gimbal)
//{
//    uint8_t		cal_crc8 = 0x00;

//    /** Calculate crc8 */
//    cal_crc8 = gs_can_crc8(CAN_DJI_CMD_GIMBAL_DIAL_LEN);

//    /**Check CRC8*/
//    if(in->crc8 == cal_crc8)
//    {
//		printf("1:%d,   2:%d,   3:%d,   4:%d,   5:%d\n", in->payload[0], in->payload[1], in->payload[2], in->payload[3], in->payload[4]);
//    }
//}


void debug_printf(gs_can_message_t *in)
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

void debug_printf_1(gs_can_message_t *in)
{
	printf("l:%d, c:%d, s:%d, r:%d, ,a:%d, set:%d, cmd:%d,	%d,	%d,	%d,	%d,	%d,	%d,	%d,	%d	%d\n",
		in->length, in->crc8, in->senderId, in->receiverId, in->ack, in->cmdSet, in->cmd,
		in->payload[0], in->payload[1], in->payload[2], in->payload[3], in->payload[4],
		in->payload[5], in->payload[6], in->payload[7], in->payload[8]);
}

/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.********END OF FILE*******/
