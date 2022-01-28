/**
  ******************************************************************************
  * @file:   gs_can_gimbal.h 
  * @author: Gremsy Team
  * @version: v1.0
  * @date:    12/05/2018
  * @brief:    Decode can data of the camera 
  * @company: GREMSY Co., Ltd
  ******************************************************************************
  *
  *
*/

#ifndef __CAN_DJI_GIMBAL_H
#define __CAN_DJI_GIMBAL_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_protocol.h"
/* Private typedef -----------------------------------------------------------*/
/**
 * @brief - _gimbal_type_widget_t
 *	Gimbal widget on the advanced settings gimbal
 */
typedef enum _gimbal_type_widget_t
{
	TYPE_CHECKBOX = 1,
	TYPE_SLIDER	= 2
}gimbal_type_widget_t;

/**
 * @brief - gimbal_setting_t
 *	Gimbal setting params
 */
typedef enum _gimbal_setting
{
	TYPE_YAW_SPEED = 6,
	TYPE_PITCH_SPEED,
	TYPE_YAW_SMOOTHNESS,
	TYPE_PITCH_SMOOTHNESS
}gimbal_setting_t;

/**
 * @brief - _gimbal_mode_t
 *	Gimbal control mode
 */
typedef enum _gimbal_mode_t
{
	GIMBAL_YAW_RESET 	= 1,
	GIMBAL_MODE_FREE 	= 2,
	GIMBAL_MODE_FOLLOW 	= 10,
	GIMBAL_MODE_FPV 	= 6,
}gimbal_mode_t;

/**
 * @brief - _gimbal_mode_t
 *	Gimbal control mode
 */
typedef enum _gimbal_flag_t 
{
	IS_DETECT_APP	= 5,
	APP_FLAG_1		= 13,
	APP_FLAG_2		= 15
} gimbal_flag_t;

/**
 * @brief - remote_control_t
 *	LB2 remote control all of the channels
 */
typedef struct _remote_control_t
{
	uint8_t		flag_control;
	int16_t	    channel[17];
} remote_control_t;

/**
 * @brief - gimbal_advanced setting
 *	in tab advanced settings params to gimbal
 */
typedef struct _gimbal_advanced_settings
{
	uint8_t					pitch_speed;
	uint8_t					yaw_speed;
	uint8_t					yaw_smoothness;
	uint8_t					pitch_smoothness;
	uint8_t					enable_pan_follow;
	uint8_t					config_select;
} gimbal_advanced_settings;

/**
 * @brief - Gimbal - Gimbal Params - 0x05
 *	Gimbal send to app 
 */
typedef struct _gimbal_params
{
	int16_t pitch;	/**"0.1 degree, gimbal angular position, zero is forward, max down..up is about -900..470*/
	int16_t roll;	/** 0.1 degree, gimbal angular position, zero is parallel to earth, max right..left is about -410..410*/
	int16_t yaw;	/** 0.1 degree, gimbal angular position, -1000 is forward, max right..left is about -1460..-540*/
	union {
		uint8_t masked06;
		uint8_t sub_mode;
		uint8_t mode;
	} mode;
	
	int8_t roll_adjust;
	
	union {
		uint16_t yaw_angle;	/**Not sure whether Yaw angle or Joytick Direction*/
		uint16_t joystick_ver_direction;
		uint16_t joystick_hor_direction;
	} control;
	union {
		uint8_t masked0a;
		uint8_t pitch_in_limit;
		uint8_t roll_in_limit;
		uint8_t yaw_in_limit;
		uint8_t auto_calibration;
		uint8_t auto_calibration_result;	/**Auto Calibration Result*/
		uint8_t stuck;
	} settings;
	union {
		uint8_t masked0b;
		uint8_t version;
		uint8_t double_click;
		uint8_t triple_click;
		uint8_t single_click;
	} click;
}gimbal_params_t;

/**    
 * @brief  This is struct to store the data of the enable pan follow or configuration tabs
 */
 typedef struct _gimbal_settings
{
    union {
        uint8_t switch_tabs;
        uint8_t is_enable_pan_follow;
    } detect;
} gimbal_settings_t;
/**
 * @brief -- gimbal get data from system- structure
 *
 */
typedef struct _can_dji_gimbal_get_t
{
	/* Gimbal advanced settings */
	gimbal_advanced_settings	advanced_settings;
	
	/** remote control */
	remote_control_t			rc;
	
	/**Gimbal roll adjust */
	int8_t						roll_adjust;
	
	/** Gimbal mode*/
	gimbal_mode_t				gimbal_mode;
    
    gimbal_settings_t           gimbal_settings;
} can_dji_gimbal_get_t;


/**
 * @brief -- gimbal get data from system- structure
 *  -- Gimbal - Gimbal Type - 0x1c
 */


typedef enum _gimbal_type_t
{
    TIMEOUT = 0x00,
    FAULT   = 0x01,
    FC550   = 0x02,
    FC300SX = 0x03,
    FC260   = 0x04,
    FC350   = 0x05,
    FC350Z  = 0x06,
    Z15     = 0x07,
    P4      = 0x08,
    D5      = 0x0b,
    GH4     = 0x0c,
    A7      = 0x0d,
    BMPCC   = 0x0E,
    WM220   = 0x14,
    RONIN   = 0x0a,
    OTHER   = 0x64
} gimbal_type_t;
enum {
	
	LB2_RC_A = 0, 
	LB2_RC_E = 1, 
	LB2_RC_T = 2, 
	LB2_RC_R = 3, 
	LB2_RC_U = 6,
	LB2_RC_GEAR = 4,
	LB2_RC_HOME = 5,
	LB2_RC_IOC = 7, 
	LB2_RC_C1= 8, 
	LB2_RC_C2= 9,
	LB2_RC_RECORD = 10,
	LB2_RC_SHUTTER = 11,
	LB2_RC_PLAYBACK = 12, 
	LB2_RC_CAM_DIAL	= 13,
	LB2_RC_TILT	 	= 14,
	LB2_RC_PAN	 	= 15,
	LB2_RC_ROLL	 	= 16,
	LB2_RC_MAX
};
/**
 * @brief -- gimbal set data to system- structure
 *
 */
typedef struct _can_dji_gimbal_set_t
{
    gimbal_params_t         gimbal_params;
    
    uint8_t                 gimbal_type;
} can_dji_gimbal_set_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
* @brief @brief can_gimbal_handle
* @param  msg: pointer to a packet after parsing data the following protocol
* @param  gimbal: pointer to buffer which stores data relate to gimbal
* @return None
*/
void can_gimbal_handle(gs_can_message_t* msg, can_dji_gimbal_get_t *gimbal);

void can_dji_cmd_params_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, gimbal_params_t *gimbal_params);

/**
 * Gimbal set data to system
*/
/** function handle cmd set params is send from gimbal to app */
uint16_t can_dji_cmd_gimbal_type_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, uint8_t type);
uint16_t can_dji_cmd_gimbal_control_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, int16_t rc_channel[3]);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAN_DJI_GIMBAL_H */
/************************ GREMSY Co., Ltd *****END OF FILE****/
