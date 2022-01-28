/**
  ******************************************************************************
  * @file:   gs_can_remote.h 
  * @author: Gremsy Team
  * @version: v1.0
  * @date:    12/05/2018
  * @brief:    Decode can data of the camera 
  * @company: GREMSY Co., Ltd
  ******************************************************************************
  *
  *
*/

#ifndef __CAN_DJI_REMOTE_H
#define __CAN_DJI_REMOTE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_protocol.h"
/* Private typedef -----------------------------------------------------------*/
/**
 * @brief - remote control button 
 *	
 */
typedef enum __can_dji_remote_button
{	
	RC_BUT_C1 		= 16, 
	RC_BUT_C2		= 32,
	RC_BUT_RECORD 	= 4,
	RC_BUT_SHUTTER 	= 2,
	RC_BUT_PLAYBACK = 8, 
	RC_BUT_CAM_DIAL	= 1
} can_dji_remote_button_t;

typedef enum _can_dji_remote_button_mode
{
	RC_BUT_MODE_A	= 0,
	RC_BUT_MODE_P	= 1,
	RC_BUT_MODE_F	= 2,
} can_dji_remote_button_mode_t;

/**
 * @brief -- remote button - structure
 *
 */
typedef struct __can_dji_remote_t
{
	union {
		uint8_t	c1_but;
		uint8_t	c2_but;
		uint8_t	playback_but;
		uint8_t	record_but;
		uint8_t	shutter_but;
	} but;
	
	uint8_t	switch_mode;
	
	uint8_t	dial_but;
	
} can_dji_remote_t;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void can_handle_remote_button(gs_can_message_t* msg, can_dji_remote_t *rc);
uint16_t can_dji_cmd_remote_control_gimbal_msg_pack(uint8_t senderId, uint8_t receiverId, gs_can_message_t* msg, gs_can_status_t* state, uint8_t remoteButton[8]);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAN_DJI_REMOTE_H */
/************************ GREMSY Co., Ltd *****END OF FILE****/
