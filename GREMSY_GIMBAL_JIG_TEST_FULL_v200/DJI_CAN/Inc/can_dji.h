/**
  ******************************************************************************
  * @file:   gs_can.h 
  * @author: Gremsy Team
  * @version: 
  * @date:    
  * @brief:   
  * @company: GREMSY Co., Ltd
  ******************************************************************************
  *
  *
*/

#ifndef __GS_CAN_H
#define __GS_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdlib.h"

#include "can_dji_camera.h"
#include "can_dji_gimbal.h"
#include "can_dji_remote.h"

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief can_dji_cmd_buffer_t
 */
typedef struct __can_dji_cmd_buffer {
	uint8_t		*buf;
	uint16_t	len;
	uint16_t	cmd_tid;          /** Command Transaction ID. Note it just use as template variable */
	uint8_t		cmd;
} can_dji_cmd_buffer_t;

/**
 * @brief can_dji_t
 */
typedef struct _can_dji_t
{
	gs_can_dji_cam_t		cam;
	can_dji_gimbal_get_t	gimbal;
	
	can_dji_remote_t		rc_but;
	
} can_dji_t;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
* @brief @brief can_dji_handle()
* @param  buff: pointer to a buffer contains data from src
* @param  buffSrc: pointer to a srouce buffer
* @param  count: number of bytes
* @return None
*/
void can_dji_handle(gs_can_message_t *msg, can_dji_t *dji_system);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GS_CAN_H */
/************************ GREMSY Co., Ltd *****END OF FILE****/
