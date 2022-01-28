/** 
  ******************************************************************************
  * @file    JIG_TEST_comm_raspberry_v2.h
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __JIG_TEST_COMM_RASPBERRY_V2_H
#define __JIG_TEST_COMM_RASPBERRY_V2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum _JIG_TEST_cloud_Data_send_command_status_t
{
    JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_NONE,
    JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_START,
    JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_STOP,
    JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_RESET,
    
}JIG_TEST_cloud_Data_send_command_status_t;

typedef enum _JIG_TEST_cloud_Data_send_command_login_t
{
    JIG_TEST_CLOUD_DATA_SEND_CMD_LOGIN_NONE,
    JIG_TEST_CLOUD_DATA_SEND_CMD_WAIT_LOGIN,
    JIG_TEST_CLOUD_DATA_SEND_CMD_LOGINED,
    
}JIG_TEST_cloud_Data_send_command_login_t;

typedef enum _JIG_TEST_cloud_Data_send_command_result_t
{
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_NONE,
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK,
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL,
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_BARCODE_READY,
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_TIMEOUT,
    JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_CANT_READ_UUID,
    
}JIG_TEST_cloud_Data_send_command_result_t;

typedef enum _JIG_TEST_send_to_cloud_Data_command_status_t
{
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_STANBY,
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_RUNNING,
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_TEST_OK,
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_TEST_FAIL,
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_RESET,
    JIG_TEST_SEND_TO_CLOUD_DATA_CMD_STATUS_GOT_RESULT,
    
}JIG_TEST_send_to_cloud_Data_command_status_t;

typedef enum _JIG_TEST_usb_speed_status_t
{
    JIG_TEST_USB_SPEED_STATUS_STANDBY,
    JIG_TEST_USB_SPEED_STATUS_RUNNING,
    JIG_TEST_USB_SPEED_STATUS_DONE,
    JIG_TEST_USB_SPEED_STATUS_RETRY,
    
}JIG_TEST_usb_speed_status_t;

typedef enum _JIG_TEST_usb_speed_result_t
{
    JIG_TEST_USB_SPEED_RESULT_NONE,
    JIG_TEST_USB_SPEED_RESULT_NO_USB,
    JIG_TEST_USB_SPEED_RESULT_LOW_SPEED,
    JIG_TEST_USB_SPEED_RESULT_PASS,
    
}JIG_TEST_usb_speed_result_t;

typedef enum _JIG_TEST_usb_speed_read_state_t
{
    JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_RESULT,
    JIG_TEST_USB_SPEED_READ_STATE_STATUS,
    JIG_TEST_USB_SPEED_READ_STATE_START,
    JIG_TEST_USB_SPEED_READ_STATE_END,
    JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED,
    
}JIG_TEST_usb_speed_read_state_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief comm_raspberry_v2_cloudData_configuration
    @return none
*/
void JIG_TEST_comm_raspberry_v2_cloudData_configuration(void);

/** @brief comm_raspberry_v2_cloudData_process
    @return none
*/
void JIG_TEST_comm_raspberry_v2_cloudData_process(void);

/** @brief usb_speed_process
    @return none
*/
void JIG_TEST_usb_speed_process(void);

/** @brief comm_raspberry_v2_heartbeat_timeOut_process
    @return none
*/
void JIG_TEST_comm_raspberry_v2_heartbeat_timeOut_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_COMM_RASPBERRY_V2_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

