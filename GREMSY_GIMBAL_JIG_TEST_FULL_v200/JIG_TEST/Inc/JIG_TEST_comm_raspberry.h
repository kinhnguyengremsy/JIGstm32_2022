/** 
  ******************************************************************************
  * @file    JIG_TEST_comm_raspberry.h
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

#ifndef __JIG_TEST_COMM_RASPBERRY_H
#define __JIG_TEST_COMM_RASPBERRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/
#define JIG_TEST_COMM_RASPBERRY_MAX_PARAM_INDEX 7
/* Exported types ------------------------------------------------------------*/
typedef enum _JIG_TEST_comm_raspberry_USB_speed_test_result_t
{
    JIG_TEST_USB_SPEED_RESULT_NONE,
    JIG_TEST_USB_SPEED_RESULT_NO_USB_FOUND,
    JIG_TEST_USB_SPEED_RESULT_LOW_SPEED,
    JIG_TEST_USB_SPEED_RESULT_PASSED,
    
}JIG_TEST_comm_raspberry_USB_speed_test_result_t;

typedef enum
{
    RASPBERRY_PUSH_CLOUD_RESULT_NONE    = 0x00,
    RASPBERRY_PUSH_CLOUD_RESULT_OK      = 0x01,
    RASPBERRY_PUSH_CLOUD_RESULT_FAIL    = 0x02,
    RASPBERRY_PUSH_CLOUD_BARCODE_READY  = 0x03,
    
}JIG_TEST_comm_raspberry_push_cloud_result;

typedef enum JIG_TEST_comm_raspberry_cloud_send_status_t
{
    JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE,
    JIG_TEST_CLOUD_DATA_SEND_STATUS_START,
    JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP,
    JIG_TEST_CLOUD_DATA_SEND_STATUS_RESET_SYSTEM,
    
}JIG_TEST_comm_raspberry_cloud_send_status_t;

typedef enum _JIG_TEST_comm_raspberry_cloud_login_t
{
    JIG_TEST_CLOUD_LOGIN_NONE,
    JIG_TEST_CLOUD_WAIT_FOR_LOGIN,
    JIG_TEST_CLOUD_LOGINED,
    
}JIG_TEST_comm_raspberry_cloud_login_t;

typedef struct
{
    float value_speed_test[8];
    bool flag_status_raspberry_cloud_data[3];
    bool read_param_done;
    
    uint32_t count_start_test;
    bool disconnect;
    uint8_t count_timeOut_connect;
    
    JIG_TEST_comm_raspberry_cloud_login_t user_login;
    
    float param_value[JIG_TEST_COMM_RASPBERRY_MAX_PARAM_INDEX];
    
}JIG_TEST_comm_raspberry_global_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief comm_raspberry_configuration
    @return none
*/
void JIG_TEST_comm_raspberry_configuration(void);

/** @brief comm_raspberry_process
    @return none
*/
void JIG_TEST_comm_raspberry_process(void);

/** @brief comm_raspberry_USB_speed_get_read_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_read_speed(void);

/** @brief comm_raspberry_USB_speed_get_write_speed
    @return result : write_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_write_speed(void);

/** @brief comm_raspberry_USB_speed_get_ref_read_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_ref_read_speed(void);

/** @brief comm_raspberry_USB_speed_get_ref_write_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_ref_write_speed(void);

/** @brief JIG_TEST_comm_raspberry_cloud_data_process
    @return none
*/
void JIG_TEST_comm_raspberry_cloud_data_process(void);

/** @brief comm_raspberry_get_heartbeat_ready
    @return none
*/
bool JIG_TEST_comm_raspberry_get_heartbeat_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_COMM_RASPBERRY_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

