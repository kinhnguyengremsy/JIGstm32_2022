/** 
  ******************************************************************************
  * @file    JIG_TEST_gimbal_FSTD_v2.h
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

#ifndef __JIG_TEST_GIMBAL_FSTD_V2_H
#define __JIG_TEST_GIMBAL_FSTD_V2_H

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
typedef enum _JIG_TEST_gimbal_FSTD_v2_mode_test_t
{
    JIG_TEST_GIMBAL_V2_MODE_IDLE = 0x00,
    JIG_TEST_GIMBAL_V2_MODE_SBUS,
    JIG_TEST_GIMBAL_V2_MODE_PPM,
    JIG_TEST_GIMBAL_V2_MODE_CAN,
    JIG_TEST_GIMBAL_V2_MODE_COM,
    JIG_TEST_GIMBAL_V2_MODE_COM4,
    JIG_TEST_GIMBAL_V2_MODE_AUX,
    JIG_TEST_GIMBAL_V2_MODE_VIBRATE,
    JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD,
    JIG_TEST_GIMBAL_V2_MODE_USB_SPEED,
    JIG_TEST_GIMBAL_V2_MODE_DONE,
    JIG_TEST_GIMBAL_V2_MODE_ERROR,
    JIG_TEST_GIMBAL_V2_TOTAL_MODE,
    
    JIG_TEST_GIMBAL_V2_MODE_LOOP,
    
}JIG_TEST_gimbal_FSTD_v2_mode_test_t;

typedef enum _JIG_TEST_gimbal_FSTD_v2_test_state_t
{
    JIG_TEST_GIMBAL_V2_STATE_IDLE = 0x00,
    JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN,
    JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE,
    JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM,
    JIG_TEST_GIMBAL_V2_STATE_WAIT_START,
    JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2,
    JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR,
    JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU,
    JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE,
    JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM,
    JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST,
    JIG_TEST_GIMBAL_V2_STATE_DONE,
    JIG_TEST_GIMBAL_V2_STATE_ERROR,
    JIG_TEST_GIMBAL_V2_TOTAL_STATE,
    
    JIG_TEST_GIMBAL_V2_STATE_LOOP,
    
}JIG_TEST_gimbal_FSTD_v2_test_state_t;

typedef struct
{
    bool get_heartbeatReady;
    
    bool get_login;
    
    bool get_bardCode;
    uint8_t result_pushData;
    
    bool get_start_stop;
    bool get_reset;
    
    bool auto_back;
    
    bool jig_timeOut;
    
    bool cant_read_uuid;
    
}JIG_TEST_gimbal_FSTD_v2_cloudData_status_t;

typedef struct
{
    uint8_t status;
    float value_speed_test[8];
    
    bool enable_test;
    
    bool send_start;
    
    bool test_done;
    
    bool send_end_done;
    
}JIG_TEST_usb_speed_t;

typedef struct
{
    /// gimbal mode test
    JIG_TEST_gimbal_FSTD_v2_mode_test_t mode_test;
    
    /// gimbal state test
    JIG_TEST_gimbal_FSTD_v2_test_state_t state_test;
    
    /// status send from clouData
    JIG_TEST_gimbal_FSTD_v2_cloudData_status_t cloudData_command;

    /// cac ket qua test gimbal
    bool result_mode_test[JIG_TEST_GIMBAL_V2_TOTAL_MODE];
    
    /// get 
    bool get_reset_system;
    
    uint8_t gimbal_startup_calib;
    
    uint32_t time_test_total;
    
    /// flag read param done sai khi test xong
    bool read_param_done;
    
    /// param value send to cloud data
    float param_value[7];
    
    /// flag profile ship error
    bool profile_ship_error;
    
    /// usb speed
    JIG_TEST_usb_speed_t usb_speed;
    
    /// s1v3 vibrate state
    uint8_t vibrate_state_s1v3;
    
}JIG_TEST_gimbal_FSTD_v2_global_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_FSTD_JIG_TEST    1
/* Exported functions --------------------------------------------------------*/
/** @brief gimbal_FSTD_v2_configuration
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_configuration(void);

/** @brief gimbal_FSTD_v2_controlWithCom4
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_controlWithCom4(void);

/** @brief gimbal_FSTD_v2_vibrate
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_vibrate_v2(void);

/** @brief gimbal_FSTD_v2_main_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_main_process(void);

/** @brief gimbal_FSTD_v2_get_first_run
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_get_first_run(void);

/** @brief gimbal_FSTD_v2_get_error_heartbeat_com2
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_get_error_heartbeat_com2(void);
#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_GIMBAL_FSTD_V2_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

