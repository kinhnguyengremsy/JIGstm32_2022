/** 
  ******************************************************************************
  * @file    JIG_TEST_gimbal_FSTD.h
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

#ifndef __JIG_TEST_GIMBAL_FSTD_H
#define __JIG_TEST_GIMBAL_FSTD_H

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

typedef enum _JIG_TEST_gimbal_FSTD_mode_test_t
{
    JIG_TEST_GIMBAL_MODE_IDLE,
    JIG_TEST_GIMBAL_MODE_SBUS,
    JIG_TEST_GIMBAL_MODE_PPM,
    JIG_TEST_GIMBAL_MODE_CAN,
    JIG_TEST_GIMBAL_MODE_COM,
    JIG_TEST_GIMBAL_MODE_COM4,
    JIG_TEST_GIMBAL_MODE_AUX,
    JIG_TEST_GIMBAL_MODE_VIBRATE,
    JIG_TEST_GIMBAL_MODE_DONE,
    JIG_TEST_GIMBAL_MODE_ERROR,
    JIG_TEST_GIMBAL_MODE_LOOP,
    JIG_TEST_GIMBAL_MODE_TOTAL,
    
}JIG_TEST_gimbal_FSTD_mode_test_t;
    
typedef struct _JIG_TEST_gimbal_FSTD_t
{
    
    JIG_TEST_gimbal_FSTD_mode_test_t mode_test;
    
    bool mode_test_result[JIG_TEST_GIMBAL_MODE_TOTAL];
    bool get_result_mode_test_done;
    
    uint8_t usb_speed_result;
    float value_read_speed;
    float value_write_speed;
    float value_ref_read_speed;
    float value_ref_write_speed;
    
    uint8_t start_stopSystem;
    uint8_t result_pushData;
    bool barcode_ready;
    bool display_barcode_done;
    uint32_t display_gimbal_id_storage;
    bool display_gimbal_calib_done;
    bool wait_init_after_reset;
    
    bool first_run_system;
    uint32_t count_start_test;
    bool scan_barCode_done;
    bool reset_all_availables_gimbal_FSTD;
    bool user_logined;
    bool reciver_reset_msg_when_run_test;
    
    uint32_t total_time;
    
}JIG_TEST_gimbal_FSTD_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define GIMBAL_FSTD_JIG_TEST    1
#define DEBUG_ONLY              0
#define GIMBAL_FSTD_AUTO_TEST   1
/* Exported functions --------------------------------------------------------*/
/** @brief gimbal_FSTD_configuration
    @return none
*/
void JIG_TEST_gimbal_FSTD_configuration(void);

/** @brief gimbal_FSTD_control_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_control_process(void);

/** @brief gimbal_FSTD_display_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_display_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_GIMBAL_FSTD_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

