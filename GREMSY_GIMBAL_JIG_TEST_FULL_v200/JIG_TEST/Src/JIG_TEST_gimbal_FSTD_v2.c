/**
  ******************************************************************************
  * @file JIG_TEST_gimbal_FSTD_v2.c
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2016 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes------------------------------------------------------------------------------*/
#include "JIG_TEST_gimbal_FSTD_v2.h"
#include "JIG_TEST_ppm_gimbal.h"
#include "JIG_TEST_can_dji_.h"
#include "JIG_TEST_sbus_gimbal.h"
#include "JIG_TEST_display_v2.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_param_gimbal.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_comm_raspberry_v2.h"
#include "JIG_TEST_button.h"
#include "JIG_TEST_aux.h"
#include "JIG_TEST_rtc.h"
#include "JIG_TEST_config.h"

#include "timeOut.h"
#include "main.h"
#include "string.h"
#include "math.h"
/* Private typedef------------------------------------------------------------------------------*/

typedef enum _control_gimbal_state
{
    STATE_IDLE,

    STATE_CHECK_FIRMWARE_VERSION,
    STATE_SETTING_GIMBAL,
    STATE_SETTING_MESSAGE_RATE,

    STATE_SET_GIMBAL_OFF,
    STATE_SET_GIMBAL_ON,
    
    STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW,
    STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CCW,
    
    STATE_SET_CTRL_GIMBAL_SPEED_MODE,
    STATE_MOVE_SPEED_MODE,
    
    STATE_MOVE_TO_ZERO,

    STATE_SET_GIMBAL_REBOOT,

    STATE_DONE
}control_gimbal_state_t; 

typedef enum _JIG_TEST_gimbal_FSTD_mode_read_t
{
    JIG_TEST_GIMBAL_MODE_READ_IDLE,
    JIG_TEST_GIMBAL_MODE_READ_SBUS = 1,
    JIG_TEST_GIMBAL_MODE_READ_PPM = 6,
    JIG_TEST_GIMBAL_MODE_READ_CAN = 11,
    JIG_TEST_GIMBAL_MODE_READ_COM = 15,
    
}JIG_TEST_gimbal_FSTD_mode_read_t;


typedef enum
{
    STIFFNESS_TYPE_FEED_BACK_IMU,
    STIFFNESS_TYPE_PROFILE_SHIP,
    
}JIG_TEST_gimbal_FSTD_stiffness_type_t;

typedef struct _JIG_TEST_gimbal_FSTD_vibrate_t
{
    
    int16_t gx_delta;
    int16_t gy_delta;
    int16_t gz_delta;
    
    double std_dev_gyro_x_result;
    double std_dev_gyro_y_result;
    double std_dev_gyro_z_result;
    
    uint8_t countDelta_x;
    uint8_t countDelta_y;
    uint8_t countDelta_z;
    
    bool set_gimbal_returnHome;
    bool is_gimbalHome;
    bool checkDome;
    bool checkResult;
    
    bool enable_set_param;
    bool enable_param_compare;
    
    double limit_gx_delta;
    double limit_gy_delta;
    double limit_gz_delta;

}JIG_TEST_gimbal_FSTD_vibrate_t;

typedef struct _JIG_TEST_gimbal_FSTD_v2_private_t
{
    JIG_TEST_gimbal_FSTD_mode_read_t mode_read;
    
    bool fisrt_run;
    uint8_t timeOut_heartbeat;
    
    JIG_TEST_mavlink_gimbal_sensor_check_t imu;
    
    uint32_t time_mode_test;
    bool gimbal_is_home;
    bool apply_mode_test[JIG_TEST_GIMBAL_V2_TOTAL_MODE];
    bool is_test_running;
    bool mode_test_process[JIG_TEST_GIMBAL_V2_TOTAL_MODE];
    uint32_t time_run_test;
    bool test_done[JIG_TEST_GIMBAL_V2_TOTAL_MODE];
    bool gimbal_is_setMODE;
    
    uint16_t aux_test_count[4];
    bool aux_test_result;
    bool aux_test_done;
    
    bool error_heartbeat_com2;
    bool error_heartbeat_pi4;
    
    JIG_TEST_gimbal_FSTD_v2_test_state_t state_storage;
    bool fstorage_state;
    
    JIG_TEST_gimbal_FSTD_vibrate_t vibrate;
    
    uint32_t time_process_, time_process;
    
}JIG_TEST_gimbal_FSTD_v2_private_t;

/* Private define------------------------------------------------------------------------------*/

#define COMMAND_CONTROL_MOTOR       31010
#define COMMAND_SETTING_MODE        31011
#define MAVLINK_RESULT_ACCEPTED     0
#define COMMAND_DO_MOUNT_CONFIG     204
#define COMMAND_DO_MOUNT_CONTROL    205

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
JIG_TEST_gimbal_FSTD_v2_private_t gimbal_FSTD_private;
JIG_TEST_gimbal_FSTD_v2_global_t  gimbal_FSTD_global;

extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM2;
extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM4;

/** @group JIG_TEST_GIMBAL_PARAM_VARIABLE
    @{
*/#ifndef JIG_TEST_GIMBAL_PARAM_VARIABLE
#define JIG_TEST_GIMBAL_PARAM_VARIABLE

struct _gimbal_param_setting
{
    float value;
    float value_param_get;
    int16_t index;
    char* param_id;
}gimbal_param_setting[JIG_TEST_NUMBER_OF_PARAM_SETTING] = 
{
    {.value = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 0, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
    {.value = 0, .index = 39, .param_id = "SBUS_PITCH_CHAN"},
    {.value = 0, .index = 42, .param_id = "SBUS_MODE_CHAN"},
    {.value = 0, .index = 10, .param_id = "YAW_D"},
    {.value = 0, .index = 4, .param_id = "PITCH_D"},
    {.value = 0, .index = 23, .param_id = "NPOLES_YAW"},
    {.value = 0, .index = 22, .param_id = "NPOLES_ROLL"},
    {.value = 0, .index = 21, .param_id = "NPOLES_PITCH"},
    {.value = 0, .index = 8, .param_id = "YAW_P"},
    {.value = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .index = 2, .param_id = "PITCH_P"},//12
    {.value = 0, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 0, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 0, .index = 13, .param_id = "YAW_POWER"},//15
    {.value = 0, .index = 63, .param_id = "JOY_AXIS"}, /// ???
    {.value = 0, .index = 60, .param_id = "RC_PITCH_SPEED"},
    {.value = 0, .index = 61, .param_id = "RC_ROLL_SPEED"},
    {.value = 0, .index = 62, .param_id = "RC_YAW_SPEED"},
    {.value = 0, .index = 29, .param_id = "GYRO_LPF"},//20
    {.value = 0, .index = 9, .param_id = "YAW_I"},
    {.value = 0, .index = 3, .param_id = "PITCH_I"},
    {.value = 0, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 0, .index = 77, .param_id = "IMU_RATE"},
    {.value = 0, .index = 72, .param_id = "HEARTBEAT_EMIT"},
    {.value = 0, .index = 73, .param_id = "STATUS_RATE"},
    {.value = 0, .index = 74, .param_id = "ENC_CNT_RATE"},
    {.value = 0, .index = 76, .param_id = "ORIEN_RATE"}, // 28
    {.value = 0, .index = 30, .param_id = "TRAVEL_MIN_PIT"}, 
    {.value = 0, .index = 31, .param_id = "TRAVEL_MAX_PIT"}, 
    {.value = 0, .index = 32, .param_id = "TRAVEL_MIN_ROLL"}, 
    {.value = 0, .index = 33, .param_id = "TRAVEL_MAX_ROLL"}, 
    {.value = 0, .index = 69, .param_id = "TRAVEL_MIN_PAN"}, 
    {.value = 0, .index = 70, .param_id = "TRAVEL_MAX_PAN"}, // 34
    {.value = 0, .index = 25, .param_id = "DIR_MOTOR_ROLL"}, // 35
};

struct _gimbal_user_profile_ship
{
    float value;
    float value_param_get;
    int16_t index;
    char* param_id;
}gimbal_user_profile_ship[JIG_TEST_GIMBAL_USER_PROFILE_SHIP] = 
{
    {.value = 0, .value_param_get = 0, .index = 100, .param_id = "       "},
    {.value = 0, .value_param_get = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .value_param_get = 0, .index = 67, .param_id = "VERSION_Y"},
    {.value = 0, .value_param_get = 0, .index = 68, .param_id = "VERSION_Z"},
    {.value = 0, .value_param_get = 0, .index = 2, .param_id = "PITCH_P"},
    {.value = 0, .value_param_get = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .value_param_get = 0, .index = 8, .param_id = "YAW_P"},
    {.value = 0, .value_param_get = 0, .index = 9, .param_id = "YAW_I"},
    {.value = 0, .value_param_get = 0, .index = 29, .param_id = "GYRO_LPF"},
    {.value = 0, .value_param_get = 0, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 0, .value_param_get = 0, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 0, .value_param_get = 0, .index = 13, .param_id = "YAW_POWER"},
    {.value = 0, .value_param_get = 0, .index = 3, .param_id = "PITCH_I"},
    {.value = 0, .value_param_get = 0, .index = 25, .param_id = "DIR_MOTOR_ROLL"},
    {.value = 0, .value_param_get = 0, .index = 14, .param_id = "PITCH_FOLLOW"},
    {.value = 0, .value_param_get = 0, .index = 17, .param_id = "PITCH_FILTER"},
    {.value = 0, .value_param_get = 0, .index = 57, .param_id = "TILT_WINDOW"},
    {.value = 0, .value_param_get = 0, .index = 7, .param_id = "ROLL_D"},
    {.value = 0, .value_param_get = 0, .index = 16, .param_id = "YAW_FOLLOW"},
    {.value = 0, .value_param_get = 0, .index = 19, .param_id = "YAW_FILTER"},
    {.value = 0, .value_param_get = 0, .index = 58, .param_id = "PAN_WINDOW"},
    {.value = 0, .value_param_get = 0, .index = 24, .param_id = "DIR_MOTOR_PITCH"},
    {.value = 0, .value_param_get = 0, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 0, .value_param_get = 0, .index = 51, .param_id = "RC_PITCH_TRIM"},
    {.value = 0, .value_param_get = 0, .index = 52, .param_id = "RC_ROLL_TRIM"},
    {.value = 0, .value_param_get = 0, .index = 42, .param_id = "SBUS_MODE_CHAN"},
    {.value = 0, .value_param_get = 0, .index = 39, .param_id = "SBUS_PITCH_CHAN"},
    {.value = 0, .value_param_get = 0, .index = 36, .param_id = "RC_PITCH_LPF"},
    {.value = 0, .value_param_get = 0, .index = 54, .param_id = "RC_PITCH_MODE"},
    {.value = 0, .value_param_get = 0, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
    {.value = 0, .value_param_get = 0, .index = 37, .param_id = "RC_ROLL_LPF"},
    {.value = 0, .value_param_get = 0, .index = 55, .param_id = "RC_ROLL_MODE"},
    {.value = 0, .value_param_get = 0, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 0, .value_param_get = 0, .index = 38, .param_id = "RC_YAW_LPF"},
    {.value = 0, .value_param_get = 0, .index = 56, .param_id = "RC_YAW_MODE"},
    {.value = 0, .value_param_get = 0, .index = 4, .param_id = "PITCH_D"},
    {.value = 0, .value_param_get = 0, .index = 10, .param_id = "YAW_D"},//36
    
    {.value = 0, .value_param_get = 0, .index = 46, .param_id = "GYROX_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 47, .param_id = "GYROY_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 48, .param_id = "GYROZ_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 43, .param_id = "ACCX_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 44, .param_id = "ACCY_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 45, .param_id = "ACCZ_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 71, .param_id = "GIMBAL_OVAL"},

};

struct _gimbal_user_profile_ship_ac30000
{
    float value;
    float value_param_get;
    int16_t index;
    char* param_id;
}gimbal_user_profile_ship_ac30000[JIG_TEST_GIMBAL_USER_PROFILE_SHIP_AC30000] = 
{
    {.value = 0, .value_param_get = 0, .index = 100, .param_id = "       "},
    {.value = 0, .value_param_get = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .value_param_get = 0, .index = 67, .param_id = "VERSION_Y"},
    {.value = 0, .value_param_get = 0, .index = 68, .param_id = "VERSION_Z"},
    {.value = 0, .value_param_get = 0, .index = 18, .param_id = "ROLL_FILTER"},
    {.value = 0, .value_param_get = 0, .index = 63, .param_id = "JOY_AXIS"},
    {.value = 0, .value_param_get = 0, .index = 2, .param_id = "PITCH_P"},
    {.value = 0, .value_param_get = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .value_param_get = 0, .index = 8, .param_id = "YAW_P"},
    {.value = 0, .value_param_get = 0, .index = 9, .param_id = "YAW_I"},
    {.value = 0, .value_param_get = 0, .index = 29, .param_id = "GYRO_LPF"},//10
    {.value = 0, .value_param_get = 0, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 0, .value_param_get = 0, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 0, .value_param_get = 0, .index = 13, .param_id = "YAW_POWER"},
    {.value = 0, .value_param_get = 0, .index = 3, .param_id = "PITCH_I"},
    {.value = 0, .value_param_get = 0, .index = 25, .param_id = "DIR_MOTOR_ROLL"},
    {.value = 0, .value_param_get = 0, .index = 14, .param_id = "PITCH_FOLLOW"},
    {.value = 0, .value_param_get = 0, .index = 17, .param_id = "PITCH_FILTER"},
    {.value = 0, .value_param_get = 0, .index = 57, .param_id = "TILT_WINDOW"},
    {.value = 0, .value_param_get = 0, .index = 7, .param_id = "ROLL_D"},
    {.value = 0, .value_param_get = 0, .index = 16, .param_id = "YAW_FOLLOW"},//20
    {.value = 0, .value_param_get = 0, .index = 19, .param_id = "YAW_FILTER"},
    {.value = 0, .value_param_get = 0, .index = 58, .param_id = "PAN_WINDOW"},
    {.value = 0, .value_param_get = 0, .index = 24, .param_id = "DIR_MOTOR_PITCH"},
    {.value = 0, .value_param_get = 0, .index = 60, .param_id = "RC_PITCH_SPEED"},
    {.value = 0, .value_param_get = 0, .index = 61, .param_id = "RC_ROLL_SPEED"},
    {.value = 0, .value_param_get = 0, .index = 62, .param_id = "RC_YAW_SPEED"},
    {.value = 0, .value_param_get = 0, .index = 36, .param_id = "RC_PITCH_LPF"},
    {.value = 0, .value_param_get = 0, .index = 37, .param_id = "RC_ROLL_LPF"},
    {.value = 0, .value_param_get = 0, .index = 38, .param_id = "RC_YAW_LPF"},
    {.value = 0, .value_param_get = 0, .index = 28, .param_id = "RADIO_TYPE"},//30
    {.value = 0, .value_param_get = 0, .index = 72, .param_id = "HEARTBEAT_EMIT"},
    {.value = 0, .value_param_get = 0, .index = 73, .param_id = "STATUS_RATE"},
    {.value = 0, .value_param_get = 0, .index = 74, .param_id = "ENC_CNT_RATE"},
    {.value = 0, .value_param_get = 0, .index = 76, .param_id = "ORIEN_RATE"},
    {.value = 0, .value_param_get = 0, .index = 77, .param_id = "IMU_RATE"},
    {.value = 0, .value_param_get = 0, .index = 75, .param_id = "ENC_TYPE_SEND"},
    {.value = 0, .value_param_get = 0, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 0, .value_param_get = 0, .index = 50, .param_id = "SKIP_GYRO_CALIB"},
    {.value = 0, .value_param_get = 0, .index = 51, .param_id = "RC_PITCH_TRIM"},
    {.value = 0, .value_param_get = 0, .index = 52, .param_id = "RC_ROLL_TRIM"},//40
    {.value = 0, .value_param_get = 0, .index = 30, .param_id = "TRAVEL_MIN_PIT"},
    {.value = 0, .value_param_get = 0, .index = 31, .param_id = "TRAVEL_MAX_PIT"},
    {.value = 0, .value_param_get = 0, .index = 32, .param_id = "TRAVEL_MIN_ROLL"},
    {.value = 0, .value_param_get = 0, .index = 33, .param_id = "TRAVEL_MAX_ROLL"},
    {.value = 0, .value_param_get = 0, .index = 69, .param_id = "TRAVEL_MIN_PAN"},
    {.value = 0, .value_param_get = 0, .index = 70, .param_id = "TRAVEL_MAX_PAN"},
    {.value = 0, .value_param_get = 0, .index = 64, .param_id = "TRIM_HOZ"},
    {.value = 0, .value_param_get = 0, .index = 15, .param_id = "ROLL_FOLLOW"},//48

    
    {.value = 0, .value_param_get = 0, .index = 46, .param_id = "GYROX_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 47, .param_id = "GYROY_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 48, .param_id = "GYROZ_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 43, .param_id = "ACCX_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 44, .param_id = "ACCY_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 45, .param_id = "ACCZ_OFFSET"},
    {.value = 0, .value_param_get = 0, .index = 71, .param_id = "GIMBAL_OVAL"},
};

const char debug_str_state[JIG_TEST_GIMBAL_V2_TOTAL_STATE][70] = 
{
    "",
    "",
    "",
    "JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM",
    "JIG_TEST_GIMBAL_V2_STATE_WAIT_START",
    "JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2",
    "JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR",
    "JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU",
    "JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE",
    "JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM",
    "JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST",
};

const char debug_str_mode_test[JIG_TEST_GIMBAL_V2_TOTAL_MODE][70] = 
{
    "JIG_TEST_GIMBAL_V2_MODE_IDLE",
    "JIG_TEST_GIMBAL_V2_MODE_SBUS",
    "JIG_TEST_GIMBAL_V2_MODE_PPM",
    "JIG_TEST_GIMBAL_V2_MODE_CAN",
    "JIG_TEST_GIMBAL_V2_MODE_COM",
    "JIG_TEST_GIMBAL_V2_MODE_COM4",
    "JIG_TEST_GIMBAL_V2_MODE_AUX",
    "JIG_TEST_GIMBAL_V2_MODE_VIBRATE",
    "JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD",
    "JIG_TEST_GIMBAL_V2_MODE_USB_SPEED"
    "JIG_TEST_GIMBAL_V2_MODE_DONE",
    "JIG_TEST_GIMBAL_V2_MODE_ERROR",
};


#endif
/**
    @}
*/

/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_GIMBAL_FSTD_V2_CONFIGURATION
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_V2_CONFIGURATION
#define JIG_TEST_GIMBAL_FSTD_V2_CONFIGURATION

/** @brief gimbal_FSTD_v2_configuration
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_configuration(void)
{
    char buff[300];
    
    /// get value storage backup register
    gimbal_FSTD_global.get_reset_system = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR11);
    gimbal_FSTD_private.fisrt_run       = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR12);
    gimbal_FSTD_private.state_storage   = (JIG_TEST_gimbal_FSTD_v2_test_state_t)JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR13);
    
    /// get usb speed variables from backup register
    gimbal_FSTD_global.usb_speed.send_start = (bool)JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR14);
    gimbal_FSTD_global.usb_speed.test_done  = (bool)JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR15);
    
    if(gimbal_FSTD_global.usb_speed.send_start == true)
    {
        /// set enable ussb speed test khi reset lai
        gimbal_FSTD_global.usb_speed.enable_test = true;
    }
    
    /// lay gia tri state from backup resister
    gimbal_FSTD_global.state_test = gimbal_FSTD_private.state_storage;
    
    /// config library
    timeOut_configuration();
    JIG_TEST_display_v2_configuration();
    JIG_TEST_ppm_gimbal_configuration();
    #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x34)
        JIG_TEST_can_dji_configuration();
    #endif
    
    #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        JIG_TEST_aux_configuration();
    #endif
    JIG_TEST_sbus_gimbal_configuration();
    JIG_TEST_mavlink_gimbal_configuration();
    JIG_TEST_console_configuration();
    JIG_TEST_button_configuration();
    JIG_TEST_comm_raspberry_v2_cloudData_configuration();
    
    /// reset all variables serialPort
    JIG_TEST_mavlink_serialPort3_Reinit();
    
    /// reset all variables mavlink
    memset(&mavlink_gimbal_COM2, 0, sizeof(JIG_TEST_mavlink_gimbal_t));
    memset(&mavlink_gimbal_COM4, 0, sizeof(JIG_TEST_mavlink_gimbal_t));
    
    /// turn off all led
    HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
    
    
    /// write to console init
    sprintf(buff, "GREMSY GIMBAL JIG TEST\n         FSTD\n         v201\n");
    JIG_TEST_console_write(buff);
}

#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_GIMBAL
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_GIMBAL
#define JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_GIMBAL

/** @brief timeOut_heartbeat
    @return true : da nhan duoc heartbeat, angle, state
            false : dang timeOut heartbeat
*/
static bool JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    bool ret = false;
    static uint32_t count_console = 0;
    static uint8_t timeOut_heartbeat = 0;
    char *str = "\nCONNECTION_GIMBAL_COM2 --->";
    
    if(gimbal_channel->seen_heartbeat == false)
    {
        /// ktra thoi gian 5s timeOut heartbeat
        if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_TIME_OUT_HEARTBEAT_COM2))
        {
            if(timeOut_heartbeat ++ > 5) 
            {
                timeOut_heartbeat = 0;
                
                /// re init mavlink serialPort3
                JIG_TEST_mavlink_serialPort3_Reinit();
                
                JIG_TEST_console_write(str);
                
                /// khong nhan duoc heartbeat --> setting param heartbeat emit
                gimbal_param_setting[25].value = JIG_TEST_SETTING_PARAM_TEST_HEARTBEAT_EMIT;
                JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[25].value, gimbal_param_setting[25].param_id);
                HAL_Delay(10);
                
                char buff[100];
                
                sprintf(buff, "setting param heartbeat emit : value <-> %f | id : %s\n"
                , gimbal_param_setting[25].value, gimbal_param_setting[25].param_id);
                JIG_TEST_console_write(buff);
                
                if(gimbal_FSTD_private.timeOut_heartbeat ++ > 10)
                {
                    /// next state error
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
                    
                    /// set flag error
                    gimbal_FSTD_private.error_heartbeat_com2 = true;
                }
            }
        }
        
        /// write to console 
        if(++count_console > 200000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("not found gimbal heartbeat !!!\n");
            
            /// toggle led red 
            HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
        }
    }
    else
    {
        ret = true;
        
        /// turn off led red
        HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
        
        /// write to console 
        if(++count_console > 200000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("got gimbal heartbeat !!!\n");
        }
    }
    
    return ret;
}



/** @brief gimbal_FSTD_copy_profile_ship_follow_gimbal_id
    @return true : copy param complete
            false : copy param running
*/
static bool JIG_TEST_gimbal_FSTD_setting_param_test(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char buff[100];

    
    #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21 || JIG_TEST_ID == 0x34)
    
        for(uint8_t i = 0; i < JIG_TEST_NUMBER_OF_PARAM_SETTING; i++)
        {
            if(gimbal_channel->vehicle_system_id == 0x44) // t3v3
            {
                gimbal_param_setting[i].value = JIG_TEST_setting_param_test_for_t3v3[i].value;
            }
            else if(gimbal_channel->vehicle_system_id == 0x22) // s1v3
            {
                gimbal_param_setting[i].value = JIG_TEST_setting_param_test_for_s1v3[i].value;
                
                /// re setting param gyro_struct
                gimbal_param_setting[23].value = JIG_TEST_SETTING_PARAM_TEST_GYRO_TRUST_S1V3;
            }
            else if(gimbal_channel->vehicle_system_id == 0x08) // t7
            {
                gimbal_param_setting[i].value = JIG_TEST_setting_param_test_for_t7[i].value;
            }
            else if(gimbal_channel->vehicle_system_id == 0x7A) // ACSL
            {
                gimbal_param_setting[i].value = JIG_TEST_setting_param_test_for_acsl[i].value;
            }
            else
            {
                gimbal_param_setting[i].value = JIG_TEST_setting_param_test_for_t3v3[i].value;
            }
        }
    
    #endif

    /// kiem tra loai imu de setting gyro filter
    if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
    {
        if(gimbal_channel->vehicle_system_id == 0x44) // t3v3
        {
            gimbal_param_setting[20].value = \
            JIG_TEST_setting_param_test_for_t3v3[20].value = \
            JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T3V3_ICM;
        }
        else if(gimbal_channel->vehicle_system_id == 0x22) // s1v3
        {
            gimbal_param_setting[20].value = \
            JIG_TEST_setting_param_test_for_s1v3[20].value = \
            JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_S1V3_ICM;
        }
        else if(gimbal_channel->vehicle_system_id == 0x08) // t7
        {
            gimbal_param_setting[20].value = \
            JIG_TEST_setting_param_test_for_t7[20].value = \
            JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T7_ICM;
        }
    }
    
    /// setting gimbal proFile ship 
    for(uint8_t i = 0; i < JIG_TEST_NUMBER_OF_PARAM_SETTING; i++)
    {
        JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[i].value, gimbal_param_setting[i].param_id);
        HAL_Delay(20);
        
        sprintf(buff, "\n SETTING PARAM GIMBAL ---> %s  <->  value : %.f\n"
        , gimbal_param_setting[i].param_id
        , gimbal_param_setting[i].value);
        JIG_TEST_console_write(buff);
    }
    
    ret = true;
    
    return ret;
}

/** @brief FSTD_request_param_gimbal
    @return true : request complete
            false : request running
*/
static bool JIG_TEST_gimbal_FSTD_request_param_gimbal(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char *str = "\nREQUEST_PARAM ---->";
    static bool console_enable = false;
    static uint32_t count_param_request = 0;
    static uint8_t param_index_temp = 0;
    static bool copy_param = false;
    static uint8_t count_timeOut;
    char buff[300];
    
    if(copy_param == false)
    {
        copy_param = JIG_TEST_gimbal_FSTD_setting_param_test(gimbal_channel);
        
        JIG_TEST_console_write(str);
        HAL_Delay(5);
        JIG_TEST_console_write("Copy param Done !!!");
        
        /// reset param index reciever from gimbal
        gimbal_channel->param_value.param_index = 0;
    }
    else
    {
        /// ktra timeOut setting param gimbal
        if(get_timeOut(100, JIG_TEST_TIMEOUT_SETTING_PARAM_GIMBAL))
        {
            count_timeOut ++;
            
            if(count_timeOut >= 50)
            {
                JIG_TEST_console_write(str);
                HAL_Delay(15);
                JIG_TEST_console_write("timeOut setting param gimbal\n");
                
                NVIC_SystemReset();
            }
        }
        
        param_index_temp = gimbal_channel->param_value.param_index ;
        
        if(param_index_temp == gimbal_param_setting[count_param_request].index)
        {
            /// send request de lay param new
            JIG_TEST_mavlink_gimbal_send_param_request_read(
                gimbal_param_setting[count_param_request + 1].index
            ,   gimbal_param_setting[count_param_request + 1].param_id);
            HAL_Delay(2);
            
            if(console_enable == false)
            {
                console_enable = true;
                
                JIG_TEST_console_write(str);
                HAL_Delay(15);
                sprintf(buff, "send request param : %s | index : %3d\n"
                , gimbal_param_setting[count_param_request + 1].param_id
                , gimbal_param_setting[count_param_request + 1].index);
                JIG_TEST_console_write(buff);
            }
        }
        else
        {
            uint8_t param_index_r = gimbal_channel->param_value.param_index;
            uint8_t param_insex_s = gimbal_param_setting[count_param_request + 1].index;
            
            /// lay gia tri param tu COM2
            if(param_index_r == param_insex_s)
            {
                /// enable console
                console_enable = false;
                
                gimbal_param_setting[count_param_request + 1].value_param_get = gimbal_channel->param_value.param_value;
                
                if(gimbal_channel->vehicle_system_id == 0x44) // t3v3
                {
                    sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                    , gimbal_param_setting[count_param_request + 1].param_id
                    , gimbal_param_setting[count_param_request + 1].value_param_get
                    , JIG_TEST_setting_param_test_for_t3v3[count_param_request + 1].value
                    , count_param_request);
                }
                else if(gimbal_channel->vehicle_system_id == 0x22) // s1v3
                {
                    sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                    , gimbal_param_setting[count_param_request + 1].param_id
                    , gimbal_param_setting[count_param_request + 1].value_param_get
                    , JIG_TEST_setting_param_test_for_s1v3[count_param_request + 1].value
                    , count_param_request);
                }
                else if(gimbal_channel->vehicle_system_id == 0x08) // t7
                {
                    sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                    , gimbal_param_setting[count_param_request + 1].param_id
                    , gimbal_param_setting[count_param_request + 1].value_param_get
                    , JIG_TEST_setting_param_test_for_t7[count_param_request + 1].value
                    , count_param_request);
                }

                JIG_TEST_console_write(str);
                HAL_Delay(5);
                JIG_TEST_console_write(buff);
                HAL_Delay(15);
                
                /// next param 
                count_param_request ++;

                /// kiem tra so luong param doc duoc
                if(count_param_request == JIG_TEST_NUMBER_OF_PARAM_SETTING - 1)
                {
//                    /// re init mavlink comm raspberry
//                    JIG_TEST_mavlink_serialPort5_Reinit();
                    
                    ret = true;
                }
            }
        }
    }

    return ret;
}


#if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x20)
/** @brief gimbal_FSTD_copy_profile_ship_follow_gimbal_id
    @return true : copy param complete
            false : copy param running
*/
static bool JIG_TEST_gimbal_FSTD_v2_copy_profile_ship_follow_gimbal_id(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char buff[100];
    
        if(gimbal_channel->vehicle_system_id == 0x44) // T3v3
        {
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                gimbal_user_profile_ship[i + 4].value = JIG_TEST_gimbal_user_profile_ship_T3[i].value;
            }
            
            /// setting gimbal proFile ship 
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                JIG_TEST_mavlink_gimbal_set_param(gimbal_user_profile_ship[i + 4].value, gimbal_user_profile_ship[i + 4].param_id);
                HAL_Delay(20);
                
                sprintf(buff, "\n SETTING PROFILE GIMBAL T3v3 SHIP ---> %s  <->  value : %.f\n", gimbal_user_profile_ship[i].param_id, gimbal_user_profile_ship[i].value);
                JIG_TEST_console_write(buff);
            }
            
            ret = true;
        }
        else if(gimbal_channel->vehicle_system_id == 0x22) // S1v3
        {
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                gimbal_user_profile_ship[i + 4].value = JIG_TEST_gimbal_user_profile_ship_S1v3[i].value;
            }
            
            /// setting gimbal proFile ship 
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                JIG_TEST_mavlink_gimbal_set_param(gimbal_user_profile_ship[i + 4].value, gimbal_user_profile_ship[i + 4].param_id);
                HAL_Delay(20);
                
                sprintf(buff, "\n SETTING PROFILE GIMBAL S1v3 SHIP ---> %s  <->  value : %.f\n", gimbal_user_profile_ship[i].param_id, gimbal_user_profile_ship[i].value);
                JIG_TEST_console_write(buff);
            }
            
            ret = true;
        }
        else if(gimbal_channel->vehicle_system_id == 0x08) // T7
        {
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                gimbal_user_profile_ship[i + 4].value = JIG_TEST_gimbal_user_profile_ship_T7[i].value;
            }
            
            /// setting gimbal proFile ship 
            for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 7; i++)
            {
                JIG_TEST_mavlink_gimbal_set_param(gimbal_user_profile_ship[i + 4].value, gimbal_user_profile_ship[i + 4].param_id);
                HAL_Delay(20);
                
                sprintf(buff, "\n SETTING PROFILE GIMBAL T7 SHIP ---> %s  <->  value : %.f\n"
                , gimbal_user_profile_ship[i].param_id
                , gimbal_user_profile_ship[i].value);
                JIG_TEST_console_write(buff);
            }
            
            ret = true;
        }
    
    return ret;
}
#endif

#if (JIG_TEST_ID == 0x21)
/** @brief gimbal_FSTD_copy_profile_ship_follow_gimbal_id
    @return true : copy param complete
            false : copy param running
*/
static bool JIG_TEST_gimbal_FSTD_v2_copy_profile_ship_follow_gimbal_id(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char buff[100];

        for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP_AC30000 - 7; i++)
        {
            gimbal_user_profile_ship_ac30000[i + 4].value = JIG_TEST_gimbal_user_profile_ship_AC30000[i].value;
        }
        
        /// setting gimbal proFile ship 
        for(uint8_t i = 0; i < JIG_TEST_GIMBAL_USER_PROFILE_SHIP_AC30000 - 7; i++)
        {
            JIG_TEST_mavlink_gimbal_set_param(gimbal_user_profile_ship_ac30000[i + 4].value, gimbal_user_profile_ship_ac30000[i + 4].param_id);
            HAL_Delay(20);
            
            sprintf(buff, "\n SETTING PROFILE GIMBAL AC30000 SHIP ---> %s  <->  value : %.f\n"
            , gimbal_user_profile_ship_ac30000[i].param_id
            , gimbal_user_profile_ship_ac30000[i].value);
            JIG_TEST_console_write(buff);
        }
        
        ret = true;
        
    return ret;
}
#endif
#if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x20)

/** @brief FSTD_request_param_gimbal
    @return true : compare complete
            false : compare running
*/
static bool JIG_TEST_gimbal_FSTD_v2_request_param_profile_ship(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char *str = "\nGIMBAL_USER_PROFILE_SHIP ---->";
    static bool console_enable = false;
    static uint32_t count_param_request = 0;
    static uint8_t param_index_temp = 0;
    static bool gimbal_profile_detect = false;
    char buff[300];
    
    if(gimbal_profile_detect == false)
    {
        gimbal_profile_detect = JIG_TEST_gimbal_FSTD_v2_copy_profile_ship_follow_gimbal_id(gimbal_channel);
        
        JIG_TEST_console_write(str);
        HAL_Delay(5);
        JIG_TEST_console_write("Copy data Done !!!");
        
        /// reset param index reciever from gimbal
        gimbal_channel->param_value.param_index = 0;
    }
    else
    {
        param_index_temp = gimbal_channel->param_value.param_index ;
        
        if(param_index_temp == gimbal_user_profile_ship[count_param_request].index)
        {
            /// send request de lay param new
            JIG_TEST_mavlink_gimbal_send_param_request_read(
                gimbal_user_profile_ship[count_param_request + 1].index
            ,   gimbal_user_profile_ship[count_param_request + 1].param_id);
            HAL_Delay(5);
            
            if(console_enable == false)
            {
                console_enable = true;
                
                JIG_TEST_console_write(str);
                HAL_Delay(5);
                sprintf(buff, "send request param : %s | index : %3d\n"
                , gimbal_user_profile_ship[count_param_request + 1].param_id
                , gimbal_user_profile_ship[count_param_request + 1].index);
                JIG_TEST_console_write(buff);
            }
        }
        else
        {
            /// lay gia tri param tu COM2
            if(gimbal_channel->param_value.param_index == gimbal_user_profile_ship[count_param_request + 1].index)
            {
                /// enable console
                console_enable = false;
                
                gimbal_user_profile_ship[count_param_request + 1].value_param_get = gimbal_channel->param_value.param_value;
                
                if(count_param_request < 3)
                {
                    
                    sprintf(buff, "reciever : %s <-> value : %.f | count : %3d\n" 
                    , gimbal_user_profile_ship[count_param_request + 1].param_id
                    , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                    , count_param_request);
                    
                    /// next param 
                    count_param_request ++;
                }
                else if(count_param_request >= 3 && count_param_request <= 35)
                {
                    /// compare param read with param set
                    if(gimbal_channel->vehicle_system_id == 0x44) // t3v3
                    {
                        sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                        , gimbal_user_profile_ship[count_param_request + 1].param_id
                        , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                        , JIG_TEST_gimbal_user_profile_ship_T3[count_param_request - 3].value
                        , count_param_request);
                        
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_T3[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                    else if(gimbal_channel->vehicle_system_id == 0x22) // s1v3
                    {
                        sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                        , gimbal_user_profile_ship[count_param_request + 1].param_id
                        , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                        , JIG_TEST_gimbal_user_profile_ship_S1v3[count_param_request - 3].value
                        , count_param_request);
                        
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_S1v3[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                    else if(gimbal_channel->vehicle_system_id == 0x08) // t7
                    {
                        sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                        , gimbal_user_profile_ship[count_param_request + 1].param_id
                        , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                        , JIG_TEST_gimbal_user_profile_ship_T7[count_param_request - 3].value
                        , count_param_request);
                        
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_T7[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                }
                else
                {
                    sprintf(buff, "reciever : %s <-> value : %.f | count : %3d\n" 
                    , gimbal_user_profile_ship[count_param_request + 1].param_id
                    , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                    , count_param_request);
                    
                    /// next param 
                    count_param_request ++;
                }
                
                JIG_TEST_console_write(str);
                HAL_Delay(5);
                JIG_TEST_console_write(buff);
                HAL_Delay(15);

                /// kiem tra so luong param doc duoc
                if(count_param_request == (JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 1))
                {
                    /// re init mavlink comm raspberry
//                    JIG_TEST_mavlink_serialPort5_Reinit();
                    
                    ret = true;
                }
            }
        }
    }

    return ret;
}

#endif

#if (JIG_TEST_ID == 0x21)

/** @brief gimbal_FSTD_get_gimbal_startup_calib_motor
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_v2_timeOut_load_proFile_ship(void)
{
    bool ret = false;
    static uint8_t count_timeOut = 0;
    
    if(get_timeOut(1000, JIG_TEST_TIMEOUT_PROFILE_SHIP))
    {
        if(++count_timeOut >= 10)
        {
            count_timeOut = 0;
            
            ret = true;
        }
    }
    
    return ret;
}

/** @brief FSTD_request_param_gimbal
    @return true : compare complete
            false : compare running
*/
static bool JIG_TEST_gimbal_FSTD_v2_request_param_profile_ship(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char *str = "\nGIMBAL_USER_PROFILE_SHIP ---->";
    static bool console_enable = false;
    static uint32_t count_param_request = 0;
    static uint8_t param_index_temp = 0;
    static bool gimbal_profile_detect = false;
    static uint8_t count_timeOut = 0;
    char buff[300];
    
    if(JIG_TEST_gimbal_FSTD_v2_timeOut_load_proFile_ship() == true)
    {
        if(++count_timeOut >= 3)
        {
            JIG_TEST_console_write(str);
            HAL_Delay(5);
            JIG_TEST_console_write("Load profile Ship Error ............................... !!!");
            
            gimbal_FSTD_global.profile_ship_error = true;
            
            /// next state error
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
        }
        
        /// timeOut profileShip ---> reset all variables
        console_enable = false;
        count_param_request = 0;
        param_index_temp = 0;
        gimbal_profile_detect = false;
        
        /// re init serialPort3
        JIG_TEST_mavlink_serialPort3_Reinit();
        
        JIG_TEST_console_write(str);
        HAL_Delay(5);
        JIG_TEST_console_write("Load profile Ship try again............................... !!!");
    }
    else
    {
        if(gimbal_profile_detect == false)
        {
            gimbal_profile_detect = JIG_TEST_gimbal_FSTD_v2_copy_profile_ship_follow_gimbal_id(gimbal_channel);
            
            JIG_TEST_console_write(str);
            HAL_Delay(5);
            JIG_TEST_console_write("Copy & setting param Done !!!");
            
            /// reset param index reciever from gimbal
            gimbal_channel->param_value.param_index = 0;
        }
        else
        {
            param_index_temp = gimbal_channel->param_value.param_index ;
            
            if(param_index_temp == gimbal_user_profile_ship_ac30000[count_param_request].index)
            {
                /// send request de lay param new
                JIG_TEST_mavlink_gimbal_send_param_request_read(
                    gimbal_user_profile_ship_ac30000[count_param_request + 1].index
                ,   gimbal_user_profile_ship_ac30000[count_param_request + 1].param_id);
                HAL_Delay(5);
                
                if(console_enable == false)
                {
                    console_enable = true;
                    
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    sprintf(buff, "send request param : %s | index : %3d\n"
                    , gimbal_user_profile_ship_ac30000[count_param_request + 1].param_id
                    , gimbal_user_profile_ship_ac30000[count_param_request + 1].index);
                    JIG_TEST_console_write(buff);
                }
            }
            else
            {
                /// lay gia tri param tu COM2
                if(gimbal_channel->param_value.param_index == gimbal_user_profile_ship_ac30000[count_param_request + 1].index)
                {
                    /// enable console
                    console_enable = false;
                    
                    gimbal_user_profile_ship_ac30000[count_param_request + 1].value_param_get = gimbal_channel->param_value.param_value;
                    
                    if(count_param_request < 3)
                    {
                        
                        sprintf(buff, "reciever : %s <-> value : %.f | count : %3d\n" 
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].param_id
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].value_param_get
                        , count_param_request);
                        
                        /// next param 
                        count_param_request ++;
                    }
                    else if(count_param_request >= 3 && count_param_request <= 47)
                    {
                        /// compare param read with param set
                        sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].param_id
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].value_param_get
                        , JIG_TEST_gimbal_user_profile_ship_AC30000[count_param_request - 3].value
                        , count_param_request);
                        
                        if(gimbal_user_profile_ship_ac30000[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_AC30000[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                    else
                    {
                        sprintf(buff, "reciever : %s <-> value : %.f | count : %3d\n" 
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].param_id
                        , gimbal_user_profile_ship_ac30000[count_param_request + 1].value_param_get
                        , count_param_request);
                        
                        /// next param 
                        count_param_request ++;
                    }
                    
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write(buff);
                    HAL_Delay(15);

                    /// kiem tra so luong param index doc duoc
                    if(count_param_request == (JIG_TEST_GIMBAL_USER_PROFILE_SHIP_AC30000 - 1))
                    {
                        /// re init mavlink comm raspberry
    //                    JIG_TEST_mavlink_serialPort5_Reinit();
                        JIG_TEST_console_write(str);
                        
                        sprintf(buff, "timeOut count : %d", count_timeOut);
                        JIG_TEST_console_write(buff);
                        ret = true;
                    }
                }
            }
        }
    }

    return ret;
}

#endif

/** @brief gimbal_FSTD_get_gimbal_startup_calib_motor
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor(void)
{
    bool ret = false;
    uint8_t motor_calib = JIG_TEST_mavlink_gimbal_get_state_calib_motor();
    
    if(motor_calib == 1)
    {
        JIG_TEST_console_write("GIMBAL_CALIB_MOTOR\n");
        
        /// set state startup calib
        gimbal_FSTD_global.gimbal_startup_calib = 1;
    }
    else
    {
        ret = true;
    }
    
    
    return ret;
}

/** @brief gimbal_FSTD_get_gimbal_startup_calib_imu
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu(void)
{
    bool ret = false;
    uint8_t imu_calib = JIG_TEST_mavlink_gimbal_get_state_calib_imu();
    
    if(imu_calib == 1)
    {
        JIG_TEST_console_write("GIMBAL_CALIB_IMU\n");
        
        /// set state startup calib
        gimbal_FSTD_global.gimbal_startup_calib = 2;
    }
    else
    {
        ret = true;
    }
    
    
    return ret;
}


/** @brief get_value_from_param
    @return bool
*/
static uint32_t JIG_TEST_gimbal_FSTD_get_value_from_param(uint16_t param_index, char *param_id)
{
    uint32_t value = 0;
    static uint32_t timeRequest = 0;
    
    if(HAL_GetTick() - timeRequest > 100 || timeRequest == 0)
    {
        timeRequest = HAL_GetTick();
        
        /// request from gimbal with param index and param id
        JIG_TEST_mavlink_gimbal_send_param_request_read(param_index, param_id);
        
//        mavlink_gimbal_COM2.param_value.param_index = 0;
//        mavlink_gimbal_COM2.param_value.param_value = 0;    
    }
    else
    {
        /// compare index reciever
        if(mavlink_gimbal_COM2.param_value.param_index == param_index)
        {
            value = mavlink_gimbal_COM2.param_value.param_value;
            
            timeRequest = 0;
        }
    }
    
    return value;
}

/** @brief set_calib_startup_imu_status
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_set_calib_startup_imu_status(bool status)
{
    bool ret = false;
    static bool getValue = false;
    static uint32_t value = 0;
    uint16_t param_index = 50;
    char *param_id = "SKIP_GYRO_CALIB";
    static uint32_t timeRequest = 0;
    static uint32_t timeSendRequest = 0;
    
    if(value == 0)
    {
        value = JIG_TEST_gimbal_FSTD_get_value_from_param(param_index, param_id);
        
        if(mavlink_gimbal_COM2.param_value.param_index == param_index)
        {
            getValue = true;
            
            if(value == 0)
            ret = true;
            
            char buff[100];
            sprintf(buff, "[set_calib_startup_imu_status] get value param %s ---> value : %5d\n", param_id, value);
            JIG_TEST_console_write(buff);
            HAL_Delay(50);
            
            mavlink_gimbal_COM2.param_value.param_index = 0;
            mavlink_gimbal_COM2.param_value.param_value = 0;
        }
    }
    else
    {
        static uint8_t firstRun = 0;
        
        if(firstRun == 0)
        {
            firstRun = 1;
            
            if(value == 0)
            {
                char buff[100];
                sprintf(buff, "[set_calib_startup_imu_status] turn off befor : %d\n", status); /// 1 : turn on | 0 : turn off
                JIG_TEST_console_write(buff);
                HAL_Delay(50);
                
                ret = true;
            }
            
            char buff[100];
            sprintf(buff, "[set_calib_startup_imu_status] reciever value param %s ---> value : %5d\n", param_id, value);
            JIG_TEST_console_write(buff);
            HAL_Delay(50);
        }
        else if(firstRun == 1)
        {
            firstRun = 2;
            /*
                CALIB_GYRO_AT_STARTUP_MASK_ON = 0x1000
                CALIB_GYRO_AT_STARTUP_MASK_OFF = 0x0000
            */
            uint32_t temp = value & 0x1000;
            uint32_t calib_flag = 0;
            
            if(status == true)
            {
                if(temp == 0x1000)
                {
                    calib_flag = 0x0000;
                }
                else
                {
                    calib_flag = 0x1000;
                }
            }
            else
            {
                if(temp == 0x1000)
                {
                    calib_flag = 0x1000;
                }
                else
                {
                    calib_flag = 0x0000;
                }
            }
            
            value ^= calib_flag;
            
            char buff[100];
            sprintf(buff, "[set_calib_startup_imu_status] add value param %s ---> value : %5d | %5d | %5d\n", param_id, value, calib_flag, status);
            JIG_TEST_console_write(buff);
            HAL_Delay(50);
        }
        
        if(mavlink_gimbal_COM2.param_value.param_index == param_index)
        {
            
            char buff[100];
            sprintf(buff, "[set_calib_startup_imu_status] status : %d\n", status); /// 1 : turn on | 0 : turn off
            JIG_TEST_console_write(buff);
            HAL_Delay(50);
            
            value = 0;
            getValue = false;
            timeRequest = 0;
            timeSendRequest = 0;
            firstRun = 0;
            
            ret = true;
        }
        else
        {
            if(firstRun == 2)
            {
                if(HAL_GetTick() - timeRequest > 500 || timeRequest == 0)
                {
                    timeRequest = HAL_GetTick();
                    
                    /// write to gimbal param SKIP_GYRO_CALIB
                    JIG_TEST_mavlink_gimbal_set_param(value, param_id);
                    
                    char buff[100];
                    sprintf(buff, "[set_calib_startup_imu_status] set value param %s\n", param_id);
                    JIG_TEST_console_write(buff);
                    HAL_Delay(50);
                }
                else
                {
                    if(HAL_GetTick() - timeSendRequest > 100 || timeSendRequest == 0)
                    {
                        timeSendRequest = HAL_GetTick();
                        
                        /// request from gimbal with param index and param id
                        JIG_TEST_mavlink_gimbal_send_param_request_read(param_index, param_id);
                    }
                }
            }
        }
    }
    
    return ret;
}

/** @brief gimbal_FSTD_control_angle
    @return true : da control xong
            false : dang control
*/
static bool JIG_TEST_gimbal_FSTD_control_angle(bool enable, JIG_TEST_mavlink_gimbal_t *gimbal_channel, control_gimbal_state_t gimbal_state, uint8_t type_control, uint8_t comm_channel)
{
    bool ret = false;
    static uint32_t count_state;
    static uint8_t mode;
    static control_gimbal_state_t state;
    static bool type_control_loop;
    char buff_control_gimbal[100];
    sprintf(buff_control_gimbal, "\nCONTROL_GIMBAL_MAVLINK ---->");
    
    uint16_t command = gimbal_channel->ack.command;
    
    mode = gimbal_channel->status.mode;
    
    if(type_control == 1)
    state = gimbal_state;
    else
    {
        if(type_control_loop == false)
        {
            type_control_loop = true;
            
            state = STATE_IDLE;
        }
    }
    
    if(enable == true)
    {
        if(state == STATE_IDLE)
        {
            state = STATE_SET_GIMBAL_OFF;
        }
        else if(state == STATE_SET_GIMBAL_OFF)
        {
            
            if(mode == GIMBAL_STATE_FOLLOW_MODE)
            {
                JIG_TEST_mavlink_gimbal_set_mode(LOCK_MODE);
                
                if(++count_state > 100000)
                {
                    count_state = 0;
                    
                    JIG_TEST_console_write(buff_control_gimbal);
                    JIG_TEST_console_write("setting gimbal to lock mode \n");
                }
                
                if(command == COMMAND_SETTING_MODE)
                {
                    /// next state
                    state = STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE;
                    
                    JIG_TEST_console_write(buff_control_gimbal);
                    JIG_TEST_console_write("switch gimbal to lock mode | nSTATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE\n");
                    
                    /// reset count state
                    count_state = 0;
                    
                    // reset command
                    command = gimbal_channel->ack.command = 0;
                }
            }
            else if(mode == GIMBAL_STATE_LOCK_MODE)
            {
//                    JIG_TEST_mavlink_gimbal_set_mode(FOLLOW_MODE);
//                    if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
//                    {
//                        /// set send msg
//                        gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_MODE;
//                        
//                        /// set value msg
//                        gimbal_FSTD.set_mode = FOLLOW_MODE;
//                    }
//                    
//                    if(++count_state > 100000)
//                    {
//                        count_state = 0;
//                        
//                        JIG_TEST_console_write(buff_control_gimbal);
//                        JIG_TEST_console_write("setting gimbal to follow mode \n");
//                    }
                
//                    if(command == COMMAND_SETTING_MODE)
//                    {
                    /// next state
                    state = STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE;
                    
                    JIG_TEST_console_write(buff_control_gimbal);
                    JIG_TEST_console_write("switch gimbal to follow mode | nSTATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE\n");
                    
//                        /// reset count state
//                        count_state = 0;
//                        
//                        // reset command
//                        command = gimbal_channel->ack.command = 0;
//                    }
            }
        }
        else if(state == STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE)
        {
            char buff[100];
            
            // Set gimbal move to 
          #if (JIG_TEST_ID == 0x34)
            float setpoint_pitch  = 30.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 0.0;
          #else
            
            float setpoint_pitch  = 40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 170.0;
            
          #endif
            
            if(++count_state > 100000)
            {
                count_state = 0;

                sprintf(buff, "Control gimbal's yaw cw follow mode! %d -- | -- %d\n", gimbal_channel->ack.command, gimbal_channel->ack.result);
                JIG_TEST_console_write(buff_control_gimbal);
                JIG_TEST_console_write(buff);
                
                /// set command gimbal move
                if(comm_channel == 2)
                {
                    JIG_TEST_mavlink_gimbal_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                else if(comm_channel == 4)
                {
                    JIG_TEST_mavlink_gimbal_com4_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                
            }

            if(get_timeOut(3000, JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL))
            {
              #if (JIG_TEST_ID == 0x34)
                
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5)
                
              #else
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                int16_t delta_yaw = (int16_t)(gimbal_channel->mount_val.yaw - setpoint_yaw);
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5 && abs(delta_yaw) < 5)
              #endif
                {
                    /// next state
                    state = STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW;
                    
                    JIG_TEST_console_write(buff_control_gimbal);
                    JIG_TEST_console_write("Control gimbal's yaw cw follow mode! ----- DONE----- %d\n");
                    
                    // reset command
                    command = gimbal_channel->ack.command = 0;
                    
                    /// reset count state
                    count_state = 0;
                }
            }
        }
        else if(state == STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW)
        {
            char buff[100];
            
            // Set gimbal move to 
          #if (JIG_TEST_ID == 0x34)
            float setpoint_pitch  = -45.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 0.0;
          #else
            
            float setpoint_pitch  = -40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = -170.0;
            
          #endif
            
            if(++count_state > 100000)
            {
                count_state = 0;
                
                sprintf(buff, "Control gimbal's yaw ccw follow mode! %d\n -- | -- %d\n", gimbal_channel->ack.command, gimbal_channel->ack.result);
                JIG_TEST_console_write(buff_control_gimbal);
                JIG_TEST_console_write(buff);
                
                /// set command gimbal move
                /// set command gimbal move
                if(comm_channel == 2)
                {
                    JIG_TEST_mavlink_gimbal_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                else if(comm_channel == 4)
                {
                    JIG_TEST_mavlink_gimbal_com4_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                
            }
            
            if(get_timeOut(3000, JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL))
            {
              #if (JIG_TEST_ID == 0x34)
                
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5)
                
              #else
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                int16_t delta_yaw = (int16_t)(gimbal_channel->mount_val.yaw - setpoint_yaw);
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5 && abs(delta_yaw) < 5)
              #endif
                {
                    /// next state
                    state = STATE_SET_CTRL_GIMBAL_SPEED_MODE;
                    
                    JIG_TEST_console_write(buff_control_gimbal);
                    JIG_TEST_console_write("Control gimbal's yaw ccw follow mode! ----- DONE----- %d\n");
                    
                    // reset command
                    command = gimbal_channel->ack.command = 0;
                }
            }
        }
        else if(state == STATE_SET_CTRL_GIMBAL_SPEED_MODE)
        {
            ret = true;
            
            state = STATE_IDLE;
        }
    }
    
    return ret;
}


#endif
/**
    @}
*/

/** @brief gimbal_FSTD_v2_controlWithCom4
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_controlWithCom4(void)
{
    JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM4, STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW, 1, 4);
}

/** @group JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_CLOUD_DATA
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_CLOUD_DATA
#define JIG_TEST_GIMBAL_FSTD_V2_CMD_WITH_CLOUD_DATA

/** @brief gimbal_FSTD_v2_get_cloudData_login
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_cloudData_login(void)
{
    return gimbal_FSTD_global.cloudData_command.get_login;
}

/** @brief gimbal_FSTD_v2_get_cloudData_scan_barCode
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_cloudData_scan_barCode(void)
{
    return gimbal_FSTD_global.cloudData_command.get_bardCode;
}

/** @brief gimbal_FSTD_v2_get_cloudData_start
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_cloudData_start(void)
{
    return gimbal_FSTD_global.cloudData_command.get_start_stop;
}

/** @brief gimbal_FSTD_v2_get_cloudData_heartbeatReady
    @return bool
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_cloudData_heartbeatReady(void)
{
    return gimbal_FSTD_global.cloudData_command.get_heartbeatReady;
}

/** @brief gimbal_FSTD_v2_get_cmd_reset_system
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_cmd_reset_system(void)
{
    return gimbal_FSTD_global.cloudData_command.get_reset;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_V2_CONTROL_PROCESS
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_V2_CONTROL_PROCESS
#define JIG_TEST_GIMBAL_FSTD_V2_CONTROL_PROCESS

/** @brief gimbal_FSTD_v2_get_first_run
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_get_first_run(void)
{
    return gimbal_FSTD_private.fisrt_run;
}

/** @brief gimbal_FSTD_v2_get_error_heartbeat_com2
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_get_error_heartbeat_com2(void)
{
    return gimbal_FSTD_private.error_heartbeat_com2;
}

/** @brief gimbal_FSTD_v2_get_reset_system
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_get_reset_system(void)
{
    return gimbal_FSTD_global.get_reset_system;
}

/** @brief get_return_home
    @return true : da ve home
            false : dang ve home
*/
static bool JIG_TEST_gimbal_FSTD_get_return_home(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    bool ret = false;
    static uint8_t state;
    static uint8_t timeOut_returnHome;
    static uint32_t count_state;
    char *str = "\nSET_GIMBAL_HOME ---->";
    
    if(state == 0)
    {
        JIG_TEST_mavlink_gimbal_set_home();

        state = 1;
    }
    else if(state == 1)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            int16_t delta_pitch = (int16_t)gimbal_channel->mount_val.pitch;
            int16_t delta_roll = (int16_t)gimbal_channel->mount_val.roll;
            int16_t delta_yaw = (int16_t)gimbal_channel->mount_val.yaw;
            if(abs(delta_pitch) < 5 && abs(delta_roll) == 0 && abs(delta_yaw) < 5)
        #endif
        
        #if (JIG_TEST_ID == 0x34)
        
            int16_t delta_pitch = (int16_t)gimbal_channel->mount_val.pitch;
            int16_t delta_roll = (int16_t)gimbal_channel->mount_val.roll;
            if(abs(delta_pitch) < 5 && abs(delta_roll) == 0)
        #endif
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("gimbal is HOME\n");
            
            state = 0;
            
            ret = true;
        }
        else
        {
            if(++count_state > 100000)
            {
                count_state = 0;
                
                if(++timeOut_returnHome > 7)
                {
                    timeOut_returnHome = 0;
                    
                    JIG_TEST_console_write(str);
                    JIG_TEST_console_write("return home TimeOut ---> re_send msg return home\n");
                    
                    state = 0;
                }
                else
                {
                    JIG_TEST_console_write(str);
                    JIG_TEST_console_write("set home gimbal running\n");
                }
            }
        }
    }
    
    return ret;
}


/** @brief gimbal_FSTD_get_mode_control
    @return mode_test
*/
static JIG_TEST_gimbal_FSTD_mode_read_t JIG_TEST_gimbal_FSTD_get_mode_control(JIG_TEST_mavlink_gimbal_t *gimbal_channel, JIG_TEST_gimbal_FSTD_mode_read_t mode_set, JIG_TEST_gimbal_FSTD_v2_mode_test_t mode_test)
{
    JIG_TEST_gimbal_FSTD_mode_read_t mode_read = JIG_TEST_GIMBAL_MODE_READ_IDLE;

    char *str = "\nSETTING_MODE_TEST --->";
    uint16_t len = strlen(str);
    static bool retry_set_param;
    static uint8_t timeOut_set_mode;
    static uint8_t timeOut_try_again = 0;
    static bool app_mode_read[4];
    char buff[100];

    if((uint8_t)mode_test >= (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX) /// mode aux khong can request param ratio
    {
        sprintf(buff, "GIMBAL_RC_MODE_COM\n");
        mode_read = JIG_TEST_GIMBAL_MODE_READ_COM;
    }
    else
    {
        
        if(retry_set_param == false)
        {
            retry_set_param = true;

            JIG_TEST_mavlink_gimbal_set_param((float)mode_set, "RADIO_TYPE");
            HAL_Delay(5);
        }
        
        if(timeOut_set_mode > 2 && retry_set_param == true)
        {
            retry_set_param = false;
            timeOut_set_mode = 0;
            
            if(++timeOut_try_again > 4)
            {
                NVIC_SystemReset();
            }
            
            JIG_TEST_console_write(str);
            HAL_Delay(5);
            
            sprintf(buff, "param_index_r : %2d | %.2f timeOut set mode --- > set mode %d Try again .....\n"
            , gimbal_channel->param_value.param_index, gimbal_channel->param_value.param_value, mode_set);
            JIG_TEST_console_write(buff);
            
            JIG_TEST_mavlink_gimbal_set_param((float)mode_set, "RADIO_TYPE");
            HAL_Delay(5);
        }
        
//        /// request param radio type (mode_test)
//        if(get_timeOut(100, JIG_TEST_SEND_REQUEST_PARAM_RADIO_TYPE))
//        {
//            JIG_TEST_console_write(str);
//            JIG_TEST_console_write("send request param radio type\n");
//            
            JIG_TEST_mavlink_gimbal_send_param_request_read(28, "RADIO_TYPE");
//        }

        
        /// search mode test
        if(gimbal_channel->param_value.param_index == 28)
        {
            if(gimbal_channel->param_value.param_value == JIG_TEST_GIMBAL_MODE_READ_SBUS)
            {
                sprintf(buff, "GIMBAL_RC_MODE_SBUS\n");
                mode_read = JIG_TEST_GIMBAL_MODE_READ_SBUS;
                
                retry_set_param = false;
                
                if(app_mode_read[0] == false)
                {
                    app_mode_read[0] = true;
                    
                    /// write to console mode read
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write(buff);
                }

            }
            else if(gimbal_channel->param_value.param_value == JIG_TEST_GIMBAL_MODE_READ_PPM)
            {
                sprintf(buff, "GIMBAL_RC_MODE_PPM\n");
                mode_read = JIG_TEST_GIMBAL_MODE_READ_PPM;
                
                retry_set_param = false;
                
                if(app_mode_read[1] == false)
                {
                    app_mode_read[1] = true;
                    
                    /// write to console mode read
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write(buff);
                }
            }
            else if(gimbal_channel->param_value.param_value == JIG_TEST_GIMBAL_MODE_READ_CAN)
            {
                sprintf(buff, "GIMBAL_RC_MODE_CAN\n");
                mode_read = JIG_TEST_GIMBAL_MODE_READ_CAN;
                
                retry_set_param = false;
                
                if(app_mode_read[2] == false)
                {
                    app_mode_read[2] = true;
                    
                    /// write to console mode read
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write(buff);
                }
            }
            else if(gimbal_channel->param_value.param_value == JIG_TEST_GIMBAL_MODE_READ_COM)
            {
                sprintf(buff, "GIMBAL_RC_MODE_COM\n");
                mode_read = JIG_TEST_GIMBAL_MODE_READ_COM;
                
                retry_set_param = false;
                
                if(app_mode_read[3] == false)
                {
                    app_mode_read[3] = true;
                    
                    /// write to console mode read
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write(buff);
                }
            }
            else
            {
                mode_read = JIG_TEST_GIMBAL_MODE_READ_IDLE;
            }
        }

    }
    
    if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_PRINTF_MODE_TEST))
    {
        /// count up tiemOut send set mode
        timeOut_set_mode++;
        
    }
    
    return mode_read;
}

/** @brief gimbal_FSTD_apply_mode_test
    @return true : da set xong mode
            false : dang set mode
*/
static bool JIG_TEST_gimbal_FSTD_apply_mode_test(JIG_TEST_gimbal_FSTD_mode_read_t mode)
{
    bool ret = false;
    
    if(gimbal_FSTD_private.gimbal_is_setMODE == false)
    {
        if(gimbal_FSTD_private.mode_read == mode)
        {
            gimbal_FSTD_private.gimbal_is_setMODE = true;
        }
        else
        {
            /// setting mode test cho gimbal
            gimbal_FSTD_private.mode_read = JIG_TEST_gimbal_FSTD_get_mode_control(&mavlink_gimbal_COM2, mode, gimbal_FSTD_global.mode_test);
        }
    }
    else if(gimbal_FSTD_private.gimbal_is_setMODE == true)
    {
        /// kiem tra gimbal da ON chua sau khi reboot
        if(mavlink_gimbal_COM2.status.mode != 0)
        {
            if(gimbal_FSTD_global.mode_test > JIG_TEST_GIMBAL_V2_MODE_CAN)
            {
                JIG_TEST_mavlink_gimbal_set_rc_input(MAVLINK_CONTROL_MODE);
            }
            else
            {
                JIG_TEST_mavlink_gimbal_set_rc_input(REMOTE_CONTROL_MODE);
            }

            if(mavlink_gimbal_COM2.ack.command == COMMAND_DO_MOUNT_CONFIG || mavlink_gimbal_COM2.ack.command == COMMAND_DO_MOUNT_CONTROL)
            {
                /// reset cac flag cho lan test sau
                gimbal_FSTD_private.gimbal_is_setMODE = false;
                
                mavlink_gimbal_COM2.ack.command = 0;
                
                ret = true;
            }
        }
    }
    
    return ret;
}


/** @brief gimbal_FSTD_check_mode_test_rc_result
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(JIG_TEST_mavlink_gimbal_t *mavlink_channel, JIG_TEST_gimbal_FSTD_v2_mode_test_t mode_test)
{
    bool ret = false;
    char buff[100];
    
    JIG_TEST_console_write("[check_mode_test_rc_result]");
    sprintf(buff, "tilt angle : %f", mavlink_channel->mount_val.pitch);
    JIG_TEST_console_write(buff);
    
    if(mode_test == JIG_TEST_GIMBAL_V2_MODE_SBUS)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            if(mavlink_channel->mount_val.pitch < -80 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -150)
            {
                ret = true;
            }
        
        #endif
        
        #if (JIG_TEST_ID == 0x34)
        
            int16_t tilt_angle = (int16_t)mavlink_channel->mount_val.pitch;
            int16_t roll_angle = (int16_t)mavlink_channel->mount_val.roll;
        
            if(abs(tilt_angle) > 80 && (uint16_t)abs(roll_angle) == 0)
            {
                ret = true;
            }
        
        #endif
    }
    else if(mode_test == JIG_TEST_GIMBAL_V2_MODE_PPM)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            if(mavlink_channel->mount_val.pitch > 30 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw > 150)
            {
                ret = true;
            }
        
        #endif
        
        #if (JIG_TEST_ID == 0x34)
        
            int16_t tilt_angle = (int16_t)mavlink_channel->mount_val.pitch;
            int16_t roll_angle = (int16_t)mavlink_channel->mount_val.roll;
        
            if(abs(tilt_angle) > 25 && (uint16_t)abs(roll_angle) == 0)
            {
                ret = true;
            }
        
        #endif
    }
    else if(mode_test == JIG_TEST_GIMBAL_V2_MODE_CAN)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            if(mavlink_channel->mount_val.pitch < -80 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -150)
            {
                ret = true;
            }
        
        #endif
            
        #if (JIG_TEST_ID == 0x34)
        
            int16_t tilt_angle = (int16_t)mavlink_channel->mount_val.pitch;
            int16_t roll_angle = (int16_t)mavlink_channel->mount_val.roll;
        
            if(abs(tilt_angle) > 80 && (uint16_t)abs(roll_angle) == 0)
            {
                ret = true;
            }
        
        #endif
    }
    else if(mode_test == JIG_TEST_GIMBAL_V2_MODE_COM)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            if(mavlink_channel->mount_val.pitch > 30 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw > 120)
            {
                ret = true;
            }
        
        #endif
        
        #if (JIG_TEST_ID == 0x34)
        
            int16_t tilt_angle = (int16_t)mavlink_channel->mount_val.pitch;
            int16_t roll_angle = (int16_t)mavlink_channel->mount_val.roll;
        
            if(abs(tilt_angle) > 25 && (uint16_t)abs(roll_angle) == 0)
            {
                ret = true;
            }
        
        #endif
    }
    else if(mode_test == JIG_TEST_GIMBAL_V2_MODE_COM4)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x101 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
        
            if(mavlink_channel->mount_val.pitch < -20 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -120)
            {
                ret = true;
            }
        
        #endif
        
        #if (JIG_TEST_ID == 0x34)
        
            int16_t tilt_angle = (int16_t)mavlink_channel->mount_val.pitch;
            int16_t roll_angle = (int16_t)mavlink_channel->mount_val.roll;
        
            if(abs(tilt_angle) > 35 && (uint16_t)abs(roll_angle) == 0)
            {
                ret = true;
            }
        
        #endif
    }
    else if(mode_test == JIG_TEST_GIMBAL_V2_MODE_VIBRATE)
    {

    }
    
    return ret;
}

/** @brief gimbal_FSTD_mode_test_process
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_mode_test_process(JIG_TEST_gimbal_FSTD_v2_private_t *FSTD, JIG_TEST_gimbal_FSTD_v2_mode_test_t mode_test, JIG_TEST_gimbal_FSTD_mode_read_t mode_read, uint16_t time)
{
    bool ret = false;
    char *str = "\nMODE_TEST_PROCESS --->";
    
    FSTD->time_mode_test = time;
    
    if(FSTD->gimbal_is_home == false)
    {
        FSTD->gimbal_is_home = JIG_TEST_gimbal_FSTD_get_return_home(&mavlink_gimbal_COM2);
    }
    else
    {
        if(FSTD->apply_mode_test[mode_test] == false)
        {
            FSTD->apply_mode_test[mode_test] = JIG_TEST_gimbal_FSTD_apply_mode_test(mode_read);
        }
        else
        {
            /// turn on motor sau khi chuyen mode motor off
            if(mavlink_gimbal_COM2.status.mode == 0)
            {
                /// set mavlink cmd turn on motor
                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
            }
            
            /// set test running tinh thoi gian timeOut
            FSTD->is_test_running = true;
            
            /// mode test process
            FSTD->mode_test_process[mode_test] = true;

            if(FSTD->time_run_test > FSTD->time_mode_test)
            {
                /// check result 
                if(mode_test < ((uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX))
                gimbal_FSTD_global.result_mode_test[mode_test] = JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(&mavlink_gimbal_COM2, mode_test);
                
                JIG_TEST_console_write(str);
                JIG_TEST_console_write("late results !!!\n");
                
                /// thong bao test xong
                FSTD->test_done[mode_test] = true;
                
                /// reset mode test process
                FSTD->mode_test_process[mode_test] = false;
                
                /// reset cac flag cho lan test sau
                FSTD->gimbal_is_home = false;
                
                /// reset mavlink ack command
                mavlink_gimbal_COM2.ack.command = 0;
                
                /// tra ve gia tri dung
                ret = true;
            }
            else
            {
                if(mode_test < JIG_TEST_GIMBAL_V2_MODE_AUX)
                {
                    if(FSTD->time_run_test > 6)
                    {
                        int16_t delta_pitch = (int16_t)(mavlink_gimbal_COM2.mount_val.pitch);
                        int16_t delta_roll = (int16_t)(mavlink_gimbal_COM2.mount_val.roll);
                        int16_t delta_yaw = (int16_t)(mavlink_gimbal_COM2.mount_val.yaw);
                        
                        /// neu dieu khien den goc truoc thoi gian timeOut thi cho ktra goc luoon
                      #if (JIG_TEST_ID == 0x34)
                        bool direct = false;
                        uint8_t angleCompare = 0;
                        
                        if(mavlink_gimbal_COM2.mount_val.pitch > 0)
                        {
                            direct = true;
                        }
                        
                        if(direct == true)
                        {
                            angleCompare = 20;
                        }
                        else
                        {
                            if(mode_test == JIG_TEST_GIMBAL_V2_MODE_COM4)
                            {
                                angleCompare = 30;
                            }
                            else
                            {
                                angleCompare = 70;
                            }
                        }
                        
                        if(abs(delta_pitch) > angleCompare && abs(delta_roll) == 0)
                      #else
                        if(abs(delta_pitch) > 20 && abs(delta_roll) == 0 && abs(delta_yaw) > 150)
                      #endif
                        {
                            
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("get results soon !!!\n");
                            
                            /// check result 
                            gimbal_FSTD_global.result_mode_test[mode_test] = JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(&mavlink_gimbal_COM2, mode_test);
                            
                            /// thong bao test xong
                            FSTD->test_done[mode_test] = true;
                            
                            /// reset mode test process
                            FSTD->mode_test_process[mode_test] = false;
                            
                            /// reset cac flag cho lan test sau
                            FSTD->gimbal_is_home = false;
                            
                            /// reset mavlink ack command
                            mavlink_gimbal_COM2.ack.command = 0;
                            
                            /// tra ve gia tri dung
                            ret = true;
                        }
                    }
                }
            }
        }
    }
    
    return ret;
}

/** @brief gimbal_FSTD_v2_aux_test
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_aux_test(void)
{
    char *str = "AUX_TEST_RESULT --->";
    static uint8_t count_timeOut_aux = 0;
    
    if(get_timeOut(1, JIG_TEST_GIMBAL_FSTD_TEST_AUX) && gimbal_FSTD_private.aux_test_done == false)
    {
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x34)
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S5_GPIO_Port, AUX_S5_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[0] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S4_GPIO_Port, AUX_S4_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[1] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S3_GPIO_Port, AUX_S3_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[2] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S2_GPIO_Port, AUX_S2_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[3] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_SET);
        #endif
        
        #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S7_GPIO_Port, AUX_S7_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[0] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S6_GPIO_Port, AUX_S6_Pin) == false)
            {
                gimbal_FSTD_private.aux_test_count[1] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_SET);
        
        #endif
    }
    
    if(get_timeOut(100, JIG_TEST_GIMBAL_FSTD_RESULT_AUX))
    {
        if(++count_timeOut_aux >= 30)
        {
            uint16_t max_count_aux = 2350; /// freq 
            
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            
            /*Configure GPIO pins : AUX_S2_Pin AUX_S3_Pin AUX_S4_Pin AUX_S5_Pin Output*/
            GPIO_InitStruct.Pin = AUX_S2_Pin|AUX_S3_Pin|AUX_S4_Pin|AUX_S5_Pin|AUX_S6_Pin|AUX_S7_Pin|AUX_S8_Pin|AUX_S9_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(AUX_S5_GPIO_Port, AUX_S5_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S4_GPIO_Port, AUX_S4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S3_GPIO_Port, AUX_S3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AUX_S2_GPIO_Port, AUX_S2_Pin, GPIO_PIN_SET);
            
            /// set flag test done aux
            gimbal_FSTD_private.aux_test_done = true;
            
            if(gimbal_FSTD_private.aux_test_count[0] >= max_count_aux)
            {
                HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(AUX_S5_GPIO_Port, AUX_S5_Pin, GPIO_PIN_RESET);
                
                if(gimbal_FSTD_private.aux_test_count[1] >= max_count_aux)
                {
                    HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(AUX_S4_GPIO_Port, AUX_S4_Pin, GPIO_PIN_RESET);
                    
                    #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x34)
                        if(gimbal_FSTD_private.aux_test_count[2] >= max_count_aux)
                        {
                            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(AUX_S3_GPIO_Port, AUX_S3_Pin, GPIO_PIN_RESET);
                            if(gimbal_FSTD_private.aux_test_count[3] >= max_count_aux)
                            {
                                char buff[100];
                                
                                gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX] = true;
                                gimbal_FSTD_private.aux_test_result = true;
                                JIG_TEST_console_write(str);
                                
                                sprintf(buff, "FSTD (AUX) ---> S9S5 : %d | S8S4 : %d | S7S3 : %d | S6S2 : %d | Total : %d | %d\n"\
                                , gimbal_FSTD_private.aux_test_count[0]
                                , gimbal_FSTD_private.aux_test_count[1]
                                , gimbal_FSTD_private.aux_test_count[2]
                                , gimbal_FSTD_private.aux_test_count[3]
                                , gimbal_FSTD_private.aux_test_result, 
                                gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                                
                                JIG_TEST_console_write(buff);
                                
                                HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_RESET);
                                HAL_GPIO_WritePin(AUX_S2_GPIO_Port, AUX_S2_Pin, GPIO_PIN_RESET);
                            }
                        }
                    #endif
                    
                    #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
                        char buff[100];
                        
                        gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX] = true;
                        gimbal_FSTD_private.aux_test_result = true;
                        
                        JIG_TEST_console_write(str);
                        
                        sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
                        , gimbal_FSTD_private.aux_test_count[0]
                        , gimbal_FSTD_private.aux_test_count[1]
                        , gimbal_FSTD_private.aux_test_done
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_console_write(buff);
                    #endif
                }
                else
                {
                    #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
                        char buff[100];
                    
                        JIG_TEST_console_write(str);
                        
                        sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
                        , gimbal_FSTD_private.aux_test_count[0]
                        , gimbal_FSTD_private.aux_test_count[1]
                        , gimbal_FSTD_private.aux_test_done
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_console_write(buff);
                    #endif
                }
            }
            else
            {
                
                #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x34)
                
                    char buff[100];

                    JIG_TEST_console_write(str);
                    
                    sprintf(buff, "FSTD (AUX) ---> S9S5 : %d | S8S4 : %d | S7S3 : %d | S6S2 : %d | Total : %d | %d\n"\
                    , gimbal_FSTD_private.aux_test_count[0]
                    , gimbal_FSTD_private.aux_test_count[1]
                    , gimbal_FSTD_private.aux_test_count[2]
                    , gimbal_FSTD_private.aux_test_count[3]
                    , gimbal_FSTD_private.aux_test_result, 
                    gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                    
                    JIG_TEST_console_write(buff);
                
                #endif
                
                #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
                    char buff[100];
                
                    JIG_TEST_console_write(str);
                    
                    sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
                    , gimbal_FSTD_private.aux_test_count[0]
                    , gimbal_FSTD_private.aux_test_count[1]
                    , gimbal_FSTD_private.aux_test_done
                    , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                    
                    JIG_TEST_console_write(buff);
                #endif
            }
        }
    }
}

/** @brief gimbal_FSTD_v2_sensor_axis
    @return double
*/
static double JIG_TEST_gimbal_FSTD_v2_sensor_axis(uint8_t numberOfaxis, int16_t gyro_axis, JIG_TEST_mavlink_gimbal_t* mavlinkChannel)
{
    char *str = "\nVIBRATE --->";
    char *BMI = "BMI160";
    char *ICM = "ICM42688";
    
    /*             . . . . . . . . . . . .
                  . E(x - x1)(x - x1)
        S = .    . ------------------
             .  .       n - 1
              .
        Trong do :
        S : ket qua
        x : input
       x1 : tong cac input
        n : so lan lay trung binh
    */

    static int16_t      sum_gyro            = 0;
    static uint16_t     count_gyro          = 0;
    static uint16_t     count_std_dev       = 0;
    static double       mid_gyro            = 0;
    static double       sum_std_dev_gyro    = 0;
    static double       std_dev_gyro_result = 0;
    double              result              = 0;
    static uint8_t      state               = 0;
        
        if(state == 0) /// tinh trung binh
        {
            /// kiem tra co ket qua raw imu moi
            if(mavlinkChannel->raw_imu.flag_raw_imu_message == true)
            {
                mavlinkChannel->raw_imu.flag_raw_imu_message = false;
                
                if(count_gyro ++ < 10)
                {
                    /// tinh tong cac gia tri raw imu
                    sum_gyro += gyro_axis;
                }
                else
                {
                    /// tinh trung binh raw imu
                    mid_gyro = sum_gyro / 10;
                    
                    /// next state
                    state = 1;
                }
            }
        }
        else if(state == 1) /// tinh do lech chuan
        {
            if(mavlinkChannel->raw_imu.flag_raw_imu_message == true)
            {
                mavlinkChannel->raw_imu.flag_raw_imu_message = false;
                
                double temp = gyro_axis - mid_gyro;
                
                if(count_std_dev ++ < 20)
                {
                    /// tinh tong cac binh phuong cac raw imu
                    sum_std_dev_gyro += temp * temp;
                }
                else
                {
                    /// tinh can bac 2 
                    std_dev_gyro_result = sqrt(sum_std_dev_gyro / 20);
                    
                    /// next state 
                    state = 2;
                }
            }
        }
        else if(state == 2)
        {
            char imu_id[200];
            
            JIG_TEST_console_write(str);
            
            if(JIG_TEST_mavlink_gimbal_get_sensor_name(mavlinkChannel) == GREMSY_SENSOR_BMI160)
            {
                sprintf(imu_id, "imu_name : %s |", BMI);
            }
            
            if(JIG_TEST_mavlink_gimbal_get_sensor_name(mavlinkChannel) == GREMSY_SENSOR_ICM42688)
            {
                sprintf(imu_id, "imu_name : %s |", ICM);
            }
            
            JIG_TEST_console_write(imu_id);
            
            if(numberOfaxis == 1)
            {
                sprintf(imu_id, " std_dev_gyro_x_result : %1.2f | sum_std_dev_gyro_x : %1.3f | sum_gyro_x : %3d\n"
                , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                
                gimbal_FSTD_private.vibrate.countDelta_x++;
            }
            else if(numberOfaxis == 2)
            {
                sprintf(imu_id, " std_dev_gyro_y_result : %1.2f | sum_std_dev_gyro_y : %1.3f | sum_gyro_y : %3d\n"
                , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                
                gimbal_FSTD_private.vibrate.countDelta_y++;
            }
            else if(numberOfaxis == 3)
            {
                sprintf(imu_id, "std_dev_gyro_z_result : %1.2f | sum_std_dev_gyro_z : %1.3f | sum_gyro_z : %3d\n\n"
                , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                
                gimbal_FSTD_private.vibrate.countDelta_z++;
            }

            
            JIG_TEST_console_write(imu_id);
            
            /// get result
            result = std_dev_gyro_result;
            
            /// reset all variable
            sum_gyro = 0;
            count_gyro = 0;
            count_std_dev = 0;
            mid_gyro = 0;
            sum_std_dev_gyro = 0;
            std_dev_gyro_result = 0;
            state = 0;
            
        }
    return result;
}
/** @brief gimbal_FSTD_v2_vibrate
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_vibrate(JIG_TEST_mavlink_gimbal_t* mavlinkChannel)
{
    bool ret = false;
    
    char *str = "VIBRATE --->";

    static uint8_t state = 0;
    static bool test_loop = false;

    if(state == 0)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_x_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(1, mavlinkChannel->raw_imu.xgyro, mavlinkChannel);
        
        if(gimbal_FSTD_private.vibrate.std_dev_gyro_x_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_x_result = 0;
            
            /// next state
            state = 1;
        }
    }
    else if(state == 1)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_y_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(2, mavlinkChannel->raw_imu.ygyro, mavlinkChannel);
        
        if( gimbal_FSTD_private.vibrate.std_dev_gyro_y_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_y_result = 0;
            
            /// next state
            state = 2;
        }
    }
    else if(state == 2)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_z_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(3, mavlinkChannel->raw_imu.zgyro, mavlinkChannel);
        
        if(gimbal_FSTD_private.vibrate.std_dev_gyro_z_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_z_result = 0;
            
            /// next state
            if(test_loop == true)
            {
                state = 0;
            }
            else
            {
                char buff[100];
                
                if(JIG_TEST_mavlink_gimbal_get_sensor_name(mavlinkChannel) == GREMSY_SENSOR_BMI160)
                {
                    ///setting limit delta x, y, z
                    gimbal_FSTD_private.vibrate.limit_gx_delta = 12.00;
                    gimbal_FSTD_private.vibrate.limit_gy_delta = 8.5;
                    gimbal_FSTD_private.vibrate.limit_gz_delta = 8.5;
                }
                
                if(JIG_TEST_mavlink_gimbal_get_sensor_name(mavlinkChannel) == GREMSY_SENSOR_ICM42688)
                {
                    ///setting limit delta x, y, z
                    gimbal_FSTD_private.vibrate.limit_gx_delta = 10.00;
                    gimbal_FSTD_private.vibrate.limit_gy_delta = 8.5;
                    gimbal_FSTD_private.vibrate.limit_gz_delta = 9.00;
                }

                if(gimbal_FSTD_private.vibrate.checkDome == false)
                {
                    gimbal_FSTD_private.vibrate.checkDome = true;
                    
                    if(gimbal_FSTD_private.vibrate.std_dev_gyro_x_result < gimbal_FSTD_private.vibrate.limit_gx_delta)
                    {
                        if(gimbal_FSTD_private.vibrate.std_dev_gyro_y_result < gimbal_FSTD_private.vibrate.limit_gy_delta)
                        {
                            if(gimbal_FSTD_private.vibrate.std_dev_gyro_z_result < gimbal_FSTD_private.vibrate.limit_gz_delta)
                            {
                                ret = true;
                                
                                gimbal_FSTD_private.vibrate.checkResult = true;
                                
                                gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = true;
                                
                                /// khong tinh gia tri delta nua
                                gimbal_FSTD_private.vibrate.is_gimbalHome = false;
                            }
                        }
                    }
                }
                
                JIG_TEST_console_write(str);
                sprintf(buff, "result : %d | %d | limit : %.2f %.2f %.2f\n"
                , gimbal_FSTD_private.vibrate.checkResult
                , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]
                , gimbal_FSTD_private.vibrate.limit_gx_delta
                , gimbal_FSTD_private.vibrate.limit_gy_delta
                , gimbal_FSTD_private.vibrate.limit_gz_delta);
                JIG_TEST_console_write(buff);
                
                /// next state none
                state = 3;
            }
            
        }
    }
    
    return ret;
}


/** @brief gimbal_FSTD_v2_vibrate
    @return none
*/
bool JIG_TEST_gimbal_FSTD_v2_vibrate_v2(void)
{
    bool ret = false;
    
    char *str = "VIBRATE --->";

    static uint8_t state = 0;
    static bool test_loop = false;

    if(state == 0)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_x_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(1, mavlink_gimbal_COM4.raw_imu.xgyro, &mavlink_gimbal_COM4);
        
        if(gimbal_FSTD_private.vibrate.std_dev_gyro_x_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_x_result = 0;
            
            /// next state
            state = 1;
        }
    }
    else if(state == 1)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_y_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(2, mavlink_gimbal_COM4.raw_imu.ygyro, &mavlink_gimbal_COM4);
        
        if( gimbal_FSTD_private.vibrate.std_dev_gyro_y_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_y_result = 0;
            
            /// next state
            state = 2;
        }
    }
    else if(state == 2)
    {
        gimbal_FSTD_private.vibrate.std_dev_gyro_z_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(3, mavlink_gimbal_COM4.raw_imu.zgyro, &mavlink_gimbal_COM4);
        
        if(gimbal_FSTD_private.vibrate.std_dev_gyro_z_result != 0)
        {
            if(test_loop == true)
            gimbal_FSTD_private.vibrate.std_dev_gyro_z_result = 0;
            
            /// next state
            if(test_loop == true)
            {
                state = 0;
            }
            else
            {
                char buff[100];
                
//                if(JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM4) == GREMSY_SENSOR_BMI160)
//                {
//                    ///setting limit delta x, y, z
//                    gimbal_FSTD_private.vibrate.limit_gx_delta = 12.00;
//                    gimbal_FSTD_private.vibrate.limit_gy_delta = 8.5;
//                    gimbal_FSTD_private.vibrate.limit_gz_delta = 8.5;
//                }
//                
//                if(JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM4) == GREMSY_SENSOR_ICM42688)
//                {
                    ///setting limit delta x, y, z
                    gimbal_FSTD_private.vibrate.limit_gx_delta = 10.00;
                    gimbal_FSTD_private.vibrate.limit_gy_delta = 8.5;
                    gimbal_FSTD_private.vibrate.limit_gz_delta = 9.00;
//                }

                if(gimbal_FSTD_private.vibrate.checkDome == false)
                {
                    gimbal_FSTD_private.vibrate.checkDome = true;
                    
                    if(gimbal_FSTD_private.vibrate.std_dev_gyro_x_result < gimbal_FSTD_private.vibrate.limit_gx_delta)
                    {
                        if(gimbal_FSTD_private.vibrate.std_dev_gyro_y_result < gimbal_FSTD_private.vibrate.limit_gy_delta)
                        {
                            if(gimbal_FSTD_private.vibrate.std_dev_gyro_z_result < gimbal_FSTD_private.vibrate.limit_gz_delta)
                            {
                                ret = true;
                                
                                gimbal_FSTD_private.vibrate.checkResult = true;
                                
                                gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = true;
                                
                                /// khong tinh gia tri delta nua
                                gimbal_FSTD_private.vibrate.is_gimbalHome = false;
                            }
                        }
                    }
                }
                
                JIG_TEST_console_write(str);
                sprintf(buff, "result : %d | %d | limit : %.2f %.2f %.2f\n"
                , gimbal_FSTD_private.vibrate.checkResult
                , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]
                , gimbal_FSTD_private.vibrate.limit_gx_delta
                , gimbal_FSTD_private.vibrate.limit_gy_delta
                , gimbal_FSTD_private.vibrate.limit_gz_delta);
                JIG_TEST_console_write(buff);
                
                /// next state none
                state = 3;
            }
            
        }
    }
    
    return ret;
}

/** @brief gimbal_FSTD_v2_set_gimbal_mapping_mode
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_set_gimbal_mapping_mode(void)
{
    bool ret = false;
    static uint32_t command = 0;
    static bool setting_param_mapping_angle = false;
    
    if(command == COMMAND_SETTING_MODE && mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
    {
        /// setting mapping angle 90
        if(setting_param_mapping_angle == false)
        {
            int16_t delta_pitch = (int16_t)(mavlink_gimbal_COM2.mount_val.pitch - (0));
            int16_t delta_roll = (int16_t)(mavlink_gimbal_COM2.mount_val.roll - 0);
            int16_t delta_yaw = (int16_t)(mavlink_gimbal_COM2.mount_val.yaw - 0);
            
            if(abs(delta_pitch) < 5 && abs(delta_roll) < 5 && abs(delta_yaw) < 5)
            {
                setting_param_mapping_angle = true;
                
                JIG_TEST_console_write("\n MAPPING_MODE --->setting mapping angle 0 -- DONE\n");
            }
            else
            {
                JIG_TEST_mavlink_gimbal_set_move(0, 0, 0, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                
                JIG_TEST_console_write("\n MAPPING_MODE --->setting mapping angle 0\n");
            }
            
            /// reset param value index
            mavlink_gimbal_COM2.param_value.param_index = 0;
        }
        else
        {
            ret = true;
        }
    }
    else
    {

        JIG_TEST_mavlink_gimbal_set_mode(LOCK_MODE);
        
        JIG_TEST_console_write("\n MAPPING_MODE --->setting mapping mode set cmd\n");

        /// ktra command ack
        command = mavlink_gimbal_COM2.ack.command;
        
    }
    
    return ret;
}

/** @brief gimbal_FSTD_setting_param_select
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_setting_param_select(uint16_t param_index, char *param_id, float param_value)
{
    bool ret = false;
    static bool set_param = false;
    static uint32_t count;
    static uint8_t timeOut = 0;
    char buff[100];
    
    if(set_param == false)
    {
        JIG_TEST_mavlink_gimbal_set_param(param_value, param_id);
        HAL_Delay(10);
        
        sprintf(buff, "\nSETTING_PARAM ---> value : %f | id : %s\n", param_value, param_id);
        JIG_TEST_console_write(buff);
        
        set_param = true;
    }
    else
    {
        if(++ count > 150000 || count == 0)
        {
            count = 1;
            
            JIG_TEST_mavlink_gimbal_send_param_request_read(param_index, param_id);
            HAL_Delay(5);
            
            /// ktra param index nhan voi param index set
            if(mavlink_gimbal_COM2.param_value.param_index == param_index)
            {
                /// ktra param value nhan voi param value set
                if(mavlink_gimbal_COM2.param_value.param_value == param_value)
                {
                    set_param = 0;
                    count = 0;
                    timeOut = 0;
                    
                    sprintf(buff, "\nSETTING_PARAM ---> value : %f | id : %s --- DONE\n", param_value, param_id);
                    JIG_TEST_console_write(buff);
                    
                    ret = true;
                }
            }
            
            /// ktra timeOut
            if(++timeOut > 5)
            {
                JIG_TEST_console_write("\nSETTING_PARAM ---> set param timeOut .... retry");
                
                set_param = 0;
                count = 0;
                timeOut = 0;
            }
        }
    }
    
    
    return ret;
}

/** @brief gimbal_FSTD_v2_vibrate_once_axis
    @return none
*/
static uint8_t JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(uint8_t *state)
{
    char buff[100];
    static float std_dev_gyro_x_result = 0;
    static float std_dev_gyro_y_result = 0;
    static float std_dev_gyro_z_result = 0;
    
    uint8_t result = 0;
    
    if(*state == 0)
    {
        std_dev_gyro_x_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(1, mavlink_gimbal_COM2.raw_imu.xgyro, &mavlink_gimbal_COM2);
        
        if(std_dev_gyro_x_result != 0)
        {
            /// next state
            *state = 1;
        }
    }
    else if(*state == 1)
    {
        std_dev_gyro_y_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(2, mavlink_gimbal_COM2.raw_imu.ygyro, &mavlink_gimbal_COM2);
        
        if(std_dev_gyro_y_result != 0)
        {
            /// next state
            *state = 2;
        }
    }
    else if(*state == 2)
    {
        std_dev_gyro_z_result = JIG_TEST_gimbal_FSTD_v2_sensor_axis(3, mavlink_gimbal_COM2.raw_imu.zgyro, &mavlink_gimbal_COM2);
        
        if(std_dev_gyro_z_result != 0)
        {
            *state = 3;
        }
    }
    else if(*state == 3)
    {
        sprintf(buff, "\nVIBRATE_S1v3 ---> x : %f | y : %f | z : %f\n", std_dev_gyro_x_result, std_dev_gyro_y_result, std_dev_gyro_z_result);
        
        if(std_dev_gyro_x_result <= 15)
        {
            result ++;
        }
        
        if(std_dev_gyro_y_result <= 15)
        {
            result ++;
        }
        
        if(std_dev_gyro_z_result <= 15)
        {
            result ++;
        }
        
        if(result == 3)
        {
            /// reset state for next test
            *state = 0;
        }
        
        JIG_TEST_console_write(buff);
    }
    
    return result;
}

/** @brief gimbal_FSTD_v2_setting_param_test_vibrate
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_test_vibrate_s1v3(uint8_t *state)
{
    bool ret = false;
    static bool apply_vibrate = false;
    static uint8_t state_vibrate = 0;
    /*
    {.value = 0, .value_param_get = 0, .index = 2, .param_id = "PITCH_P"},
    {.value = 0, .value_param_get = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .value_param_get = 0, .index = 8, .param_id = "YAW_P"},
    */
    if(*state == 0)
    {
        uint16_t param_index = 2;
        char *param_id = "PITCH_P";
        float param_value = 55;
        
        if(apply_vibrate == false)
        {
            if(JIG_TEST_gimbal_FSTD_setting_param_select(param_index, param_id, param_value) == true)
            {
                apply_vibrate = true;
            }
        }
        else
        {
            uint8_t result = 0;
            
            /// ktra trang thai test
            if(state_vibrate == 3)
            {
                
                result = JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
                
                if(result == 3)
                {
                    /// next state
                    *state = 1;
                    
                    state_vibrate = 0;
                    apply_vibrate = false;
                }
                else
                {
                    ret = true;
                }
            }
            else
            {
                JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
            }
        }
    }
    else if(*state == 1)
    {
        uint16_t param_index = 5;
        char *param_id = "ROLL_P";
        float param_value = 70;
        
        if(apply_vibrate == false)
        {
            if(JIG_TEST_gimbal_FSTD_setting_param_select(param_index, param_id, param_value) == true)
            {
                apply_vibrate = true;
            }
        }
        else
        {
            uint8_t result = 0;
            
            /// ktra trang thai test
            if(state_vibrate == 3)
            {
                
                result = JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
                
                if(result == 3)
                {
                    /// next state
                    *state = 2;
                    
                    state_vibrate = 0;
                    apply_vibrate = false;
                }
                else
                {
                    ret = true;
                }
            }
            else
            {
                JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
            }
        }
    }
    else if(*state == 2)
    {
        uint16_t param_index = 8;
        char *param_id = "YAW_P";
        float param_value = 80;
        
        if(apply_vibrate == false)
        {
            if(JIG_TEST_gimbal_FSTD_setting_param_select(param_index, param_id, param_value) == true)
            {
                apply_vibrate = true;
            }
        }
        else
        {
            uint8_t result = 0;

            /// ktra trang thai test
            if(state_vibrate == 3)
            {
                
                result = JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
                HAL_Delay(10);
                
                if(result == 3)
                {
                    /// next state
                    *state = 3;
                    
                    state_vibrate = 0;
                    apply_vibrate = false;
                }
                else
                {
                    ret = true;
                }
            }
            else
            {
                JIG_TEST_gimbal_FSTD_v2_vibrate_once_axis(&state_vibrate);
            }
        }
    }
    else if(*state == 3)
    {
        ret = true;
    }
        
    
    return ret;
}

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_set_stiffness_gimbal_20(void)
{
    bool ret = false;
    static uint8_t state = 0;
    
    if(state == 0)
    {
        if(JIG_TEST_gimbal_FSTD_setting_param_select(2, "PITCH_P", 20) == true)
        {
            /// next state
            state = 1;
        }
    }
    else if(state == 1)
    {
        if(JIG_TEST_gimbal_FSTD_setting_param_select(5, "ROLL_P", 20) == true)
        {
            /// next state
            state = 2;
        }
    }
    else if(state == 2)
    {
        if(JIG_TEST_gimbal_FSTD_setting_param_select(8, "YAW_P", 20) == true)
        {
            /// next state
            state = 3;
        }
    }
    else if(state == 3)
    {
        ret = true;
    }
    
    
    return ret;
}

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_all_mode_control_process(JIG_TEST_gimbal_FSTD_v2_private_t *FSTD)
{
    static bool enable_ppm;
    static uint8_t can_control_state;
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_SBUS] == true)
    {
        JIG_TEST_sbus_gimbal_process();
    }
    else
    {
        if(FSTD->test_done[JIG_TEST_GIMBAL_V2_MODE_SBUS] == true)
        JIG_TEST_sbus_gimbal_enable(false);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] == true)
    {
        if(enable_ppm == false)
        {
            enable_ppm = true;
            JIG_TEST_ppm_gimbal_enable(true);
        }
        
        JIG_TEST_ppm_gimbal_process();
    }
    else
    {
        if(FSTD->test_done[JIG_TEST_GIMBAL_V2_MODE_PPM] == true)
        JIG_TEST_ppm_gimbal_enable(false);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_CAN] == true)
    {
        
            if(can_control_state == 0)
            {
                JIG_TEST_can_dji_set_move(504, 504, true);
                
                /// next can state
                can_control_state = 1;
            }

        JIG_TEST_can_dji_process();
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] == true)
    {
        JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM2, STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE, 1, 2);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] == true)
    {
        JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM4, STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW, 1, 4);
    }
    else if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] == true)
    {
        JIG_TEST_gimbal_FSTD_v2_aux_test();
    }
    else if(FSTD->mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] == true)
    {
      #if (S1v3_VIRATE_TEST == 1)
        if(mavlink_gimbal_COM2.vehicle_system_id == 0x22)
      #else
        if(false)
      #endif
        {
            static uint32_t count_console = 0;
            static bool apply_virate = false;
            static bool set_default_stiffness = false;
            char buff[100];
            
            if(apply_virate == false)
            {
                if(++count_console > 100000 || count_console == 0)
                {
                    count_console = 1;

                    /// setting gimbal mapping mode
                    if(JIG_TEST_gimbal_FSTD_v2_set_gimbal_mapping_mode() == true)
                    {
                        apply_virate = true;
                    }
                }
            }
            else
            {
                if(set_default_stiffness == false)
                {
                    if(JIG_TEST_gimbal_FSTD_v2_set_stiffness_gimbal_20() == true)
                    {
                         set_default_stiffness = true;
                    }
                }
                else if (set_default_stiffness == true)
                {
                    
                    uint8_t result = JIG_TEST_gimbal_FSTD_v2_test_vibrate_s1v3(&gimbal_FSTD_global.vibrate_state_s1v3);
                    
                    
                    if(result == true) 
                    {
                        /// ktra trang thai test
                        if(gimbal_FSTD_global.vibrate_state_s1v3 == 3)
                        {
                            gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = true;
                        }
                        
                        /// set done test vibrate
                        gimbal_FSTD_private.time_run_test = 200;
                    }
                }
            }
        }
        else
        {
            JIG_TEST_gimbal_FSTD_v2_vibrate(&mavlink_gimbal_COM2);
        }
    }
}



/** @brief gimbal_FSTD_write_to_console_result_mode_test
    @return none
*/
static void JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test(char *mode_test, bool result)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    char buff[200];
    
    JIG_TEST_console_write(control_process);
    sprintf(buff,"%s : %d\n", mode_test, result);
    JIG_TEST_console_write(buff);
    HAL_Delay(50);
}


/** @brief gimbal_FSTD_v2_setting_stiffness
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_setting_stiffness(JIG_TEST_gimbal_FSTD_stiffness_type_t type)
{
    char buff[100];
    char *setting_stiffness = "\nSETTING_STIFFNESS --->";
    
    JIG_TEST_console_write(setting_stiffness);
    
    if(type == STIFFNESS_TYPE_FEED_BACK_IMU)
    {
        if(mavlink_gimbal_COM2.vehicle_system_id == 0x44) // t3v3
        {
            gimbal_param_setting[10].value = JIG_TEST_SETTING_PARAM_TEST_YAW_P_T3V3;
            gimbal_param_setting[11].value = JIG_TEST_SETTING_PARAM_TEST_ROLL_P_T3V3;
            gimbal_param_setting[12].value = JIG_TEST_SETTING_PARAM_TEST_PITCH_P_T3V3;
            
            sprintf(buff, "stiffness run feed back imu T3v3 : Pan:%f|Roll:%f|Tilt:%f\n"
            ,gimbal_param_setting[10].value
            ,gimbal_param_setting[11].value
            ,gimbal_param_setting[12].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        else if(mavlink_gimbal_COM2.vehicle_system_id == 0x22) // s1v3
        {
            gimbal_param_setting[10].value = JIG_TEST_SETTING_PARAM_TEST_YAW_P_S1V3;
            gimbal_param_setting[11].value = JIG_TEST_SETTING_PARAM_TEST_ROLL_P_S1V3;
            gimbal_param_setting[12].value = JIG_TEST_SETTING_PARAM_TEST_PITCH_P_S1V3;
            
            sprintf(buff, "stiffness run feed back imu S1v3 : Pan:%f|Roll:%f|Tilt:%f\n"
            ,gimbal_param_setting[10].value
            ,gimbal_param_setting[11].value
            ,gimbal_param_setting[12].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        else if(mavlink_gimbal_COM2.vehicle_system_id == 0x08) // t7
        {
            gimbal_param_setting[10].value = JIG_TEST_SETTING_PARAM_TEST_YAW_P_T7;
            gimbal_param_setting[11].value = JIG_TEST_SETTING_PARAM_TEST_ROLL_P_T7;
            gimbal_param_setting[12].value = JIG_TEST_SETTING_PARAM_TEST_PITCH_P_T7;
            
            sprintf(buff, "stiffness run feed back imu T7 : Pan:%f|Roll:%f|Tilt:%f\n"
            ,gimbal_param_setting[10].value
            ,gimbal_param_setting[11].value
            ,gimbal_param_setting[12].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        
        /// setting param stiffness
        for(uint8_t i = 10; i <= 12; i++)
        {
            JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[i].value, gimbal_param_setting[i].param_id);
            HAL_Delay(50);
        }
    }
    else if(STIFFNESS_TYPE_PROFILE_SHIP)
    {
        if(mavlink_gimbal_COM2.vehicle_system_id == 0x44) // t3v3
        {
            gimbal_user_profile_ship[4].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_P;
            gimbal_user_profile_ship[5].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_P;
            gimbal_user_profile_ship[6].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_P;
            
            sprintf(buff, "stiffness profile ship T3v3 : Tilt:%f|Roll:%f|Pan:%f\n"
            ,gimbal_user_profile_ship[4].value
            ,gimbal_user_profile_ship[5].value
            ,gimbal_user_profile_ship[6].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        else if(mavlink_gimbal_COM2.vehicle_system_id == 0x22) // s1v3
        {
            gimbal_user_profile_ship[4].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_P;
            gimbal_user_profile_ship[5].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_P;
            gimbal_user_profile_ship[6].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_P;
            
            sprintf(buff, "stiffness profile ship S1v3 : Tilt:%f|Roll:%f|Pan:%f\n"
            ,gimbal_user_profile_ship[4].value
            ,gimbal_user_profile_ship[5].value
            ,gimbal_user_profile_ship[6].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        else if(mavlink_gimbal_COM2.vehicle_system_id == 0x08) // t7
        {
            gimbal_user_profile_ship[4].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_P;
            gimbal_user_profile_ship[5].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_P;
            gimbal_user_profile_ship[6].value = JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_P;
            
            sprintf(buff, "stiffness profile ship T7 : Tilt:%f|Roll:%f|Pan:%f\n"
            ,gimbal_user_profile_ship[4].value
            ,gimbal_user_profile_ship[5].value
            ,gimbal_user_profile_ship[6].value);
            HAL_Delay(5);
            JIG_TEST_console_write(buff);
        }
        
        /// setting param stiffness
        for(uint8_t i = 4; i <= 6; i++)
        {
            JIG_TEST_mavlink_gimbal_set_param(gimbal_user_profile_ship[i].value, gimbal_user_profile_ship[i].param_id);
            HAL_Delay(50);
        }
    }
}

/** @brief gimbal_FSTD_run_feed_back_imu
    @return none
*/
static void JIG_TEST_gimbal_FSTD_run_feed_back_imu(void)
{
    static bool run_feed_back_imu;
    static uint32_t count_state;
    
    /// button press run gimbal move for mavlink
    if(JIG_TEST_button_state_feed_back_imu() == true)
    {
        run_feed_back_imu = true;
        
        /// setting lai stiffness test
        JIG_TEST_gimbal_FSTD_v2_setting_stiffness(STIFFNESS_TYPE_FEED_BACK_IMU);
        
        if(mavlink_gimbal_COM2.status.mode == 0)
        {
            /// on motor test feed back imu
            JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
        }
    }
    
    /// run move gimbal cw & ccw test feedback imu
    if(run_feed_back_imu == true)
    {
        if(JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM2, STATE_IDLE, 0, 2))
        {
            run_feed_back_imu = false;
            
            ///  set home gimbal
            JIG_TEST_mavlink_gimbal_set_home();
        }
        
        /// toggle led blue theo mode gimbal
        if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
        {
            if(++ count_state > 50000)
            {
                count_state = 0;
                
                HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
            }
        }
        else
        {
            HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
        }
    }
}

/** @brief gimbal_FSTD_back_to_scan_new_barCode
    @return none
*/
static void JIG_TEST_gimbal_FSTD_back_to_scan_new_barCode(void)
{
    char *back_to_scan_new_barCode = "\nBACK_TO_SCAN_BEW_BARCODE ---> ";
    
    /// reset all availables cho lan test sau
    if(JIG_TEST_gimbal_FSTD_v2_get_cmd_reset_system() == true)
    {
        /// khi test xong nhan nut de test gimbal khac
        if(JIG_TEST_button_state_back_to_scan_new_barCode() == true)
        {
            memset(&gimbal_FSTD_global, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_global_t));
            memset(&gimbal_FSTD_private, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_private_t));
            
            /// setting lai stiffness profile ship
            JIG_TEST_gimbal_FSTD_v2_setting_stiffness(STIFFNESS_TYPE_PROFILE_SHIP);
            
            /// write to console
            JIG_TEST_console_write(back_to_scan_new_barCode);
            JIG_TEST_console_write(" TEST DONE \n");
        }
        else
        {
            if(gimbal_FSTD_global.state_test < JIG_TEST_GIMBAL_V2_STATE_DONE)
            {
                    //// trong qua trinh test Pi send reset
                memset(&gimbal_FSTD_global, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_global_t));
                memset(&gimbal_FSTD_private, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_private_t));
                
                /// setting lai stiffness profile ship
                JIG_TEST_gimbal_FSTD_v2_setting_stiffness(STIFFNESS_TYPE_PROFILE_SHIP);
                
                /// write to console
                JIG_TEST_console_write(back_to_scan_new_barCode);
                JIG_TEST_console_write(" TEST ERROR \n");
            }
        }
    }
}






/** @brief gimbal_FSTD_v2_storage_state_to_backup_resister
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_gimbal_FSTD_v2_test_state_t state)
{
    if(gimbal_FSTD_private.fstorage_state == false)
    {
        /// set flag storage_state
        gimbal_FSTD_private.fstorage_state = true;
        
        /// get state present
        gimbal_FSTD_private.state_storage = state;
        
        /// storage backup resister
        JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_private.state_storage, LL_RTC_BKP_DR13);
    }
}

/** @brief gimbal_FSTD_v2_reset_flag_storage_state
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state(void)
{
    /// reset flag storage_state
    gimbal_FSTD_private.fstorage_state = false;
}

/** @brief gimbal_FSTD_v2_timeOut_state
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_timeOut_state(void)
{
    char *state_timeOut = "\nSTATE_TIMEOUT --->";
    static uint8_t count_debug = 0;
    static uint8_t pre_state = 0;
    static bool get_state = false;
    
    /// get state first
    if(get_state == false)
    {
        get_state = true;
        
        /// reset count state
        count_debug = 0;
        
        pre_state = (uint8_t)gimbal_FSTD_global.state_test;
    }
    
    /// tinh thoi gian 1 state
    if(get_timeOut(1000, JIG_TEST_DEBUG_STATE_AND_MODE))
    {
        uint8_t time = 0;
        /// chon timeOut cho state test
        if(gimbal_FSTD_global.state_test == JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST)
        {
            time = 115;
        }
        else
        {
            time = 20;
        }
        
        if(++count_debug > time)
        {
            /// kiem tra persent
            uint8_t state_ = (uint8_t)gimbal_FSTD_global.state_test;
            uint8_t compare_state = state_ - pre_state;
            if(compare_state == 0)
            {
                count_debug = 21;
                
                char buff[200];
                
                if(gimbal_FSTD_global.state_test == JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE)
                {
                    /// timeOut state ---> chay lai tu dau
                    NVIC_SystemReset();
                }
                else
                {
                    /// timeOut state ---> in ra Console
                    JIG_TEST_console_write(state_timeOut);
                    sprintf(buff, "state : %s | mode : %s"
                    , debug_str_state[gimbal_FSTD_global.state_test]
                    , debug_str_mode_test[gimbal_FSTD_global.mode_test]);
                    JIG_TEST_console_write(buff);
                    HAL_Delay(10);
                }
            }
        }
        else
        {
            
            /// kiem tra persent
            uint8_t state_ = (uint8_t)gimbal_FSTD_global.state_test;
            uint8_t compare_state = state_ - pre_state;
            
            if(compare_state != 0)
            {
                /// reset flag get state ---> get state new
                get_state = false;
            }
        }
    }
}

#if (JIG_TEST_ID == 0x10)

/** @brief gimbal_FSTD_v2_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_control_process(void)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    static uint32_t count_console_case = 0;
    
    switch((uint8_t)gimbal_FSTD_global.state_test)
    {
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_IDLE:
        {
            /// next state
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
        {
            /// waitting start from raspberry
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2;
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
        {
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            /// check com 2
            if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
            {
                char buff[100];
                
                JIG_TEST_console_write(control_process);
                sprintf(buff, "gimbal ID : %3d", mavlink_gimbal_COM2.vehicle_system_id);
                JIG_TEST_console_write(buff);
                
                /// reset flag storage state
                JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
        {
            static uint32_t count_console = 0;
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor() == true)
                {
                    /// reset flag storage state
                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                    
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
        {
            static uint32_t count_console = 0;
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu() == true)
                {
                    char buff[100];
                    /// setting param imu rate
                    gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                    JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    
                    sprintf(buff, "setting param imu rate : value <-> %f | id : %s\n"
                    , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    JIG_TEST_console_write(buff);
                    
                    /// reset flag storage state
                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                    
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
        {
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            /// kiem tra loai imu dang dung
            gimbal_FSTD_private.imu = JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM2);
            if(gimbal_FSTD_private.imu != GREMSY_SENSOR_ERROR)
            {
                JIG_TEST_console_write(control_process);
                
                if(gimbal_FSTD_private.imu == GREMSY_SENSOR_BMI160)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_BMI160\n");
                }
                else if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_ICM42688\n");
                }
                
                /// reset flag storage state
                JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("GREMSY_SENSOR_ERROR\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
        {
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            /// setting param gimbal
            if(JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2) == true)
            {
                
                /// reset flag storage state
                JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
        {
            
            /// get state present
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST);
            
            /// timeOut state
            JIG_TEST_gimbal_FSTD_v2_timeOut_state();
            
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST))
            {
                gimbal_FSTD_global.time_test_total ++;
                
                if(gimbal_FSTD_private.is_test_running == true)
                {
                    gimbal_FSTD_private.time_run_test ++;
                    
                    /// toggle led blue
                    if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_FOLLOW_MODE)
                    {
                        HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                    }
                    else if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
                    {
                        HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                    }
                }
            }
            
            /// mode test process
            switch((uint8_t)gimbal_FSTD_global.mode_test)
            {
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_IDLE:
                {
                    // next state
                    gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_SBUS;
                    
                    /// write console
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_SBUS\n");
                    
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_SBUS:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PPM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PPM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_CAN;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_CAN\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_CAN:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_CAN, JIG_TEST_GIMBAL_MODE_READ_CAN, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_CAN] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM4;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM4\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM4:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM4, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_AUX;
                        
                        /// reset command ack
                        mavlink_gimbal_COM2.ack.command = 0;

                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_AUX\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_AUX, JIG_TEST_GIMBAL_MODE_READ_COM, 4))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] = false;
                        
                        /// on motor gimbal next state test vibrate
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                        
                        /// reset flag storage state
                        JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_VIBRATE;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_VIBRATE\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                {
                    uint8_t time = 0;
                    
                  #if (S1v3_VIRATE_TEST == 1)
                    if(mavlink_gimbal_COM2.vehicle_system_id == 0x22)
                  #else
                    if(false)
                  #endif
                    {
                        time = 100;
                    }
                    else
                    {
                        time = 12;
                    }
                    
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                    , JIG_TEST_GIMBAL_V2_MODE_VIBRATE, JIG_TEST_GIMBAL_MODE_READ_COM, time))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_DONE;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_DONE\n");
                    }
                    else
                    {
                        if(++ count_console_case > 200000)
                        {
                            count_console_case = 0;
                            
                            /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                            if(mavlink_gimbal_COM2.status.mode == 0)
                            {
                                /// on motor gimbal in mode test vibration
                                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                            }
                        }
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_DONE:
                {
                    
                    static bool stateDone = false;
                    
                    if(stateDone == false)
                    {
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_SBUS"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_SBUS]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PPM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PPM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_CAN"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_CAN]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM4"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM4]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_AUX"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_VIBRATE"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD]);
                        
                        /// turn off motor gimbal
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        
                        /// reset flag storage state
                        JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        
                        stateDone = true;
                        
                    }
                    else
                    {
                        static bool setImuStartupCalib = false;
        
                        if(setImuStartupCalib == false)
                        {
                            setImuStartupCalib = JIG_TEST_gimbal_FSTD_v2_set_calib_startup_imu_status(false);
                        }
                        else
                        {
                            setImuStartupCalib = false;
                            stateDone = false;
                            
                        
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                            HAL_Delay(10);
                            
                            /// next state
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                        }
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_LOOP:
                {
                    static uint8_t count_wait_loop = 0;
                    if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP))
                    {
                        if(++count_wait_loop > 5)
                        {
                            gimbal_FSTD_global.param_value[1] = -23;
                            gimbal_FSTD_global.param_value[2] = -24;
                            gimbal_FSTD_global.param_value[3] = -25;
                            gimbal_FSTD_global.param_value[4] = 100;
                            gimbal_FSTD_global.param_value[5] = 102;
                            gimbal_FSTD_global.param_value[6] = 101;
                            
                            gimbal_FSTD_global.read_param_done = true;
                            
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                            gimbal_FSTD_global.mode_test  = JIG_TEST_GIMBAL_V2_MODE_DONE;
                        }
                    }
                }break;
            }
            
            /// run all mode test
            JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD_private);
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_DONE:
        {
            /// get state idle cho lan sau sau
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_IDLE);
            
                /// button prcess
            JIG_TEST_button_process();
            
            /// xoay gimbal kiem tra feedback imu
            JIG_TEST_gimbal_FSTD_run_feed_back_imu();
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_ERROR:
        {
            /// get state idle cho lan sau sau
            JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_IDLE);
            
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR))
            {
                if(gimbal_FSTD_private.error_heartbeat_com2 == true)
                {
                    /// toggle led red 
                    HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("COM2 Heartbeat error !!!\n");
                }
            }
        }break;
    }
}

#endif

#if (JIG_TEST_ID == 0x11)

/** @brief gimbal_FSTD_v2_check_result
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_check_result(void)
{
    bool ret = false;
    static uint8_t count = 0;
    
    for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD; i++)
    {
        if(gimbal_FSTD_global.result_mode_test[i] == true)
        {
            count++;
        }
    }
    
    if(count == 7)
    {
        ret = true;
    }
    
    return ret;
}

/** @brief gimbal_FSTD_v2_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_control_process(void)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    static uint32_t count_console_case = 0;
    
    switch((uint8_t)gimbal_FSTD_global.state_test)
    {
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_IDLE:
        {
            /// get heartbeat from raspberry
            if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_heartbeatReady() == true)
            {
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN;
                
                JIG_TEST_console_write(control_process);
                JIG_TEST_console_write("cloudData_heartbeat Ready\n");
                
                /// reset count console case
                count_console_case = 0;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("cloudData_heartbeat not Ready\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN:
        {
            /// get login from raspberry
            if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_login() == true)
            {
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_scan_barCode() == true)
                {
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE;
                }
                
                /// kiem tra jig test co reset hay khong
                if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == true)
                {
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                }
                
                /// kiem tra Pi send start sau khi login
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                {
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                }
                
                /// reset count console case
                count_console_case = 0;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE:
        {
            /// get barCode scan from raspberry
            if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
            {
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                
                /// reset count console case
                count_console_case = 0;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    ///wait scan barcode is turn off gimbal
                    JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                    
                    /// write to console
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM:
        {
            //// kiem tra flag first run get from backup register
            gimbal_FSTD_private.fisrt_run = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR12);
            
            if(gimbal_FSTD_private.fisrt_run == false)
            {
                gimbal_FSTD_private.fisrt_run = true;
                JIG_TEST_rtc_storage_register_value(gimbal_FSTD_private.fisrt_run, LL_RTC_BKP_DR12);
                
                JIG_TEST_console_write(control_process);
                JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                
                /// set flag reset system
                gimbal_FSTD_global.get_reset_system = false;
                JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
            }
            else
            {
                //// kiem tra flag reset system get from backup register
                gimbal_FSTD_global.get_reset_system = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR11);
                
                /// get reset system
                if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == false)
                {
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM --- RESTART\n");
                    HAL_Delay(100);
                    /// set flag reset system
                    gimbal_FSTD_global.get_reset_system = true;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                    
                    /// apply reset system
                    NVIC_SystemReset();
                }
                else
                {
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                    
                    /// set flag reset system
                    gimbal_FSTD_global.get_reset_system = false;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                    
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
        {
            /// waitting start from raspberry
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2;
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
        {
            /// check com 2
            if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
            {
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
        {
            static uint32_t count_console = 0;
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor() == true)
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
        {
            static uint32_t count_console = 0;
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu() == true)
                {
                    char buff[100];
                    /// setting param imu rate
                    gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                    JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    
                    sprintf(buff, "setting param imu rate : value <-> %f | id : %s\n"
                    , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    JIG_TEST_console_write(buff);
                    
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
        {
            /// kiem tra loai imu dang dung
            gimbal_FSTD_private.imu = JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM2);
            if(gimbal_FSTD_private.imu != GREMSY_SENSOR_ERROR)
            {
                JIG_TEST_console_write(control_process);
                
                if(gimbal_FSTD_private.imu == GREMSY_SENSOR_BMI160)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_BMI160\n");
                }
                else if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_ICM42688\n");
                }
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("GREMSY_SENSOR_ERROR\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
        {
            /// setting param gimbal
            if(JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2) == true)
            {
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
        {
            
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST))
            {
                gimbal_FSTD_global.time_test_total ++;
                
                if(gimbal_FSTD_private.is_test_running == true)
                {
                    gimbal_FSTD_private.time_run_test ++;
                    
                    /// toggle led blue
                    if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_FOLLOW_MODE)
                    {
                        HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                    }
                    else if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
                    {
                        HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                    }
                }
            }
            
            /// mode test process
            switch((uint8_t)gimbal_FSTD_global.mode_test)
            {
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_IDLE:
                {
                    // next state
                    gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_SBUS;
                    
                    /// write console
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_SBUS\n");
                    
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_SBUS:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PPM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PPM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_CAN;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_CAN\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_CAN:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_CAN, JIG_TEST_GIMBAL_MODE_READ_CAN, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_CAN] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM4;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM4\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM4:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM4, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_AUX;

                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_AUX\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_AUX, JIG_TEST_GIMBAL_MODE_READ_COM, 4))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] = false;
                        
                        /// on motor gimbal next state test vibrate
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_VIBRATE;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_VIBRATE\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_VIBRATE, JIG_TEST_GIMBAL_MODE_READ_COM, 12))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD\n");
                    }
                    else
                    {
                        if(++ count_console_case > 200000)
                        {
                            count_console_case = 0;
                            
                            /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                            if(mavlink_gimbal_COM2.status.mode == 0)
                            {
                                /// on motor gimbal in mode test vibration
                                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                            }
                        }
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD: /// setting profile ship & read param imu send to cloud data
                {
                    char buff[200];
                    
                    if(JIG_TEST_gimbal_FSTD_v2_request_param_profile_ship(&mavlink_gimbal_COM2) == true)
                    {
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("write param profile ship DONE\n");
                        
                        gimbal_FSTD_global.param_value[1] = gimbal_user_profile_ship[37].value_param_get;
                        gimbal_FSTD_global.param_value[2] = gimbal_user_profile_ship[38].value_param_get;
                        gimbal_FSTD_global.param_value[3] = gimbal_user_profile_ship[39].value_param_get;
                        gimbal_FSTD_global.param_value[4] = gimbal_user_profile_ship[40].value_param_get;
                        gimbal_FSTD_global.param_value[5] = gimbal_user_profile_ship[41].value_param_get;
                        gimbal_FSTD_global.param_value[6] = gimbal_user_profile_ship[42].value_param_get;
                        
                        /// set flag read param done
                        gimbal_FSTD_global.read_param_done = true;
                        
                        sprintf(buff, "\nGYROX_OFFSET : %.f | GYROY_OFFSET : %.f| GYROZ_OFFSET : %.f\nACCX_OFFSET : %.f | ACCY_OFFSET : %.f | ACCZ_OFFSET : %.f\n"
                        , gimbal_FSTD_global.param_value[1]
                        , gimbal_FSTD_global.param_value[2]
                        , gimbal_FSTD_global.param_value[3]
                        , gimbal_FSTD_global.param_value[4]
                        , gimbal_FSTD_global.param_value[5]
                        , gimbal_FSTD_global.param_value[6]
                        );
                        JIG_TEST_console_write(buff);
                        HAL_Delay(10);
                        
                        /// kiem tra test result ok or fail
                        if(JIG_TEST_gimbal_FSTD_v2_check_result() == true)
                        {
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_DONE;
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_DONE\n");
                            HAL_Delay(10);
                        }
                        else
                        {
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_ERROR;
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_ERROR\n");
                            HAL_Delay(10);
                        }
                        
                        /// turn off motor
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        

                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_DONE:
                {
                    /// cho Pi gui ket qua
                    if(gimbal_FSTD_global.cloudData_command.result_pushData != (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_NONE)
                    {
                        /// lay ket qua push data to cloud
                        gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = gimbal_FSTD_global.cloudData_command.result_pushData;
                        
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                        HAL_Delay(10);
                        
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_ERROR:
                {
                    /// cho Pi gui ket qua
                    if(gimbal_FSTD_global.cloudData_command.result_pushData != (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_NONE)
                    {
                        /// lay ket qua push data to cloud
                        gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = gimbal_FSTD_global.cloudData_command.result_pushData;
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_SBUS"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_SBUS]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PPM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PPM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_CAN"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_CAN]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM4"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM4]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_AUX"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_VIBRATE"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD]);
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                        HAL_Delay(10);
                        
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_LOOP:
                {
                    static uint8_t count_wait_loop = 0;
                    if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP))
                    {
                        if(++count_wait_loop > 5)
                        {
                            gimbal_FSTD_global.param_value[1] = -23;
                            gimbal_FSTD_global.param_value[2] = -24;
                            gimbal_FSTD_global.param_value[3] = -25;
                            gimbal_FSTD_global.param_value[4] = 100;
                            gimbal_FSTD_global.param_value[5] = 102;
                            gimbal_FSTD_global.param_value[6] = 101;
                            
                            gimbal_FSTD_global.read_param_done = true;
                            
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                            gimbal_FSTD_global.mode_test  = JIG_TEST_GIMBAL_V2_MODE_DONE;
                        }
                    }
                }break;
            }
            
            /// run all mode test
            JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD_private);
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_DONE:
        {
                /// button prcess
            JIG_TEST_button_process();
            
            /// xoay gimbal kiem tra feedback imu
            JIG_TEST_gimbal_FSTD_run_feed_back_imu();
            
            /// apply cmd reset from Pi with button back to scan new barCode
            JIG_TEST_gimbal_FSTD_back_to_scan_new_barCode();
            
            /// apply cmd reset from Pi auto back to scan new barCode
            if(gimbal_FSTD_global.cloudData_command.auto_back == true)
            {
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                {
                        //// trong qua trinh test raspberry send reset
                    memset(&gimbal_FSTD_global, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_global_t));
                    memset(&gimbal_FSTD_private, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_private_t));
                    
                    /// setting lai stiffness profile ship
                    JIG_TEST_gimbal_FSTD_v2_setting_stiffness(STIFFNESS_TYPE_PROFILE_SHIP);
                    
                    /// write to console
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write(" AUTO BACK SCAN NEW BARCODE \n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_ERROR:
        {
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR))
            {
                if(gimbal_FSTD_private.error_heartbeat_com2 == true)
                {
                    /// toggle led red 
                    HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("COM2 Heartbeat error !!!\n");
                }
            }
        }break;
    }
}

#endif




#if (JIG_TEST_ID == 0x20)

/** @brief gimbal_FSTD_v2_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_control_process(void)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    static uint32_t count_console_case = 0;
    
    /// get heartbeat from raspberry
    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_heartbeatReady() == true)
    {
        switch((uint8_t)gimbal_FSTD_global.state_test)
        {
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_IDLE:
            {
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN:
            {
                /// get login from raspberry
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_login() == true)
                {
                    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_scan_barCode() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE;
                    }
                    
                    /// kiem tra jig test co reset hay khong
                    if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    }
                    
                    /// kiem tra Pi send start sau khi login
                    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    }
                    
                    /// reset count console case
                    count_console_case = 0;
                }
                else
                {
                    if(++count_console_case > 200000)
                    {
                        count_console_case = 0;
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN\n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE:
            {
                /// get barCode scan from raspberry
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                {
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    
                    /// reset count console case
                    count_console_case = 0;
                }
                else
                {
                    if(++count_console_case > 200000)
                    {
                        count_console_case = 0;
                        
                        ///wait scan barcode is turn off gimbal
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        
                        /// write to console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE\n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM:
            {
                //// kiem tra flag first run get from backup register
                gimbal_FSTD_private.fisrt_run = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR12);
                
                if(gimbal_FSTD_private.fisrt_run == false)
                {
                    gimbal_FSTD_private.fisrt_run = true;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_private.fisrt_run, LL_RTC_BKP_DR12);
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                    
                    /// set flag reset system
                    gimbal_FSTD_global.get_reset_system = false;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                    
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
                }
                else
                {
                    //// kiem tra flag reset system get from backup register
                    gimbal_FSTD_global.get_reset_system = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR11);
                    
                    /// get reset system
                    if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == false)
                    {
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM --- RESTART\n");
                        HAL_Delay(100);
                        /// set flag reset system
                        gimbal_FSTD_global.get_reset_system = true;
                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                        
                        /// apply reset system
                        NVIC_SystemReset();
                    }
                    else
                    {
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                        
                        /// set flag reset system
                        gimbal_FSTD_global.get_reset_system = false;
                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                        
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
            {
                gimbal_FSTD_global.usb_speed.enable_test = true;
                
                /// waitting start from raspberry
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2;
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
            {
                /// check com 2
                if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR;
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
            {
                static uint32_t count_console = 0;
                
                if(++count_console > 150000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(control_process);
                    
                    /// waitting gimbal startup calib
                    if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor() == true)
                    {
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU;
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
            {
                static uint32_t count_console = 0;
                
                if(++count_console > 150000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(control_process);
                    
                    /// waitting gimbal startup calib
                    if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu() == true)
                    {
                        char buff[100];
                        /// setting param imu rate
                        gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                        JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                        
                        sprintf(buff, "setting param imu rate : value <-> %f | id : %s\n"
                        , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                        JIG_TEST_console_write(buff);
                        
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE;
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
            {
                /// kiem tra loai imu dang dung
                gimbal_FSTD_private.imu = JIG_TEST_mavlink_gimbal_get_sensor_name(mavlink_gimbal_COM2);
                if(gimbal_FSTD_private.imu != GREMSY_SENSOR_ERROR)
                {
                    JIG_TEST_console_write(control_process);
                    
                    if(gimbal_FSTD_private.imu == GREMSY_SENSOR_BMI160)
                    {
                        JIG_TEST_console_write("GREMSY_SENSOR_BMI160\n");
                    }
                    else if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
                    {
                        JIG_TEST_console_write("GREMSY_SENSOR_ICM42688\n");
                    }
                    
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM;
                }
                else
                {
                    if(++count_console_case > 200000)
                    {
                        count_console_case = 0;
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("GREMSY_SENSOR_ERROR\n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
            {
                /// setting param gimbal
                if(JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2) == true)
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST;
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
            {
                
                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST))
                {
                    gimbal_FSTD_global.time_test_total ++;
                    
                    if(gimbal_FSTD_private.is_test_running == true)
                    {
                        gimbal_FSTD_private.time_run_test ++;
                        
                        /// toggle led blue
                        if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_FOLLOW_MODE)
                        {
                            HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                        }
                        else if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
                        {
                            HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                        }
                    }
                }
                
                /// mode test process
                switch((uint8_t)gimbal_FSTD_global.mode_test)
                {
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_IDLE:
                    {
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_SBUS;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_SBUS\n");
                        
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_SBUS:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PPM;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PPM\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_COM, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM4;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM4\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM4:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_COM4, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_AUX;

                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_AUX\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_AUX, JIG_TEST_GIMBAL_MODE_READ_COM, 4))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] = false;
                            
                            /// on motor gimbal next state test vibrate
                            JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_VIBRATE;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_VIBRATE\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_VIBRATE, JIG_TEST_GIMBAL_MODE_READ_COM, 12))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_DONE;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_DONE\n");
                        }
                        else
                        {
                            if(++ count_console_case > 200000)
                            {
                                count_console_case = 0;
                                
                                /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                                if(mavlink_gimbal_COM2.status.mode == 0)
                                {
                                    /// on motor gimbal in mode test vibration
                                    JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                                }
                            }
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_DONE:
                    {
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_SBUS"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_SBUS]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PPM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PPM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_CAN"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_CAN]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM4"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM4]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_AUX"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_VIBRATE"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD]);
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                        HAL_Delay(10);
                        
                        /// turn off motor
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_LOOP:
                    {
                        static uint8_t count_wait_loop = 0;
                        if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP))
                        {
                            if(++count_wait_loop > 5)
                            {
                                gimbal_FSTD_global.param_value[1] = -23;
                                gimbal_FSTD_global.param_value[2] = -24;
                                gimbal_FSTD_global.param_value[3] = -25;
                                gimbal_FSTD_global.param_value[4] = 100;
                                gimbal_FSTD_global.param_value[5] = 102;
                                gimbal_FSTD_global.param_value[6] = 101;
                                
                                gimbal_FSTD_global.read_param_done = true;
                                
                                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                                gimbal_FSTD_global.mode_test  = JIG_TEST_GIMBAL_V2_MODE_DONE;
                            }
                        }
                    }break;
                }
                
                /// run all mode test
                JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD_private);
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_DONE:
            {
                    /// button prcess
                JIG_TEST_button_process();
                
                /// xoay gimbal kiem tra feedback imu
                JIG_TEST_gimbal_FSTD_run_feed_back_imu();

            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_ERROR:
            {
                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR))
                {
                    if(gimbal_FSTD_private.error_heartbeat_com2 == true)
                    {
                        /// toggle led red 
                        HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("COM2 Heartbeat error !!!\n");
                    }
                }
            }break;
        }
        
        /// reset count console case
        count_console_case = 0;
    }
    else
    {
        if(++count_console_case > 200000)
        {
            count_console_case = 0;
            
            JIG_TEST_console_write(control_process);
            JIG_TEST_console_write("cloudData_heartbeat not Ready\n");
        }
    }
    

}

#endif
#if (JIG_TEST_ID == 0x21)

/** @brief gimbal_FSTD_v2_check_result
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_v2_check_result(void)
{
    bool ret = false;
    static uint8_t count = 0;
    
    for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD; i++)
    {
        if(gimbal_FSTD_global.result_mode_test[i] == true)
        {
            count++;
        }
    }
    
    if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_PASS)
    {
        count++;
    }
    
    if(count == 7)
    {
        ret = true;
    }
    
    return ret;
}




/** @brief gimbal_FSTD_v2_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_control_process(void)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    static uint32_t count_console_case = 0;
    
    /// get heartbeat from raspberry
    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_heartbeatReady() == true)
    {
        switch((uint8_t)gimbal_FSTD_global.state_test)
        {
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_IDLE:
            {
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_IDLE);
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN;
                
                /// reset flag storage state
                JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN:
            {
                
                static uint32_t count_console_wait_login = 0;
                
                /// get login from raspberry
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_login() == true)
                {
                    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_scan_barCode() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE;
                    }
                    
                    /// kiem tra jig test co reset hay khong
                    if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    }
                    
                    /// kiem tra Pi send start sau khi login
                    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                    {
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    }
                    
                    /// reset count console case
                    count_console_wait_login = 0;
                }
                else
                {
                    if(++count_console_wait_login > 200000)
                    {
                        count_console_wait_login = 0;
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN\n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE:
            {
                
                static uint32_t count_console_wait_scan_barCode = 0;

                /// get barCode scan from raspberry
                if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM;
                    
                    /// reset count_console_wait_scan_barCode
                    count_console_wait_scan_barCode = 0;
                }
                else
                {
                    if(++count_console_wait_scan_barCode > 150000)
                    {
                        count_console_wait_scan_barCode = 0;
                        
                        /// write to console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE\n");
                    }
                }
                
                /// waitting scan barCode ---> Off motor gimbal
                if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_ON)
                {
                    JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM:
            {

                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                //// kiem tra flag first run get from backup register
                gimbal_FSTD_private.fisrt_run = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR12);
                
                if(gimbal_FSTD_private.fisrt_run == false)
                {
                    gimbal_FSTD_private.fisrt_run = true;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_private.fisrt_run, LL_RTC_BKP_DR12);
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                    
                    /// set flag reset system
                    gimbal_FSTD_global.get_reset_system = false;
                    JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                    
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
                }
                else
                {
                    //// kiem tra flag reset system get from backup register
                    gimbal_FSTD_global.get_reset_system = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR11);
                    
                    /// get reset system
                    if(JIG_TEST_gimbal_FSTD_v2_get_reset_system() == false)
                    {
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM --- RESTART\n");
                        HAL_Delay(100);
                        /// set flag reset system
                        gimbal_FSTD_global.get_reset_system = true;
                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                        
                        /// apply reset system
                        NVIC_SystemReset();
                    }
                    else
                    {
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM -- SCAN_BARCODE_DONE\n");
                        
                        /// set flag reset system
                        gimbal_FSTD_global.get_reset_system = false;
                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_global.get_reset_system, LL_RTC_BKP_DR11);
                        
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
            {
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_WAIT_START);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                /// ktra co enable truoc do hay chua
                if(gimbal_FSTD_global.usb_speed.enable_test == false)
                {
                    gimbal_FSTD_global.usb_speed.enable_test = true;
                }
                
                /// waitting start from raspberry
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2;
                
                /// reset flag storage state
                JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();

                JIG_TEST_console_write(control_process);
                JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_START\n");
                
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
            {
                
                static uint32_t count_console_check_COM2 = 0;
                char buff[100];
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                /// check com 2
                if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR;
                    
                    JIG_TEST_console_write(control_process);
                    
                    /// setting param imu rate
                    gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                    JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    
                    sprintf(buff, "setting param imu rate : value <-> %f | id : %s\n"
                    , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    JIG_TEST_console_write(buff);

                    /// reset flag storage state
                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                    
                    count_console_check_COM2 = 0;
                }
                else
                {
                    if(++count_console_check_COM2 > 150000)
                    {
                        count_console_check_COM2 = 0;
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2\n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
            {
                static uint32_t count_console = 0;
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                    
                /// trong qua trinh calib motor ktra COM2 connection
                if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
                {
                    /// waitting gimbal startup calib
                    if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor() == true)
                    {
                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU;
                        
                        /// reset flag storage state
                        JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        
                        /// reset param index
                        mavlink_gimbal_COM2.param_value.param_index = 0;
                    
                        count_console = 0;
                    }
                    else
                    {
                        if(++count_console > 150000)
                        {
                            count_console = 0;
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR\n");
                        }
                    }
                }
                else
                {
                     if(++count_console > 150000)
                    {
                        count_console = 0;
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("MOTOR CALIB -- HEARTBEAT timeOut\n");
                    }
                }
                
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
            {
                static uint32_t count_console = 0;
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                /// check com 2
                if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
                {
                    /// waitting gimbal startup calib
                    if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu() == true)
                    {

                        /// next state
                        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE;
                        
                        /// reset flag storage state
                        JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        
                        ///
                        count_console = 0;
                    }
                    else
                    {
                        
                        if(++count_console > 150000)
                        {
                            count_console = 0;
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU\n");
                        }
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
            {
                
                static uint32_t count_console_check_imu_type = 0;
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                /// kiem tra loai imu dang dung
                gimbal_FSTD_private.imu = JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM2);
                if(gimbal_FSTD_private.imu != GREMSY_SENSOR_ERROR)
                {
                    JIG_TEST_console_write(control_process);
                    
                    if(gimbal_FSTD_private.imu == GREMSY_SENSOR_BMI160)
                    {
                        JIG_TEST_console_write("GREMSY_SENSOR_BMI160\n");
                    }
                    else if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
                    {
                        JIG_TEST_console_write("GREMSY_SENSOR_ICM42688\n");
                    }

                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM;
                    
                    /// reset flag storage state
                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                    
                    count_console_check_imu_type = 0;
                }
                else
                {
                    char buff[100];
                    
                    if(++count_console_check_imu_type > 200000)
                    {
                        count_console_check_imu_type = 0;

                        JIG_TEST_console_write(control_process);
                        
                        /// setting param imu rate
                        gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                        JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                        
                        sprintf(buff, "setting param imu rate Try again: value <-> %f | id : %s\n"
                        , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                        JIG_TEST_console_write(buff);
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
            {
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                /// setting param gimbal
                if(JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2) == true)
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST;
                    
                    /// reset flag storage state
                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM - Done\n");
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
            {
                
                /// get state present
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST);
                
                /// timeOut state
                JIG_TEST_gimbal_FSTD_v2_timeOut_state();
                
                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST))
                {
                    gimbal_FSTD_global.time_test_total ++;
                    
                    if(gimbal_FSTD_private.is_test_running == true)
                    {
                        gimbal_FSTD_private.time_run_test ++;
                        
                        /// toggle led blue
                        if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_FOLLOW_MODE)
                        {
                            HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                        }
                        else if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
                        {
                            HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                        }
                    }
                }
                
                /// mode test process
                switch((uint8_t)gimbal_FSTD_global.mode_test)
                {
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_IDLE:
                    {
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_SBUS;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_SBUS\n");
                        
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_SBUS:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PPM;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PPM\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_COM, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM4;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM4\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM4:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_COM4, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_AUX;

                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_AUX\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_AUX, JIG_TEST_GIMBAL_MODE_READ_COM, 4))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] = false;
                            
                            /// on motor gimbal next state test vibrate
                            JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_VIBRATE;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_VIBRATE\n");
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                    {
                        if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                            , JIG_TEST_GIMBAL_V2_MODE_VIBRATE, JIG_TEST_GIMBAL_MODE_READ_COM, 12))
                        {
                            /// reset many available for after test
                            gimbal_FSTD_private.is_test_running = false;
                            gimbal_FSTD_private.time_run_test = 0;
                            gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = false;
                            
                            // next state
                            gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD;
                            
                            /// write console
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD\n");
                        }
                        else
                        {
                            if(++ count_console_case > 200000)
                            {
                                count_console_case = 0;
                                
                                /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                                if(mavlink_gimbal_COM2.status.mode == 0)
                                {
                                    /// on motor gimbal in mode test vibration
                                    JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                                }
                            }
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD: /// setting profile ship & read param imu send to cloud data
                    {
                        char buff[200];
                        
                        /// ktra COM2 connection
                        if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
                        {
                            if(JIG_TEST_gimbal_FSTD_v2_request_param_profile_ship(&mavlink_gimbal_COM2) == true)
                            {
                                
                                JIG_TEST_console_write(control_process);
                                JIG_TEST_console_write("write param profile ship DONE\n");
                                
                                gimbal_FSTD_global.param_value[1] = gimbal_user_profile_ship_ac30000[49].value_param_get;
                                gimbal_FSTD_global.param_value[2] = gimbal_user_profile_ship_ac30000[50].value_param_get;
                                gimbal_FSTD_global.param_value[3] = gimbal_user_profile_ship_ac30000[51].value_param_get;
                                gimbal_FSTD_global.param_value[4] = gimbal_user_profile_ship_ac30000[52].value_param_get;
                                gimbal_FSTD_global.param_value[5] = gimbal_user_profile_ship_ac30000[53].value_param_get;
                                gimbal_FSTD_global.param_value[6] = gimbal_user_profile_ship_ac30000[54].value_param_get;
                                
                                /// set flag read param done
                                gimbal_FSTD_global.read_param_done = true;
                                
                                sprintf(buff, "\nGYROX_OFFSET : %.f | GYROY_OFFSET : %.f| GYROZ_OFFSET : %.f\nACCX_OFFSET : %.f | ACCY_OFFSET : %.f | ACCZ_OFFSET : %.f\n"
                                , gimbal_FSTD_global.param_value[1]
                                , gimbal_FSTD_global.param_value[2]
                                , gimbal_FSTD_global.param_value[3]
                                , gimbal_FSTD_global.param_value[4]
                                , gimbal_FSTD_global.param_value[5]
                                , gimbal_FSTD_global.param_value[6]
                                );
                                JIG_TEST_console_write(buff);
                                HAL_Delay(10);
                                
                                /// turn off motor
                                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                                
                                // next state
                                gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_USB_SPEED;
                                
                            }
                        }
                    }break;
                    case JIG_TEST_GIMBAL_V2_MODE_USB_SPEED:
                    {
                        static uint32_t usb_speed_console = 0;
                        
                        /// kiem tra test usb
                        if(gimbal_FSTD_global.usb_speed.send_end_done == true)
                        {
                            /// kiem tra test result ok or fail
                            if(JIG_TEST_gimbal_FSTD_v2_check_result() == true)
                            {
                                // next state
                                gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_DONE;
                                
                                JIG_TEST_console_write(control_process);
                                JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_DONE\n");
                                HAL_Delay(10);
                            }
                            else
                            {
                                // next state
                                gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_ERROR;
                                
                                JIG_TEST_console_write(control_process);
                                JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_ERROR\n");
                                HAL_Delay(10);
                            }
                            
                            usb_speed_console = 0;
                        }
                        else
                        {
                            if(HAL_GetTick() - usb_speed_console > 1000 || usb_speed_console == 0)
                            {
                                char buff[200];
                                
                                usb_speed_console = HAL_GetTick();
                                
                                JIG_TEST_console_write(control_process);
                                sprintf(buff, "JIG_TEST_GIMBAL_V2_MODE_USB_SPEED| enable_test : %d | send_end_done : %d | send_start : %d | test_done : %d | status : %d\n"
                                , gimbal_FSTD_global.usb_speed.enable_test
                                , gimbal_FSTD_global.usb_speed.send_end_done
                                , gimbal_FSTD_global.usb_speed.send_start
                                , gimbal_FSTD_global.usb_speed.test_done
                                ,gimbal_FSTD_global.usb_speed.status
                                );
                                JIG_TEST_console_write(buff);
                            }
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_DONE:
                    {
                        static uint8_t count_timeOut = 0;
                        
                        /// cho Pi gui ket qua 
                        if(gimbal_FSTD_global.cloudData_command.result_pushData == (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK)
                        {
                            /// lay ket qua push data to cloud
                            gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = true;
                            
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                            HAL_Delay(10);
                            
                            /// next state
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                            
                            /// reset flag storage state
                            JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                            
                            /// reset flag storage state
                            JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        }
                        else if(gimbal_FSTD_global.cloudData_command.result_pushData == (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL)
                        {
                            /// lay ket qua push data to cloud
                            gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = false;
                            
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_ERROR\n");
                            HAL_Delay(10);
                            
                            /// next state
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
                            
                            /// reset flag storage state
                            JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        }
                        else
                        {
                            /// khong nhan duoc ket qua timeOut ????
                            if(get_timeOut(1000, JIG_TEST_TIMEOUT_MODE_DONE))
                            {
                                if(++count_timeOut > 30)
                                {
                                    count_timeOut = 0;
                                    
                                    JIG_TEST_console_write(control_process);
                                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_ERROR -- Timeout push Data to Cloud\n");
                                    HAL_Delay(10);
                                    
                                    /// next state
                                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
                                    
                                    /// reset flag storage state
                                    JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                                    
                                    /// lay ket qua push data to cloud
                                    gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = false;
                                }
                            }
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_ERROR:
                    {
                        /// cho Pi gui ket qua
                        if(gimbal_FSTD_global.cloudData_command.result_pushData != (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_NONE)
                        {
                            /// lay ket qua push data to cloud
                            gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] = gimbal_FSTD_global.cloudData_command.result_pushData;
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_SBUS"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_SBUS]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PPM"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PPM]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_CAN"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_CAN]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM4"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM4]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_AUX"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_VIBRATE"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]);
                            
                            JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD"
                            , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD]);
                            
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                            HAL_Delay(10);
                            
                            /// next state
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                            
                            /// reset flag storage state
                            JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        }
                    }break;
                    case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_LOOP:
                    {
                        static uint8_t count_wait_loop = 0;
                        if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP))
                        {
                            if(++count_wait_loop > 5)
                            {
                                gimbal_FSTD_global.param_value[1] = -23;
                                gimbal_FSTD_global.param_value[2] = -24;
                                gimbal_FSTD_global.param_value[3] = -25;
                                gimbal_FSTD_global.param_value[4] = 100;
                                gimbal_FSTD_global.param_value[5] = 102;
                                gimbal_FSTD_global.param_value[6] = 101;
                                
                                gimbal_FSTD_global.read_param_done = true;
                                
                                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                                gimbal_FSTD_global.mode_test  = JIG_TEST_GIMBAL_V2_MODE_DONE;
                            }
                        }
                    }break;
                }
                
                /// run all mode test
                JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD_private);
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_DONE:
            {
                
                /// get state idle cho lan sau sau
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_IDLE);
                
                    /// button prcess
                JIG_TEST_button_process();
                
                /// xoay gimbal kiem tra feedback imu
                JIG_TEST_gimbal_FSTD_run_feed_back_imu();
                
                /// apply cmd reset from Pi with button back to scan new barCode
                JIG_TEST_gimbal_FSTD_back_to_scan_new_barCode();
                
                /// apply cmd reset from Pi auto back to scan new barCode
                if(gimbal_FSTD_global.cloudData_command.auto_back == true)
                {
                    if(JIG_TEST_gimbal_FSTD_v2_get_cloudData_start() == true)
                    {
                            //// trong qua trinh test raspberry send reset
                        memset(&gimbal_FSTD_global, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_global_t));
                        memset(&gimbal_FSTD_private, 0, sizeof(JIG_TEST_gimbal_FSTD_v2_private_t));
                        
                        /// reset flag storage state
                        JIG_TEST_gimbal_FSTD_v2_reset_flag_storage_state();
                        
                        /// setting lai stiffness profile ship
                        JIG_TEST_gimbal_FSTD_v2_setting_stiffness(STIFFNESS_TYPE_PROFILE_SHIP);
                        
                        /// write to console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write(" AUTO BACK SCAN NEW BARCODE \n");
                    }
                }
            }break;
            case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_ERROR:
            {
                /// get state idle cho lan sau sau
                JIG_TEST_gimbal_FSTD_v2_storage_state_to_backup_resister(JIG_TEST_GIMBAL_V2_STATE_IDLE);
                
                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR))
                {
                    if(gimbal_FSTD_private.error_heartbeat_com2 == true)
                    {
                        /// toggle led red 
                        HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                        
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("COM2 Heartbeat error !!!\n");
                    }
                    
                    if(gimbal_FSTD_global.mode_test >= JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD)
                    {
                        if(gimbal_FSTD_global.profile_ship_error == true)
                        {
                            /// toggle led red 
                            HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                            
                            JIG_TEST_console_write(control_process);
                            HAL_Delay(5);
                            JIG_TEST_console_write("Load profile ship error ........................ !!!\n");
                        }
                    }
                    
                    if(gimbal_FSTD_global.cloudData_command.jig_timeOut == true)
                    {
                        /// toggle led red 
                        HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                        
                        JIG_TEST_console_write(control_process);
                        HAL_Delay(5);
                        JIG_TEST_console_write("jig_timeOut No DOne signal!!!\n");
                    }
                    
                    if(gimbal_FSTD_global.cloudData_command.cant_read_uuid == true)
                    {
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("Can't read uuid !!!\n");
                    }
                }
            }break;
        }
        
        /// reset count console case
        count_console_case = 0;
    }
    else
    {
        if(++count_console_case > 200000)
        {
            count_console_case = 0;
            
            JIG_TEST_console_write(control_process);
            JIG_TEST_console_write("cloudData_heartbeat not Ready\n");
        }
    }
}
#endif

#if (JIG_TEST_ID == 0x34)

/** @brief gimbal_FSTD_v2_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_v2_control_process(void)
{
    char *control_process = "\nCONTROL_PROCESS ---> ";
    static uint32_t count_console_case = 0;
    
    switch((uint8_t)gimbal_FSTD_global.state_test)
    {
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_IDLE:
        {
            /// next state
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_START;
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
        {
            /// waitting start from raspberry
            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2;
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
        {
            /// check com 2
            if(JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2))
            {
                char buff[100];
                
                JIG_TEST_console_write(control_process);
                sprintf(buff, "gimbal ID : %3d", mavlink_gimbal_COM2.vehicle_system_id);
                JIG_TEST_console_write(buff);
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
        {
            static uint32_t count_console = 0;
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_motor() == true)
                {
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
        {
            static uint32_t count_console = 0;
            
            if(++count_console > 150000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(control_process);
                
                /// waitting gimbal startup calib
                if(JIG_TEST_gimbal_FSTD_get_gimbal_startup_calib_imu() == true)
                {
                    char buff[100];
                    /// setting param imu rate
                    gimbal_param_setting[24].value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE;
                    JIG_TEST_mavlink_gimbal_set_param(gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    
                    sprintf(buff, "setting param imu rate : value <-> %f | id : %s\n"
                    , gimbal_param_setting[24].value, gimbal_param_setting[24].param_id);
                    JIG_TEST_console_write(buff);
                    
                    /// next state
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE;
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
        {
            /// kiem tra loai imu dang dung
            gimbal_FSTD_private.imu = JIG_TEST_mavlink_gimbal_get_sensor_name(&mavlink_gimbal_COM2);
            if(gimbal_FSTD_private.imu != GREMSY_SENSOR_ERROR)
            {
                JIG_TEST_console_write(control_process);
                
                if(gimbal_FSTD_private.imu == GREMSY_SENSOR_BMI160)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_BMI160\n");
                }
                else if(gimbal_FSTD_private.imu == GREMSY_SENSOR_ICM42688)
                {
                    JIG_TEST_console_write("GREMSY_SENSOR_ICM42688\n");
                }
                
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM;
            }
            else
            {
                if(++count_console_case > 200000)
                {
                    count_console_case = 0;
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("GREMSY_SENSOR_ERROR\n");
                }
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
        {
            /// setting param gimbal
            if(JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2) == true)
            {
                /// next state
                gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST;
            }
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
        {
            
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST))
            {
                gimbal_FSTD_global.time_test_total ++;
                
                if(gimbal_FSTD_private.is_test_running == true)
                {
                    gimbal_FSTD_private.time_run_test ++;
                    
                    /// toggle led blue
                    if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_FOLLOW_MODE)
                    {
                        HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                    }
                    else if(mavlink_gimbal_COM2.status.mode == GIMBAL_STATE_LOCK_MODE)
                    {
                        HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                    }
                }
            }
            
            /// mode test process
            switch((uint8_t)gimbal_FSTD_global.mode_test)
            {
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_IDLE:
                {
                    // next state
                    gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_SBUS;
                    
                    /// write console
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_SBUS\n");
                    
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_SBUS:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_PPM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_PPM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_PPM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_CAN;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_CAN\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_CAN:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_CAN, JIG_TEST_GIMBAL_MODE_READ_CAN, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_CAN] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_COM4;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_COM4\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_COM4:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_COM4, JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_COM4] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_AUX;
                        
                        /// reset command ack
                        mavlink_gimbal_COM2.ack.command = 0;

                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_AUX\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_AUX:
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                        , JIG_TEST_GIMBAL_V2_MODE_AUX, JIG_TEST_GIMBAL_MODE_READ_COM, 4))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_AUX] = false;
                        
                        /// on motor gimbal next state test vibrate
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_VIBRATE;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_VIBRATE\n");
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                {
                    uint8_t time = 0;
                    
                    if(mavlink_gimbal_COM2.vehicle_system_id == 0x22)
                    {
                        time = 100;
                    }
                    else
                    {
                        time = 12;
                    }
                    
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD_private
                    , JIG_TEST_GIMBAL_V2_MODE_VIBRATE, JIG_TEST_GIMBAL_MODE_READ_COM, time))
                    {
                        /// reset many available for after test
                        gimbal_FSTD_private.is_test_running = false;
                        gimbal_FSTD_private.time_run_test = 0;
                        gimbal_FSTD_private.mode_test_process[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] = false;
                        
                        // next state
                        gimbal_FSTD_global.mode_test = JIG_TEST_GIMBAL_V2_MODE_DONE;
                        
                        /// write console
                        JIG_TEST_console_write(control_process);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_MODE_DONE\n");
                    }
                    else
                    {
                        if(++ count_console_case > 200000)
                        {
                            count_console_case = 0;
                            
                            /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                            if(mavlink_gimbal_COM2.status.mode == 0)
                            {
                                /// on motor gimbal in mode test vibration
                                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                            }
                        }
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_DONE:
                {
                    static bool stateDone = false;
                    
                    if(stateDone == false)
                    {
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_SBUS"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_SBUS]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PPM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PPM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_CAN"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_CAN]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_COM4"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_COM4]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_AUX"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_VIBRATE"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE]);
                        
                        JIG_TEST_gimbal_FSTD_write_to_console_result_mode_test("JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD"
                        , gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD]);
                        
                        /// turn off motor gimbal
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        
                        stateDone = true;
                        
                    }
                    else
                    {
                        static bool setImuStartupCalib = false;
        
                        if(setImuStartupCalib == false)
                        {
                            setImuStartupCalib = JIG_TEST_gimbal_FSTD_v2_set_calib_startup_imu_status(true);
                        }
                        else
                        {
                            setImuStartupCalib = false;
                            stateDone = false;
                            
                        
                            JIG_TEST_console_write(control_process);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_V2_STATE_DONE\n");
                            HAL_Delay(10);
                            
                            /// next state
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                        }
                    }
                }break;
                case (uint8_t)JIG_TEST_GIMBAL_V2_MODE_LOOP:
                {
                    static uint8_t count_wait_loop = 0;
                    if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP))
                    {
                        if(++count_wait_loop > 5)
                        {
                            gimbal_FSTD_global.param_value[1] = -23;
                            gimbal_FSTD_global.param_value[2] = -24;
                            gimbal_FSTD_global.param_value[3] = -25;
                            gimbal_FSTD_global.param_value[4] = 100;
                            gimbal_FSTD_global.param_value[5] = 102;
                            gimbal_FSTD_global.param_value[6] = 101;
                            
                            gimbal_FSTD_global.read_param_done = true;
                            
                            gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_DONE;
                            gimbal_FSTD_global.mode_test  = JIG_TEST_GIMBAL_V2_MODE_DONE;
                        }
                    }
                }break;
            }
            
            /// run all mode test
            JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD_private);
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_DONE:
        {
                /// button prcess
            JIG_TEST_button_process();
            
            /// xoay gimbal kiem tra feedback imu
            JIG_TEST_gimbal_FSTD_run_feed_back_imu();
        }break;
        case (uint8_t)JIG_TEST_GIMBAL_V2_STATE_ERROR:
        {
            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR))
            {
                if(gimbal_FSTD_private.error_heartbeat_com2 == true)
                {
                    /// toggle led red 
                    HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
                    
                    JIG_TEST_console_write(control_process);
                    JIG_TEST_console_write("COM2 Heartbeat error !!!\n");
                }
            }
        }break;
    }
}

#endif

/*
    End group
*/
#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_V2_MAIN_PROCESS
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_V2_MAIN_PROCESS
#define JIG_TEST_GIMBAL_FSTD_V2_MAIN_PROCESS

/** @brief gimbal_FSTD_v2_main_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_v2_main_process(void)
{
    /// reset time process
    calculator_reset_time(&gimbal_FSTD_private.time_process_);
    
    /// send & reciever data from COM2, COM4, RASPBERRY
    JIG_TEST_mavlink_gimbal_process();
    
    #if (RUN_SAMPLE_CODE == 1)
        static bool setImuStartupCalib = false;
        
        if(setImuStartupCalib == false)
        {
            setImuStartupCalib = JIG_TEST_gimbal_FSTD_v2_set_calib_startup_imu_status(false);
        }
    
    #else
    
        /// main control jig test process
        JIG_TEST_gimbal_FSTD_v2_control_process();
        
        #if (JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
            /// check raspberry heartbeat timeOut process
            JIG_TEST_comm_raspberry_v2_heartbeat_timeOut_process();
        #endif
        
        #if (JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x21)
            /// raspberry cloud data process
            JIG_TEST_comm_raspberry_v2_cloudData_process();
        #endif
        
        #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
            /// usb speed test process
            JIG_TEST_usb_speed_process();
        #endif
        /// display process
        if(get_timeOut(200, JIG_TEST_DISPLAY_PROCESS))
        JIG_TEST_display_v2_process();
        
        /// gpio aux result ok process
        JIG_TEST_aux_process();
    
    #endif

    /// calculation time process
    gimbal_FSTD_private.time_process = calculator_get_time_us(&gimbal_FSTD_private.time_process_);
}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


