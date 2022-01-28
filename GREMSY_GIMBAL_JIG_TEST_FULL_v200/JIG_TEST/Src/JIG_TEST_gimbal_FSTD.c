/**
  ******************************************************************************
  * @file JIG_TEST_gimbal_FSTD.c
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
#include "JIG_TEST_gimbal_FSTD.h"
#include "JIG_TEST_ppm_gimbal.h"
#include "JIG_TEST_can_dji_.h"
#include "JIG_TEST_sbus_gimbal.h"
#include "JIG_TEST_display.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_comm_raspberry.h"
#include "JIG_TEST_button.h"
#include "JIG_TEST_rtc.h"

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

typedef enum _JIG_TEST_gimbal_FSTD_test_state_t
{
    JIG_TEST_GIMBAL_FSTD_STATE_IDLE,
    JIG_TEST_GIMBAL_FSTD_STATE_WAIT_COM2,
    JIG_TEST_GIMBAL_FSTD_STATE_SETTING_PARAM,
    JIG_TEST_GIMBAL_FSTD_STATE_WAIT_START_FROM_RASP,
    JIG_TEST_GIMBAL_FSTD_STATE_CONTROL,
    JIG_TEST_GIMBAL_FSTD_STATE_ERROR,
    JIG_TEST_GIMBAL_FSTD_STATE_DONE,
    JIG_TEST_GIMBAL_FSTD_STATE_LOOP,
    
}JIG_TEST_gimbal_FSTD_test_state_t;


/* ratio gimbal
    readonly property int _rc_sync: 7
    readonly property int _rc_sbus_fasst: 1
    readonly property int _rc_spektrum_10bits: 5
    readonly property int _rc_spektrum_11bits: 4
    readonly property int _rc_ppm: 6
    readonly property int _rc_joystick: 2
    readonly property int _rc_pc: 3
    readonly property int _rc_calib: 8
    readonly property int _rc_timelapse: 9
    readonly property int _rc_lb2_single: 10
    readonly property int _rc_lb2_normal: 11
    readonly property int _rc_sync2: 12
    readonly property int _rc_sbus_sfhss: 13
    readonly property int _rc_nothing: 14
*/

typedef enum _JIG_TEST_gimbal_FSTD_mode_read_t
{
    JIG_TEST_GIMBAL_MODE_READ_IDLE,
    JIG_TEST_GIMBAL_MODE_READ_SBUS = 1,
    JIG_TEST_GIMBAL_MODE_READ_PPM = 6,
    JIG_TEST_GIMBAL_MODE_READ_CAN = 11,
    JIG_TEST_GIMBAL_MODE_READ_COM = 14,
    
}JIG_TEST_gimbal_FSTD_mode_read_t;


typedef enum _JIG_TEST_gimbal_FSTD_timeOut_task_t
{
    HEARTBEAT,
    LED_RED_JIG_TEST_FSTD,
    MAVLINK_COM2_MOUNT_VALUE_ANGLE,
    RAW_IMU_XGYRO_CACULATOR,
    RAW_IMU_YGYRO_CACULATOR,
    RAW_IMU_ZGYRO_CACULATOR,
    DISPLAY_PROCESS,
    JIG_TEST_GIMBAL_FSTD_WAIT_RESET_SYS_SECTION_MORE,
    JIG_TEST_GIMBAL_FSTD_WAIT_INIT_AFTER_RESET,
    JIG_TEST_GIMBAL_FSTD_WAIT_SET_PARAM,
    JIG_TEST_GIMBAL_FSTD_WAIT_READ_PARAM,
    
}JIG_TEST_gimbal_FSTD_timeOut_task_t;

typedef enum _JIG_TEST_gimbal_FSTD_mavlink_msg_send_t
{
    FSTD_MAVLINK_MSG_SET_NONE                   = 0,
    FSTD_MAVLINK_MSG_SET_PARAM                  = 1,
    FSTD_MAVLINK_MSG_SEND_PARAM_REQUEST_READ    = 2,
    FSTD_MAVLINK_MSG_SET_CONTROL_MOTOR          = 3,
    FSTD_MAVLINK_MSG_SET_MOVE                   = 4,
    FSTD_MAVLINK_MSG_SET_MODE                   = 5,
    FSTD_MAVLINK_MSG_SET_HOME                   = 6,
    FSTD_MAVLINK_MSG_SET_RC_INPUT               = 7,
    FSTD_MAVLINK_MSG_SET_REBOOT                 = 8,
    FSTD_MAVLINK_MSG_SET_MODE_RATIO             = 9,
    
}JIG_TEST_gimbal_FSTD_mavlink_msg_send_t;

typedef enum _JIG_TEST_gimbal_FSTD_mavlink_msg_manager_t
{
    FSTD_MAVLINK_MSG_TYPE_SET = 0x01,
    FSTD_MAVLINK_MSG_TYPE_RESET = 0x02
    
}JIG_TEST_gimbal_FSTD_mavlink_msg_manager_t;

typedef struct _JIG_TEST_gimbal_FSTD_timeOut_t
{
    uint32_t time;
    uint8_t heartbeat;
    uint8_t angle;
    
    uint32_t time_process;
    uint32_t _time_process;
    
}JIG_TEST_gimbal_FSTD_timeOut_t;

typedef struct _JIG_TEST_gimbal_FSTD_led_status_t
{ 
    uint32_t time_led_red;
    uint32_t time_led_blue;
    uint32_t time_led_green;
    
}JIG_TEST_gimbal_FSTD_led_status_t;


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

typedef struct _JIG_TEST_gimbal_FSTD_request_param_read_t
{
    uint16_t param_index;
    char *param_id;
    
}JIG_TEST_gimbal_FSTD_request_param_read_t;

typedef struct _JIG_TERST_gimbal_FSTD_set_move_t
{
    int16_t tilt;
    int16_t roll;
    int16_t pan;
    
}JIG_TERST_gimbal_FSTD_set_move_t;

typedef struct _JIG_TEST_gimbal_FSTD_private_t
{
    uint32_t timeOut_[25];
    
    JIG_TEST_gimbal_FSTD_timeOut_t      timeOut;
    JIG_TEST_gimbal_FSTD_led_status_t   led_status;
    
    bool gimbal_ready;
    bool is_COM2_ready;
    uint8_t wait_calib_ready;
    bool re_test;
    bool re_init_dma1_stream3;
    bool timeOut_dma1_stream3_process;
    
    uint8_t param_readCount;
    uint16_t param_value;
    bool is_settingParam;
    bool is_compareParam;
    
    bool gimbal_is_home;
    bool gimbal_is_setMODE;
    uint32_t time_mode_test;
    uint8_t apply_mode_test[JIG_TEST_GIMBAL_MODE_TOTAL];
    uint8_t mode_test_process[JIG_TEST_GIMBAL_MODE_TOTAL];
    bool mode_test_result[JIG_TEST_GIMBAL_MODE_TOTAL];
    uint8_t test_done[JIG_TEST_GIMBAL_MODE_TOTAL];
    uint32_t time_run_test;
    
    bool is_test_running;
    
    bool control_lb2_dual_mode;
    
    uint16_t aux_test_count[4];
    bool aux_test_result;
    

    JIG_TEST_gimbal_FSTD_mavlink_msg_send_t     mavlink_msg_manager;
    JIG_TEST_gimbal_FSTD_request_param_read_t   request_param_read;
    control_gimbal_motor_t                      motor_control;
    JIG_TERST_gimbal_FSTD_set_move_t            set_move;
    control_gimbal_mode_t                       set_mode;
    remote_control_gimbal_t                     rc_input;
    JIG_TEST_gimbal_FSTD_mode_read_t            mode_ratio;
    
    JIG_TEST_gimbal_FSTD_test_state_t           state_FSTD;
    JIG_TEST_gimbal_FSTD_mavlink_msg_manager_t  msg_manager;
    JIG_TEST_gimbal_FSTD_mode_test_t            mode_test;
    JIG_TEST_gimbal_FSTD_mode_read_t            mode_read;
    JIG_TEST_gimbal_FSTD_vibrate_t              vibrate;
    
}JIG_TEST_gimbal_FSTD_private_t;

/* Private define------------------------------------------------------------------------------*/

#define USE_VIBRATE  1

#define JIG_TEST_GIMBAL_FSTD_LED_RED_TOGGLE     HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin)
#define JIG_TEST_GIMBAL_FSTD_LED_RED_OFF        HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET)
#define JIG_TEST_GIMBAL_FSTD_LED_RED_ON         HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_RESET)

#define JIG_TEST_GIMBAL_FSTD_LED_GREEN_TOGGLE   HAL_GPIO_TogglePin(green_GPIO_Port, green_Pin)
#define JIG_TEST_GIMBAL_FSTD_LED_GREEN_OFF      HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET)
#define JIG_TEST_GIMBAL_FSTD_LED_GREEN_ON       HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_RESET)

#define JIG_TEST_GIMBAL_FSTD_NUMBER_OF_PARAM        36
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP           44
#define JIG_TEST_GIMBAL_FSTD_NUMBER_PARAM_COMPARE   15

#define COMMAND_CONTROL_MOTOR       31010
#define COMMAND_SETTING_MODE        31011
#define MAVLINK_RESULT_ACCEPTED     0
#define COMMAND_DO_MOUNT_CONFIG     204
#define COMMAND_DO_MOUNT_CONTROL    205

#define TIME_TEST_SBUS      5000
#define TIME_TEST_PPM       5000
#define TIME_TEST_CAN       5000
#define TIME_TEST_COM2      5000
#define TIME_TEST_COM4      5000
#define TIME_TEST_AUX       5000

#define MAIN_JIG_TEST           1

#define USE_STD_DEV_FOR_IMU     1

#define JIG_TEST_GIMBAL_FSTD_VIBTATE_LIMIT_COUNT    3

/** @group JIG_TEST_GIMBAL_USER_PROFILE_SHIP_DEFINE
    @{
*/#ifndef JIG_TEST_GIMBAL_USER_PROFILE_SHIP_DEFINE
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_DEFINE

#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_P            25
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_P             25
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_P              25
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_I              4
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_GYRO_LPF           2
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_POWER        40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_POWER         40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_POWER          40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_I            120
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_DIR_MOTOR_ROLL     0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_FOLLOW       100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_FILTER       50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_TILT_WINDOW        5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_D             1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_FOLLOW         100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_FILTER         50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PAN_WINDOW         5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_DIR_MOTOR_PITCH    0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_GYRO_TRUST         210
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_TRIM      0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_TRIM       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_MODE_CHAN     5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_PITCH_CHAN    1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_LPF       50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_MODE      1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_ROLL_CHAN     9
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_LPF        50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_MODE       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_YAW_CHAN      3
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_YAW_LPF         60
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_YAW_MODE        1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_D            4
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_D              4

#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_P            40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_P             40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_P              50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_I              3
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_GYRO_LPF           2
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_POWER        30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_POWER         30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_POWER          35
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_I            120
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_DIR_MOTOR_ROLL     0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_FOLLOW       100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_FILTER       50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_TILT_WINDOW        5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_D             0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_FOLLOW         100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_FILTER         50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PAN_WINDOW         5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_DIR_MOTOR_PITCH    0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_GYRO_TRUST         220
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_TRIM      0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_TRIM       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_MODE_CHAN     6
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_PITCH_CHAN    1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_LPF       30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_MODE      1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_ROLL_CHAN     9
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_LPF        30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_MODE       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_YAW_CHAN      3
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_YAW_LPF         30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_YAW_MODE        1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_D            2
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_D              2

#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_P            40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_P             40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_P              40
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_I              4
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_GYRO_LPF           2
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_POWER        30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_POWER         30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_POWER          30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_I            120
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_DIR_MOTOR_ROLL     0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_FOLLOW       100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_FILTER       50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_TILT_WINDOW        5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_D             0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_FOLLOW         100
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_FILTER         50
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PAN_WINDOW         5
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_DIR_MOTOR_PITCH    0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_GYRO_TRUST         220
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_TRIM      0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_TRIM       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_MODE_CHAN     6
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_PITCH_CHAN    1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_LPF       30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_MODE      1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_ROLL_CHAN     7
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_LPF        30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_MODE       0
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_YAW_CHAN      3
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_YAW_LPF         30
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_YAW_MODE        1
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_D            2
#define JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_D              2

struct _JIG_TEST_gimbal_user_profile_ship_T3
{
    float value;
    
}JIG_TEST_gimbal_user_profile_ship_T3[33] = 
{
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_GYRO_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_DIR_MOTOR_ROLL},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_TILT_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_ROLL_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PAN_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_DIR_MOTOR_PITCH},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_GYRO_TRUST},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_MODE_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_PITCH_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_PITCH_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_ROLL_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_ROLL_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_SBUS_YAW_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_YAW_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_RC_YAW_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_PITCH_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T3_YAW_D},
};

struct _JIG_TEST_gimbal_user_profile_ship_S1v3
{
    float value;
    
}JIG_TEST_gimbal_user_profile_ship_S1v3[33] = 
{
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_GYRO_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_DIR_MOTOR_ROLL},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_TILT_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_ROLL_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PAN_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_DIR_MOTOR_PITCH},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_GYRO_TRUST},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_MODE_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_PITCH_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_PITCH_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_ROLL_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_ROLL_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_SBUS_YAW_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_YAW_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_RC_YAW_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_PITCH_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_S1V3_YAW_D},
};

struct _JIG_TEST_gimbal_user_profile_ship_T7
{
    float value;
    
}JIG_TEST_gimbal_user_profile_ship_T7[33] = 
{
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_P},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_GYRO_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_POWER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_I},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_DIR_MOTOR_ROLL},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_TILT_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_ROLL_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_FOLLOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_FILTER},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PAN_WINDOW},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_DIR_MOTOR_PITCH},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_GYRO_TRUST},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_TRIM},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_MODE_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_PITCH_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_PITCH_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_ROLL_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_ROLL_MODE},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_SBUS_YAW_CHAN},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_RC_YAW_LPF},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_D },
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_PITCH_D},
    {.value = (float)JIG_TEST_GIMBAL_USER_PROFILE_SHIP_T7_YAW_D},
};

#endif
/**
    @}
*/


/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

JIG_TEST_gimbal_FSTD_private_t  gimbal_FSTD;
JIG_TEST_gimbal_FSTD_t          gimbal_FSTD_comm;

extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM2;
extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM4;

extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern JIG_TEST_comm_raspberry_global_t            raspberry_global;

/** @group JIG_TEST_GIMBAL_PARAM_VARIABLE
    @{
*/#ifndef JIG_TEST_GIMBAL_PARAM_VARIABLE
#define JIG_TEST_GIMBAL_PARAM_VARIABLE

struct param_gimbal_set
{
    uint16_t value;
    float value_param_get;
    int16_t index;
    char* param_id;
}param_gimbal[JIG_TEST_GIMBAL_FSTD_NUMBER_OF_PARAM] =
{
    {.value = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 7, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
    {.value = 1, .index = 39, .param_id = "SBUS_PITCH_CHAN"},
    {.value = 6, .index = 42, .param_id = "SBUS_MODE_CHAN"},
    {.value = 2, .index = 10, .param_id = "YAW_D"},
    {.value = 2, .index = 4, .param_id = "PITCH_D"},
    {.value = 40, .index = 23, .param_id = "NPOLES_YAW"},
    {.value = 40, .index = 22, .param_id = "NPOLES_ROLL"},
    {.value = 40, .index = 21, .param_id = "NPOLES_PITCH"},
    {.value = 90, .index = 8, .param_id = "YAW_P"},
    {.value = 80, .index = 5, .param_id = "ROLL_P"},
    {.value = 70, .index = 2, .param_id = "PITCH_P"},
    {.value = 40, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 40, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 40, .index = 13, .param_id = "YAW_POWER"},
    {.value = 1, .index = 63, .param_id = "JOY_AXIS"}, /// ???
    {.value = 60, .index = 60, .param_id = "RC_PITCH_SPEED"},
    {.value = 60, .index = 61, .param_id = "RC_ROLL_SPEED"},
    {.value = 60, .index = 62, .param_id = "RC_YAW_SPEED"},
    {.value = 2, .index = 29, .param_id = "GYRO_LPF"}, // gyro filter for BMI
//    {.value = 4, .index = 29, .param_id = "GYRO_LPF"}, // gyro filter for ICM
    {.value = 4, .index = 9, .param_id = "YAW_I"},
    {.value = 120, .index = 3, .param_id = "PITCH_I"},
    {.value = 210, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 15, .index = 77, .param_id = "IMU_RATE"},
    {.value = 1, .index = 72, .param_id = "HEARTBEAT_EMIT"},
    {.value = 10, .index = 73, .param_id = "STATUS_RATE"},
    {.value = 10, .index = 74, .param_id = "ENC_CNT_RATE"},
    {.value = 10, .index = 76, .param_id = "ORIEN_RATE"},
    
    {.value = 0, .index = 46, .param_id = "GYROX_OFFSET"},
    {.value = 0, .index = 47, .param_id = "GYROY_OFFSET"},
    {.value = 0, .index = 48, .param_id = "GYROZ_OFFSET"},
    {.value = 0, .index = 43, .param_id = "ACCX_OFFSET"},
    {.value = 0, .index = 44, .param_id = "ACCY_OFFSET"},
    {.value = 0, .index = 45, .param_id = "ACCZ_OFFSET"},
    {.value = 0, .index = 71, .param_id = "GIMBAL_OVAL"},
    
};

struct _gimbal_user_profile_ship
{
    float value;
    float value_param_get;
    int16_t index;
    char* param_id;
}gimbal_user_profile_ship[JIG_TEST_GIMBAL_USER_PROFILE_SHIP] = 
{
    {.value = 0, .index = 100, .param_id = "       "},
    {.value = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .index = 67, .param_id = "VERSION_Y"},
    {.value = 0, .index = 68, .param_id = "VERSION_Z"},
    {.value = 0, .index = 2, .param_id = "PITCH_P"},
    {.value = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .index = 8, .param_id = "YAW_P"},
    {.value = 0, .index = 9, .param_id = "YAW_I"},
    {.value = 0, .index = 29, .param_id = "GYRO_LPF"},
    {.value = 0, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 0, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 0, .index = 13, .param_id = "YAW_POWER"},
    {.value = 0, .index = 3, .param_id = "PITCH_I"},
    {.value = 0, .index = 25, .param_id = "DIR_MOTOR_ROLL"},
    {.value = 0, .index = 14, .param_id = "PITCH_FOLLOW"},
    {.value = 0, .index = 17, .param_id = "PITCH_FILTER"},
    {.value = 0, .index = 57, .param_id = "TILT_WINDOW"},
    {.value = 0, .index = 7, .param_id = "ROLL_D"},
    {.value = 0, .index = 16, .param_id = "YAW_FOLLOW"},
    {.value = 0, .index = 19, .param_id = "YAW_FILTER"},
    {.value = 0, .index = 58, .param_id = "PAN_WINDOW"},
    {.value = 0, .index = 24, .param_id = "DIR_MOTOR_PITCH"},
    {.value = 0, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 0, .index = 51, .param_id = "RC_PITCH_TRIM"},
    {.value = 0, .index = 52, .param_id = "RC_ROLL_TRIM"},
    {.value = 0, .index = 42, .param_id = "SBUS_MODE_CHAN"},
    {.value = 0, .index = 39, .param_id = "SBUS_PITCH_CHAN"},
    {.value = 0, .index = 36, .param_id = "RC_PITCH_LPF"},
    {.value = 0, .index = 54, .param_id = "RC_PITCH_MODE"},
    {.value = 0, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
    {.value = 0, .index = 37, .param_id = "RC_ROLL_LPF"},
    {.value = 0, .index = 55, .param_id = "RC_ROLL_MODE"},
    {.value = 0, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 0, .index = 38, .param_id = "RC_YAW_LPF"},
    {.value = 0, .index = 56, .param_id = "RC_YAW_MODE"},
    {.value = 0, .index = 4, .param_id = "PITCH_D"},
    {.value = 0, .index = 10, .param_id = "YAW_D"},//36
    
    {.value = 0, .index = 46, .param_id = "GYROX_OFFSET"},
    {.value = 0, .index = 47, .param_id = "GYROY_OFFSET"},
    {.value = 0, .index = 48, .param_id = "GYROZ_OFFSET"},
    {.value = 0, .index = 43, .param_id = "ACCX_OFFSET"},
    {.value = 0, .index = 44, .param_id = "ACCY_OFFSET"},
    {.value = 0, .index = 45, .param_id = "ACCZ_OFFSET"},
    {.value = 0, .index = 71, .param_id = "GIMBAL_OVAL"},

};

#endif
/**
    @}
*/



/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_GIMBAL_FAC30K_CONFIGURATION
    @{
*/#ifndef JIG_TEST_GIMBAL_FAC30K_CONFIGURATION
#define JIG_TEST_GIMBAL_FAC30K_CONFIGURATION

/** @brief gimbal_FAC30K_configuration
    @return none
*/
static void JIG_TEST_gimbal_FAC30K_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, AUX_S8_Pin | AUX_S9_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : AUX_S6_Pin AUX_S7_Pin */
    GPIO_InitStruct.Pin = AUX_S6_Pin | AUX_S7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : AUX_S8_Pin AUX_S9_Pin */
    GPIO_InitStruct.Pin = AUX_S8_Pin | AUX_S9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_CONFIGURATION
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_CONFIGURATION
#define JIG_TEST_GIMBAL_FSTD_CONFIGURATION

/** @brief gimbal_FSTD_configuration
    @return none
*/
void JIG_TEST_gimbal_FSTD_configuration(void)
{
    char buff[300];
    
    /// read variables from backup register
    raspberry_global.count_start_test           = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR10);
    gimbal_FSTD_comm.scan_barCode_done          = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR11);
//    gimbal_FSTD_comm.display_gimbal_id_storage  = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR12);
    gimbal_FSTD_comm.user_logined               = JIG_TEST_rtc_get_value_from_backup_register(LL_RTC_BKP_DR13);
    
    timeOut_configuration();
    JIG_TEST_display_configuration();
    JIG_TEST_ppm_gimbal_configuration();
    JIG_TEST_can_dji_configuration();
    JIG_TEST_sbus_gimbal_configuration();
    JIG_TEST_mavlink_gimbal_configuration();
    JIG_TEST_console_configuration();
    JIG_TEST_button_configuration();
    JIG_TEST_comm_raspberry_configuration();

    #if (GIMBAL_FSTD_JIG_TEST == 0)
    
        JIG_TEST_gimbal_FAC30K_configuration();

    #endif
    /// reset all led
    HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
    
    /// config gimbal_FSTD param test
    gimbal_FSTD.timeOut.time = 20000;
    gimbal_FSTD.led_status.time_led_blue = 1000;
    mavlink_gimbal_COM2.ack.command = 0;
    
    /// write to console init
    sprintf(buff, "GREMSY GIMBAL JIG TEST\n         FSTD\n         v200\n");
    JIG_TEST_console_write(buff);
}

#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_CONTROL
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_CONTROL
#define JIG_TEST_GIMBAL_FSTD_CONTROL

/** @brief timeOut_get_ms
    @return 1ms
*/
static uint32_t JIG_TEST_gimbal_FSTD_timeOut_get_ms(void)
{
    return (__HAL_TIM_GET_COUNTER(&htim5) / 1000);
}

/** @brief timeOut_reset_ms
    @return 1ms
*/
static uint32_t JIG_TEST_gimbal_FSTD_timeOut_reset(JIG_TEST_gimbal_FSTD_timeOut_task_t task)
{
    gimbal_FSTD.timeOut_[HEARTBEAT] = __HAL_TIM_GET_COUNTER(&htim5);
}

/** @brief gimbal_FSTD_get_timeOut
    @return true : done timeOut
*/
static bool JIG_TEST_gimbal_FSTD_get_timeOut(uint32_t time, JIG_TEST_gimbal_FSTD_timeOut_task_t task)
{
    bool ret = false;
    uint32_t temp = JIG_TEST_gimbal_FSTD_timeOut_get_ms() - gimbal_FSTD.timeOut_[(uint8_t)task];

    if(temp >= time)
    {
        gimbal_FSTD.timeOut_[(uint8_t)task] = JIG_TEST_gimbal_FSTD_timeOut_get_ms();
        
        ret = true;
    }

    return ret;
}

/** @brief timeOut_heartbeat
    @return true : da nhan duoc heartbeat, angle, state
            false : dang timeOut heartbeat
*/
static bool JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(JIG_TEST_mavlink_gimbal_t *gimbal_channel, JIG_TEST_gimbal_FSTD_private_t *FSTD)
{
    bool ret = false;
    static uint8_t state;
    static uint8_t heartbeat_timeOut_Count_printConsole;
    static uint8_t timeOut_angle_count;
    static uint8_t timeOut_heartbeat = 0;
    char *str = "\nCONNECTION_GIMBAL_COM2 --->";
    
    if(gimbal_channel->seen_heartbeat == false)
    {
        /// ktra thoi gian 5s timeOut heartbeat
        if(JIG_TEST_gimbal_FSTD_get_timeOut(1000, HEARTBEAT))
        {
            
            if(timeOut_heartbeat ++ > 5) 
            {
                timeOut_heartbeat = 0;
                
                /// re init mavlink serialPort3
                JIG_TEST_mavlink_serialPort3_Reinit();
                
                if(FSTD->timeOut.heartbeat ++ > 10)
                /// next state error
                FSTD->state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_ERROR;
            }
        }
        
        /// toggle led red 0.2s check timeOut heartbeat
        gimbal_FSTD.led_status.time_led_red = 200;
        if(JIG_TEST_gimbal_FSTD_get_timeOut(gimbal_FSTD.led_status.time_led_red, LED_RED_JIG_TEST_FSTD))
        {
            JIG_TEST_GIMBAL_FSTD_LED_RED_TOGGLE;
            if(++heartbeat_timeOut_Count_printConsole > 5)
            {
                heartbeat_timeOut_Count_printConsole = 0;
                
                JIG_TEST_console_write(str);
                JIG_TEST_console_write("not found gimbal heartbeat !!!\n");
            }
        }
        
        /// set state 1
        state = 1;
    }
    else
    {
        /// setting thoi gian timeOut angle
        FSTD->timeOut.time = 2000;
        
        /// set state 2
        state = 2;
        
//        if(gimbal_channel->mount_val.pitch == 0.00f
//            && gimbal_channel->mount_val.roll == 0.00f
//            && gimbal_channel->mount_val.yaw == 0.00f
//        )
//        {
//            /// kiem tra thoi gian 2s timeOut angle
//            if(JIG_TEST_gimbal_FSTD_get_timeOut(FSTD->timeOut.time, MAVLINK_COM2_MOUNT_VALUE_ANGLE))
//            {
//                
//                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_IMU_ANGLE))
//                {
//                    timeOut_angle_count ++;
//                    
//                    JIG_TEST_console_write(str);
//                    HAL_Delay(10);
//                    JIG_TEST_console_write("not gimbal angle !!!\n");
//                }
//                
//                if(timeOut_angle_count > 10)
//                {
//                    timeOut_angle_count = 0;
//                    
//                    FSTD->timeOut.angle = true;

//                    /// next state error
//                    FSTD->state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_ERROR;
//                }
//            }
//        }
//        else
//        {
            /// set state 3
            state = 3;
            
//            JIG_TEST_console_write(str);
//            HAL_Delay(10);
//            JIG_TEST_console_write("got gimbal heartbeat & angle !!!\n");
//        }
    }
    
    /// setting result
    if(state == 3)
    {
        /// OFF led red
        JIG_TEST_GIMBAL_FSTD_LED_RED_OFF;
        
        /// reset state
        state = 0;
        
        ret = true;
    }
    
    return ret;
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
    
    
//    uint16_t command = gimbal_channel->ack.command;
    
    if(state == 0)
    {
        if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
        {
            gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_HOME;
        }
        
        state = 1;
    }
    else if(state == 1)
    {
        int16_t delta_pitch = (int16_t)gimbal_channel->mount_val.pitch;
        int16_t delta_roll = (int16_t)gimbal_channel->mount_val.roll;
        int16_t delta_yaw = (int16_t)gimbal_channel->mount_val.yaw;
        
        if(abs(delta_pitch) < 5 && abs(delta_roll) == 0 && abs(delta_yaw) < 5)
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
static JIG_TEST_gimbal_FSTD_mode_read_t JIG_TEST_gimbal_FSTD_get_mode_control(JIG_TEST_mavlink_gimbal_t *gimbal_channel, JIG_TEST_gimbal_FSTD_mode_read_t mode_set, JIG_TEST_gimbal_FSTD_mode_test_t mode_test)
{
    JIG_TEST_gimbal_FSTD_mode_read_t mode_read = JIG_TEST_GIMBAL_MODE_READ_IDLE;

    char *str = "\nSETTING_MODE_TEST --->";
    uint16_t len = strlen(str);
    static bool retry_set_param;
//    static bool retry_param_request_read;
    static uint8_t timeOut_set_mode;
    static bool app_mode_read[4];
    char buff[50];
    
    
//    /// setting mode test
//    JIG_TEST_mavlink_gimbal_set_param((uint16_t)mode_set, "RADIO_TYPE");
//    
//    /// read mode from gimbal
//    JIG_TEST_mavlink_gimbal_send_param_request_read(28, "RADIO_TYPE");
    
    
    if((uint8_t)mode_test >= (uint8_t)JIG_TEST_GIMBAL_MODE_AUX) /// mode aux khong can request param ratio
    {
        sprintf(buff, "GIMBAL_RC_MODE_COM\n");
        mode_read = JIG_TEST_GIMBAL_MODE_READ_COM;
    }
    else
    {
        
        if(retry_set_param == false)
        {
            retry_set_param = true;
            
//            write_console = true;
            
            if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
            {
                gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_MODE_RATIO;
                
                /// set value msg
                gimbal_FSTD.mode_ratio = mode_set;
            }
        }
        
        if(timeOut_set_mode > 2 && retry_set_param == true)
        {
            retry_set_param = false;
            timeOut_set_mode = 0;
            
            JIG_TEST_console_write(str);
            HAL_Delay(5);
            JIG_TEST_console_write("timeOut set mode --- > set mode Try again .....\n");
        }
        
        JIG_TEST_mavlink_gimbal_send_param_request_read(28, "RADIO_TYPE");

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
//                    JIG_TEST_mavlink_gimbal_set_mode(LOCK_MODE);
                if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
                {
                    /// set send msg
                    gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_MODE;
                    
                    /// set value msg
                    gimbal_FSTD.set_mode = LOCK_MODE;
                }
                
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
            float setpoint_pitch  = 40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = 170.0;
            
            if(++count_state > 100000)
            {
                count_state = 0;

                sprintf(buff, "Control gimbal's yaw cw follow mode! %d -- | -- %d\n", gimbal_channel->ack.command, gimbal_channel->ack.result);
                JIG_TEST_console_write(buff_control_gimbal);
                JIG_TEST_console_write(buff);
                
                /// set command gimbal move
                if(comm_channel == 2)
                {
//                    JIG_TEST_mavlink_gimbal_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                    if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
                    {
                        /// set send msg
                        gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_MOVE;
                        
                        /// set value msg
                        gimbal_FSTD.set_move.tilt = setpoint_pitch;
                        gimbal_FSTD.set_move.roll = setpoint_roll;
                        gimbal_FSTD.set_move.pan = setpoint_yaw;
                    }
                }
                else if(comm_channel == 4)
                {
                    JIG_TEST_mavlink_gimbal_com4_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                
            }

            if(get_timeOut(3000, JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL))
            {
//                if(gimbal_channel->ack.result == MAVLINK_RESULT_ACCEPTED)
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                int16_t delta_yaw = (int16_t)(gimbal_channel->mount_val.yaw - setpoint_yaw);
                
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5 && abs(delta_yaw) < 5)
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
            float setpoint_pitch  = -40.0;
            float setpoint_roll   = 0;
            float setpoint_yaw    = -170.0;
            
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
//                    JIG_TEST_mavlink_gimbal_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                    if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
                    {
                        /// set send msg
                        gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_MOVE;
                        
                        /// set value msg
                        gimbal_FSTD.set_move.tilt = setpoint_pitch;
                        gimbal_FSTD.set_move.roll = setpoint_roll;
                        gimbal_FSTD.set_move.pan = setpoint_yaw;
                    }
                }
                else if(comm_channel == 4)
                {
                    JIG_TEST_mavlink_gimbal_com4_set_move(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                }
                
            }
            
            if(get_timeOut(3000, JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL))
            {
//                if(gimbal_channel->ack.result == MAVLINK_RESULT_ACCEPTED)
                int16_t delta_pitch = (int16_t)(gimbal_channel->mount_val.pitch - setpoint_pitch);
                int16_t delta_roll = (int16_t)(gimbal_channel->mount_val.roll - setpoint_roll);
                int16_t delta_yaw = (int16_t)(gimbal_channel->mount_val.yaw - setpoint_yaw);
                
                if(abs(delta_pitch) < 5 && abs(delta_roll) < 5 && abs(delta_yaw) < 5)
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


/** @group JIG_TEST_GIMBAL_FAC30K
    @{
*/#ifndef JIG_TEST_GIMBAL_FAC30K
#define JIG_TEST_GIMBAL_FAC30K

/** @brief gimbal_FAC30K_get_usb_speed_result
    @return result usb speed
*/
static JIG_TEST_comm_raspberry_USB_speed_test_result_t JIG_TEST_gimbal_FAC30K_get_usb_speed_result(void)
{
    JIG_TEST_comm_raspberry_USB_speed_test_result_t result = JIG_TEST_USB_SPEED_RESULT_NONE;
    
    if(raspberry_global.value_speed_test[2] == (float)JIG_TEST_USB_SPEED_RESULT_NO_USB_FOUND)
    {
        result = JIG_TEST_USB_SPEED_RESULT_NO_USB_FOUND;
    }
    else if(raspberry_global.value_speed_test[2] == (float)JIG_TEST_USB_SPEED_RESULT_LOW_SPEED)
    {
        result = JIG_TEST_USB_SPEED_RESULT_LOW_SPEED;
    }
    else if(raspberry_global.value_speed_test[2] == (float)JIG_TEST_USB_SPEED_RESULT_PASSED)
    {
        result = JIG_TEST_USB_SPEED_RESULT_PASSED;
    }
    
    return result;
}



#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_PARAM_SET
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_PARAM_SET
#define JIG_TEST_GIMBAL_FSTD_PARAM_SET

/** @brief gimbal_FSTD_set_param
    @return true : set param DONE
            false : set param RUNNING
*/
static bool JIG_TEST_gimbal_FSTD_set_param(void)
{
    bool ret = false;
    uint8_t i = 0;
    char buff[100];

    for(i = 0; i < JIG_TEST_GIMBAL_FSTD_NUMBER_OF_PARAM - 7; i++)
    {
        JIG_TEST_mavlink_gimbal_set_param(param_gimbal[i].value, param_gimbal[i].param_id);
        HAL_Delay(50);
        
        sprintf(buff, "\nSETTING_PARAM_GIMBAL ---> %s  <->  value : %3d\n", param_gimbal[i].param_id, param_gimbal[i].value);
        JIG_TEST_console_write(buff);
    }
    ret = true;
    
    return ret;
}


/** @brief gimbal_FSTD_copy_profile_ship_follow_gimbal_id
    @return true : copy param complete
            false : copy param running
*/
static bool JIG_TEST_gimbal_FSTD_copy_profile_ship_follow_gimbal_id(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
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


/** @brief FSTD_request_param_gimbal
    @return true : compare complete
            false : compare running
*/
static bool JIG_TEST_gimbal_FSTD_request_param_gimbal(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    static bool ret = false;
    char *str = "\nGIMBAL_USER_PROFILE_SHIP ---->";
    static bool console_enable = false;
    static bool console_reciever = false;
    static uint32_t count_param_request = 0;
    static uint8_t param_index_temp = 0;
    static bool gimbal_profile_detect = false;
    char buff[300];
    
    if(gimbal_profile_detect == false)
    {
        gimbal_profile_detect = JIG_TEST_gimbal_FSTD_copy_profile_ship_follow_gimbal_id(gimbal_channel);
        
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
            HAL_Delay(15);
            
            if(console_enable == false)
            {
                console_enable = true;
                
                JIG_TEST_console_write(str);
                HAL_Delay(15);
                sprintf(buff, "send request param : %s | index : %3d\n"
                , gimbal_user_profile_ship[count_param_request + 1].param_id
                , gimbal_user_profile_ship[count_param_request + 1].index);
                JIG_TEST_console_write(buff);
            }
        }
        else
        {
            if(console_reciever == false)
            {
                console_reciever = true;
                
                sprintf(buff, "reciever : %s <-> value : %.f | value_set : %.f| count : %3d\n" 
                , gimbal_user_profile_ship[count_param_request + 1].param_id
                , gimbal_user_profile_ship[count_param_request + 1].value_param_get
                , JIG_TEST_gimbal_user_profile_ship_T3[count_param_request - 3].value
                , count_param_request);
                
                JIG_TEST_console_write(str);
                HAL_Delay(5);
                JIG_TEST_console_write(buff);
                HAL_Delay(15);
            }

            
            /// lay gia tri param tu COM2
            if(gimbal_channel->param_value.param_index == gimbal_user_profile_ship[count_param_request + 1].index)
            {
                /// enable console
                console_enable = false;
                
                /// enable consolo reciever
                console_reciever = false;
                
                gimbal_user_profile_ship[count_param_request + 1].value_param_get = gimbal_channel->param_value.param_value;
                
                if(count_param_request <= 3)
                {
                    /// next param 
                    count_param_request ++;
                    
                }
                else if(count_param_request > 3 && count_param_request <= 35)
                {
                    /// compare param read with param set
                    if(gimbal_channel->vehicle_system_id == 0x44) // t3v3
                    {
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_T3[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                    else if(gimbal_channel->vehicle_system_id == 0x22) // s1v3
                    {
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_S1v3[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }
                    else if(gimbal_channel->vehicle_system_id == 0x08) // t7
                    {
                        if(gimbal_user_profile_ship[count_param_request + 1].value_param_get == JIG_TEST_gimbal_user_profile_ship_T7[count_param_request - 3].value)
                        {
                            /// next param 
                            count_param_request ++;
                        }
                    }

                }
                else
                {
                    /// next param 
                    count_param_request ++;

                }

                /// kiem tra so luong param doc duoc
                if(count_param_request == (JIG_TEST_GIMBAL_USER_PROFILE_SHIP - 1))
                {
                    /// re init mavlink comm raspberry
                    JIG_TEST_mavlink_serialPort5_Reinit();
                    
                    ret = true;
                }
            }
        }
    }

    return ret;
}

/** @brief gimbal_FSTD_control_process
    @return true : compare complete
            false : compare running
*/
static bool JIG_TEST_gimbal_FSTD_compare_param_set(JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    bool ret = false;
    static uint8_t paramCompare_count;
    static uint32_t count_state;
    char *str = "\nPARAM_GIMBAL_READ ---->";
    
    JIG_TEST_mavlink_gimbal_send_param_request_read(param_gimbal[gimbal_FSTD.param_readCount].index, param_gimbal[gimbal_FSTD.param_readCount].param_id);
    
    if(++count_state > 200000)
    {
        count_state = 0;
        
        JIG_TEST_console_write(str);
        JIG_TEST_console_write("send param request read msg\n");
    }
    
    if(gimbal_channel->param_value.param_index == param_gimbal[gimbal_FSTD.param_readCount].index)
    {
        /// lay gia tri param tu COM2
        param_gimbal[gimbal_FSTD.param_readCount].value_param_get = gimbal_channel->param_value.param_value;
        
        /// compare param reciever with param read from COM2
        if(param_gimbal[gimbal_FSTD.param_readCount].value_param_get == param_gimbal[gimbal_FSTD.param_readCount].value)
        {
            paramCompare_count++;
        }
        
        char buff[200];

        sprintf(buff, " %s <-> value : %.3f\n", 
        param_gimbal[gimbal_FSTD.param_readCount].param_id, 
        param_gimbal[gimbal_FSTD.param_readCount].value_param_get);
        
        JIG_TEST_console_write(str);
        HAL_Delay(5);
        JIG_TEST_console_write(buff);
        
        gimbal_FSTD.param_readCount++;
    }
    
    if(paramCompare_count == JIG_TEST_GIMBAL_FSTD_NUMBER_PARAM_COMPARE)
    {
        /// reset param count cho lan test tiesp theo
        gimbal_FSTD.param_readCount = 0 ;
        
        for(uint8_t i; i < JIG_TEST_GIMBAL_FSTD_NUMBER_PARAM_COMPARE; i++)
        {
            param_gimbal[i].value_param_get = 0;
        }
        
        ret = true;
    }
    
    return ret;
}


//static 

#endif
/**
    @}
*/

/** @group JIG_TEST_GIMBAL_FSTD_PROCESS
    @{
*/#ifndef JIG_TEST_GIMBAL_FSTD_PROCESS
#define JIG_TEST_GIMBAL_FSTD_PROCESS

/** @brief gimbal_FSTD_apply_mode_test
    @return true : da set xong mode
            false : dang set mode
*/
static bool JIG_TEST_gimbal_FSTD_apply_mode_test(JIG_TEST_gimbal_FSTD_mode_read_t mode)
{
    bool ret = false;
    
    if(gimbal_FSTD.gimbal_is_setMODE == false)
    {
        if(gimbal_FSTD.mode_read == mode)
        {
            gimbal_FSTD.gimbal_is_setMODE = true;
        }
        else
        {
            /// setting mode test cho gimbal
            gimbal_FSTD.mode_read = JIG_TEST_gimbal_FSTD_get_mode_control(&mavlink_gimbal_COM2, mode, gimbal_FSTD.mode_test);
        }
    }
    else if(gimbal_FSTD.gimbal_is_setMODE == true)
    {
        /// kiem tra gimbal da ON chua sau khi reboot
        if(mavlink_gimbal_COM2.status.mode != 0)
        {
            
//            if( mavlink_gimbal_COM2.mount_val.pitch > -5.00f && mavlink_gimbal_COM2.mount_val.pitch < 5.00f
//             && mavlink_gimbal_COM2.mount_val.roll > -5.00f && mavlink_gimbal_COM2.mount_val.roll < 5.00f
//             && mavlink_gimbal_COM2.mount_val.yaw > -5.00f && mavlink_gimbal_COM2.mount_val.yaw < 5.00f
//            )
//            {
                if(gimbal_FSTD.mode_test > JIG_TEST_GIMBAL_MODE_CAN)
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
                    gimbal_FSTD.gimbal_is_setMODE = false;
                    
                    mavlink_gimbal_COM2.ack.command = 0;
                    
                    ret = true;
                }
//            }
        }
    }
    
    return ret;
}

/** @brief gimbal_FSTD_check_mode_test_rc_result
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(JIG_TEST_mavlink_gimbal_t *mavlink_channel, JIG_TEST_gimbal_FSTD_mode_test_t mode_test)
{
    bool ret = false;
    
    if(mode_test == JIG_TEST_GIMBAL_MODE_SBUS)
    {
        if(mavlink_channel->mount_val.pitch < -80 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -150)
        {
            ret = true;
        }
    }
    else if(mode_test == JIG_TEST_GIMBAL_MODE_PPM)
    {
        if(mavlink_channel->mount_val.pitch > 30 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw > 150)
        {
            ret = true;
        }
    }
    else if(mode_test == JIG_TEST_GIMBAL_MODE_CAN)
    {
        if(mavlink_channel->mount_val.pitch < -80 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -150)
        {
            ret = true;
        }
    }
    else if(mode_test == JIG_TEST_GIMBAL_MODE_COM)
    {
        if(mavlink_channel->mount_val.pitch > 30 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw > 120)
        {
            ret = true;
        }
    }
    else if(mode_test == JIG_TEST_GIMBAL_MODE_COM4)
    {
        if(mavlink_channel->mount_val.pitch < -20 && (uint16_t)mavlink_channel->mount_val.roll == 0 && mavlink_channel->mount_val.yaw < -120)
        {
            ret = true;
        }
    }
    else if(mode_test == JIG_TEST_GIMBAL_MODE_VIBRATE)
    {
        if(gimbal_FSTD.vibrate.checkResult == true)
        {
            ret = true;
        }
    }
    
    return ret;
}

/** @brief gimbal_FSTD_mode_test_process
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_mode_test_process(JIG_TEST_gimbal_FSTD_private_t *FSTD, JIG_TEST_gimbal_FSTD_mode_test_t mode_test, JIG_TEST_gimbal_FSTD_mode_read_t mode_read, uint16_t time)
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
            /// set test running tinh thoi gian timeOut
            FSTD->is_test_running = true;
            
            /// mode test process
            FSTD->mode_test_process[mode_test] = true;
            
//            /// chi apply cho mode lb2
//            if(gimbal_FSTD.mode_read == JIG_TEST_GIMBAL_MODE_READ_CAN)
//            {
//                gimbal_FSTD.control_lb2_dual_mode = true;
//            }
            
            if(FSTD->time_run_test > FSTD->time_mode_test)
            {
                /// check result 
                if(mode_test < ((uint8_t)JIG_TEST_GIMBAL_MODE_TOTAL - (uint8_t)JIG_TEST_GIMBAL_MODE_COM4))
                gimbal_FSTD.mode_test_result[mode_test] = JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(&mavlink_gimbal_COM2, mode_test);
                
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
                if(mode_test < JIG_TEST_GIMBAL_MODE_AUX)
                {
                    if(FSTD->time_run_test > 6)
                    {
                        int16_t delta_pitch = (int16_t)(mavlink_gimbal_COM2.mount_val.pitch);
                        int16_t delta_roll = (int16_t)(mavlink_gimbal_COM2.mount_val.roll);
                        int16_t delta_yaw = (int16_t)(mavlink_gimbal_COM2.mount_val.yaw);
                        
                        /// neu dieu khien den goc truoc thoi gian timeOut thi cho ktra goc luoon
                        if(abs(delta_pitch) > 20 && abs(delta_roll) == 0 && abs(delta_yaw) > 150)
                        {
                            
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("get results soon !!!\n");
                            
                            /// check result 
                            gimbal_FSTD.mode_test_result[mode_test] = JIG_TEST_gimbal_FSTD_check_mode_test_rc_result(&mavlink_gimbal_COM2, mode_test);
                            
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

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static double JIG_TEST_gimbal_FSTD_sensor_axis(uint8_t numberOfaxis, int16_t gyro_axis, JIG_TEST_gimbal_FSTD_timeOut_task_t TASK)
{
    char *str = "\nVIBRATE --->";
    char *BMI = "BMI160";
    char *ICM = "ICM42688";
    
    #if (USE_STD_DEV_FOR_IMU == 1)
    
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
        double              result               = 0;
        static uint8_t      state               = 0;
            
            if(state == 0) /// tinh trung binh
            {
                /// kiem tra co ket qua raw imu moi
                if(mavlink_gimbal_COM2.raw_imu.flag_raw_imu_message == true)
                {
                    mavlink_gimbal_COM2.raw_imu.flag_raw_imu_message = false;
                    
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
                if(mavlink_gimbal_COM2.raw_imu.flag_raw_imu_message == true)
                {
                    mavlink_gimbal_COM2.raw_imu.flag_raw_imu_message = false;
                    
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
                
                if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_BMI160)
                {
                    sprintf(imu_id, "imu_name : %s |", BMI);
                }
                
                if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_ICM42688)
                {
                    sprintf(imu_id, "imu_name : %s |", ICM);
                }
                
                JIG_TEST_console_write(imu_id);
                
                if(numberOfaxis == 1)
                {
                    sprintf(imu_id, " std_dev_gyro_x_result : %1.2f | sum_std_dev_gyro_x : %1.3f | sum_gyro_x : %3d\n"
                    , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                    
                    gimbal_FSTD.vibrate.countDelta_x++;
                }
                else if(numberOfaxis == 2)
                {
                    sprintf(imu_id, " std_dev_gyro_y_result : %1.2f | sum_std_dev_gyro_y : %1.3f | sum_gyro_y : %3d\n"
                    , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                    
                    gimbal_FSTD.vibrate.countDelta_y++;
                }
                else if(numberOfaxis == 3)
                {
                    sprintf(imu_id, "std_dev_gyro_z_result : %1.2f | sum_std_dev_gyro_z : %1.3f | sum_gyro_z : %3d\n\n"
                    , std_dev_gyro_result, sum_std_dev_gyro, sum_gyro);
                    
                    gimbal_FSTD.vibrate.countDelta_z++;
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
                
                /// return result
            }
        return result;
    #else
        static int16_t gx_min = 0, gx_max = 0;
        static int16_t delta = 0;
        
        if(gx_min > x)
        {
            gx_min = x;
        }
        if(gx_max <  x)
        {
            gx_max = x;
        }
        
        if(JIG_TEST_gimbal_FSTD_get_timeOut(3000, TASK))
        {
            char buff[100];
            
            delta =  gx_max - gx_min;
            
            if(numberOfaxis == 1)
            {
                sprintf(buff, "gx_max : %5d | gx_min : %5d | delta x : %5d | raw_x : %5d | Count : %2d\n"
                , gx_max, gx_min, delta, mavlink_gimbal_COM2.raw_imu.xgyro, gimbal_FSTD.vibrate.countDelta_x);
                gimbal_FSTD.vibrate.countDelta_x++;
            }
            else if(numberOfaxis == 2)
            {
                sprintf(buff, "gy_max : %5d | gy_min : %5d | delta y : %5d | raw_y : %5d | Count : %2d\n"
                , gx_max, gx_min, delta, mavlink_gimbal_COM2.raw_imu.ygyro, gimbal_FSTD.vibrate.countDelta_y);
                gimbal_FSTD.vibrate.countDelta_y++;
            }
            else if(numberOfaxis == 3)
            {
                sprintf(buff, "gz_max : %5d | gz_min : %5d | delta z : %5d | raw_z : %5d\n\n"
                , gx_max, gx_min, delta, mavlink_gimbal_COM2.raw_imu.zgyro);
            }
            
            gx_min = x;
            gx_max = x;

            JIG_TEST_console_write(str);
            JIG_TEST_console_write(buff);
        }
        
        return delta;
    #endif
}

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_vibrate(void)
{
    char *str = "VIBRATE --->";

    #if (USE_STD_DEV_FOR_IMU == 1)
        static uint8_t state = 0;
        static bool test_loop = false;
    
        if(state == 0)
        {
            gimbal_FSTD.vibrate.std_dev_gyro_x_result = JIG_TEST_gimbal_FSTD_sensor_axis(1, mavlink_gimbal_COM2.raw_imu.xgyro, RAW_IMU_XGYRO_CACULATOR);
            
            if(gimbal_FSTD.vibrate.std_dev_gyro_x_result != 0)
            {
                if(test_loop == true)
                gimbal_FSTD.vibrate.std_dev_gyro_x_result = 0;
                
                /// next state
                state = 1;
            }
        }
        else if(state == 1)
        {
            gimbal_FSTD.vibrate.std_dev_gyro_y_result = JIG_TEST_gimbal_FSTD_sensor_axis(2, mavlink_gimbal_COM2.raw_imu.ygyro, RAW_IMU_YGYRO_CACULATOR);
            
            if( gimbal_FSTD.vibrate.std_dev_gyro_y_result != 0)
            {
                if(test_loop == true)
                gimbal_FSTD.vibrate.std_dev_gyro_y_result = 0;
                
                /// next state
                state = 2;
            }
        }
        else if(state == 2)
        {
            gimbal_FSTD.vibrate.std_dev_gyro_z_result = JIG_TEST_gimbal_FSTD_sensor_axis(3, mavlink_gimbal_COM2.raw_imu.zgyro, RAW_IMU_ZGYRO_CACULATOR);
            
            if(gimbal_FSTD.vibrate.std_dev_gyro_z_result != 0)
            {
                if(test_loop == true)
                gimbal_FSTD.vibrate.std_dev_gyro_z_result = 0;
                
                /// next state
                if(test_loop == true)
                {
                    state = 0;
                }
                else
                {
                    char buff[100];
                    
                    if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_BMI160)
                    {
                        ///setting limit delta x, y, z
                        gimbal_FSTD.vibrate.limit_gx_delta = 12.00;
                        gimbal_FSTD.vibrate.limit_gy_delta = 8.5;
                        gimbal_FSTD.vibrate.limit_gz_delta = 8.5;
                    }
                    
                    if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_ICM42688)
                    {
                        ///setting limit delta x, y, z
                        gimbal_FSTD.vibrate.limit_gx_delta = 10.00;
                        gimbal_FSTD.vibrate.limit_gy_delta = 8.5;
                        gimbal_FSTD.vibrate.limit_gz_delta = 9.00;
                    }

                    if(gimbal_FSTD.vibrate.checkDome == false)
                    {
                        gimbal_FSTD.vibrate.checkDome = true;
                        
                        if(gimbal_FSTD.vibrate.std_dev_gyro_x_result < gimbal_FSTD.vibrate.limit_gx_delta)
                        {
                            if(gimbal_FSTD.vibrate.std_dev_gyro_y_result < gimbal_FSTD.vibrate.limit_gy_delta)
                            {
                                if(gimbal_FSTD.vibrate.std_dev_gyro_z_result < gimbal_FSTD.vibrate.limit_gz_delta)
                                {
                                    gimbal_FSTD.vibrate.checkResult = true;
                                    
                                    gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_VIBRATE] = true;
                                    
                                    /// khong tinh gia tri delta nua
                                    gimbal_FSTD.vibrate.is_gimbalHome = false;
                                }
                            }
                        }
                    }
                    
                    JIG_TEST_console_write(str);
                    sprintf(buff, "result : %d | %d | limit : %.2f %.2f %.2f\n"
                    , gimbal_FSTD.vibrate.checkResult, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_VIBRATE]
                    , gimbal_FSTD.vibrate.limit_gx_delta, gimbal_FSTD.vibrate.limit_gy_delta, gimbal_FSTD.vibrate.limit_gz_delta);
                    JIG_TEST_console_write(buff);
                    
                    /// next state none
                    state = 3;
                }
                
            }
        }
    #else
        static uint32_t count_console;
        
        gimbal_FSTD.vibrate.gx_delta = JIG_TEST_gimbal_FSTD_sensor_axis(1, mavlink_gimbal_COM2.raw_imu.xgyro, RAW_IMU_XGYRO_CACULATOR);
        gimbal_FSTD.vibrate.gy_delta = JIG_TEST_gimbal_FSTD_sensor_axis(2, mavlink_gimbal_COM2.raw_imu.ygyro, RAW_IMU_YGYRO_CACULATOR);
        gimbal_FSTD.vibrate.gz_delta = JIG_TEST_gimbal_FSTD_sensor_axis(3, mavlink_gimbal_COM2.raw_imu.zgyro, RAW_IMU_ZGYRO_CACULATOR);
        
        if(gimbal_FSTD.vibrate.countDelta_x > JIG_TEST_GIMBAL_FSTD_VIBTATE_LIMIT_COUNT && gimbal_FSTD.vibrate.countDelta_y > JIG_TEST_GIMBAL_FSTD_VIBTATE_LIMIT_COUNT)
        {
            ///setting limit delta x, y, z
            gimbal_FSTD.vibrate.limit_gx_delta = 90;
            gimbal_FSTD.vibrate.limit_gy_delta = 90;
            gimbal_FSTD.vibrate.limit_gz_delta = 70;
            
            if(gimbal_FSTD.vibrate.checkDome == false)
            {
                gimbal_FSTD.vibrate.checkDome = true;
                
                if(gimbal_FSTD.vibrate.gx_delta < gimbal_FSTD.vibrate.limit_gx_delta)
                {
                    if(gimbal_FSTD.vibrate.gy_delta < gimbal_FSTD.vibrate.limit_gy_delta)
                    {
                        if(gimbal_FSTD.vibrate.gz_delta < gimbal_FSTD.vibrate.limit_gz_delta)
                        {
                            gimbal_FSTD.vibrate.checkResult = true;
                            
                            gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_VIBRATE] = true;
                            
                            /// khong tinh gia tri delta nua
                            gimbal_FSTD.vibrate.is_gimbalHome = false;
                        }
                    }
                }
                
                char buff[100];
                
                sprintf(buff, "GIMBAL RUNG result : %d | %d  <---> limit : %3d | %3d | %3d\n"\
                , gimbal_FSTD.vibrate.checkResult
                , gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_VIBRATE]
                , gimbal_FSTD.vibrate.limit_gx_delta
                , gimbal_FSTD.vibrate.limit_gy_delta
                , gimbal_FSTD.vibrate.limit_gz_delta
                );
                JIG_TEST_console_write(str);
                JIG_TEST_console_write(buff);
            }
        }
    
    #endif
    
}

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_aux_test(void)
{
    char *str = "AUX_TEST_RESULT --->";
    
    if(get_timeOut(50, JIG_TEST_GIMBAL_FSTD_TEST_AUX))
    {
        #if (GIMBAL_FSTD_JIG_TEST == 1)
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S5_GPIO_Port, AUX_S5_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[0] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S4_GPIO_Port, AUX_S4_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[1] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S3_GPIO_Port, AUX_S3_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[2] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S7_GPIO_Port, AUX_S7_Pin, GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S2_GPIO_Port, AUX_S2_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[3] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S6_GPIO_Port, AUX_S6_Pin, GPIO_PIN_SET);
        #else
        
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S7_GPIO_Port, AUX_S7_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[0] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S9_GPIO_Port, AUX_S9_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_RESET);
            if(HAL_GPIO_ReadPin(AUX_S6_GPIO_Port, AUX_S6_Pin) == false)
            {
                gimbal_FSTD.aux_test_count[1] ++;
            }
            
            HAL_GPIO_WritePin(AUX_S8_GPIO_Port, AUX_S8_Pin, GPIO_PIN_SET);
        
        #endif
    }
    
    if(get_timeOut(3000, JIG_TEST_GIMBAL_FSTD_RESULT_AUX))
    {
        uint16_t max_count_aux = 15 * 4;
        
        if(gimbal_FSTD.aux_test_count[0] > max_count_aux)
        {
            if(gimbal_FSTD.aux_test_count[1] > max_count_aux)
            {
                #if (GIMBAL_FSTD_JIG_TEST == 1)
                    if(gimbal_FSTD.aux_test_count[2] > max_count_aux)
                    {
                        if(gimbal_FSTD.aux_test_count[3] > max_count_aux)
                        {
                            char buff[100];
                            
                            gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX] = true;
                            gimbal_FSTD.aux_test_result = true;
                            JIG_TEST_console_write(str);
                            
                            sprintf(buff, "FSTD (AUX) ---> S9S5 : %d | S8S4 : %d | S7S3 : %d | S6S2 : %d | Total : %d | %d\n"\
                            , gimbal_FSTD.aux_test_count[0]
                            , gimbal_FSTD.aux_test_count[1]
                            , gimbal_FSTD.aux_test_count[2]
                            , gimbal_FSTD.aux_test_count[3]
                            , gimbal_FSTD.aux_test_result, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX]);
                            
                            JIG_TEST_console_write(buff);
                        }
                    }
                #else
                
                    char buff[100];
                    
                    gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX] = true;
                    gimbal_FSTD.aux_test_result = true;
                    JIG_TEST_console_write(str);
                    
                    sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
                    , gimbal_FSTD.aux_test_count[0]
                    , gimbal_FSTD.aux_test_count[1]
                    , gimbal_FSTD.aux_test_result, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX]);
                    
                    JIG_TEST_console_write(buff);
                #endif
            }
            else
            {
                char buff[100];
            
                JIG_TEST_console_write(str);
                
                sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
                , gimbal_FSTD.aux_test_count[0]
                , gimbal_FSTD.aux_test_count[1]
                , gimbal_FSTD.aux_test_result, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX]);
                
                JIG_TEST_console_write(buff);
            }
        }
        else
        {
            char buff[100];
        
            JIG_TEST_console_write(str);
            
            sprintf(buff, "FAC30K (AUX) ---> S9S7 : %d | S8S6 : %d | Total : %d | %d\n"\
            , gimbal_FSTD.aux_test_count[0]
            , gimbal_FSTD.aux_test_count[1]
            , gimbal_FSTD.aux_test_result, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX]);
            
            JIG_TEST_console_write(buff);
        }
    }
}

/** @brief gimbal_FSTD_all_msg_mavlink_handle
    @return none
*/
static void JIG_TEST_gimbal_FSTD_all_msg_mavlink_handle(JIG_TEST_gimbal_FSTD_private_t *FSTD)
{
    char *str = "\n FSTD_MAVLINK_SEND_HANDLE --->";
    JIG_TEST_gimbal_FSTD_mavlink_msg_send_t manager = FSTD->mavlink_msg_manager;
    static uint32_t count_set_none;
    static bool enable_console = false;
    
    if(manager == FSTD_MAVLINK_MSG_SET_NONE)
    {
        if(++ count_set_none > 100000)
        {
        
            count_set_none = 0;
            
//            JIG_TEST_console_write(str);
//            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_NONE\n");
        }
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_PARAM)
    {
        
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SEND_PARAM_REQUEST_READ)
    {
        JIG_TEST_mavlink_gimbal_send_param_request_read(FSTD->request_param_read.param_index, FSTD->request_param_read.param_id);
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SEND_PARAM_REQUEST_READ\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_CONTROL_MOTOR)
    {
        JIG_TEST_mavlink_gimbal_set_control_motor(FSTD->motor_control);
        
//        if(enable_console == true)
//        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_CONTROL_MOTOR\n");
//        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_MOVE)
    {
        JIG_TEST_mavlink_gimbal_set_move(FSTD->set_move.tilt, FSTD->set_move.roll, FSTD->set_move.pan, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_MOVE\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_MODE)
    {
        JIG_TEST_mavlink_gimbal_set_mode(FSTD->set_mode);
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_MODE\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_HOME)
    {
        JIG_TEST_mavlink_gimbal_set_home();
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_HOME\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_RC_INPUT)
    {
        JIG_TEST_mavlink_gimbal_set_rc_input(FSTD->rc_input);
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_RC_INPUT\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager == FSTD_MAVLINK_MSG_SET_REBOOT)
    {
        JIG_TEST_mavlink_gimbal_set_reboot();
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_REBOOT\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    else if(manager ==  FSTD_MAVLINK_MSG_SET_MODE_RATIO)
    {
        
        JIG_TEST_mavlink_gimbal_set_param((uint16_t)FSTD->mode_ratio, "RADIO_TYPE");
        
        if(enable_console == true)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("FSTD_MAVLINK_MSG_SET_MODE_RATIO\n");
        }
        
        /// reset send msg
        FSTD->mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_NONE;
    }
    
}

/** @brief gimbal_FSTD_all_mode_control_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_all_mode_control_process(JIG_TEST_gimbal_FSTD_private_t *FSTD)
{
    static bool enable_ppm;
    static uint8_t can_control_state;
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_SBUS] == true)
    {
        JIG_TEST_sbus_gimbal_process();
    }
    else
    {
        if(gimbal_FSTD.test_done[JIG_TEST_GIMBAL_MODE_SBUS] == true)
        JIG_TEST_sbus_gimbal_enable(false);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_PPM] == true)
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
        if(gimbal_FSTD.test_done[JIG_TEST_GIMBAL_MODE_PPM] == true)
        JIG_TEST_ppm_gimbal_enable(false);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_CAN] == true)
    {
        
            if(can_control_state == 0)
            {
                JIG_TEST_can_dji_set_move(504, 504, true);
                
                /// next can state
                can_control_state = 1;
            }

        JIG_TEST_can_dji_process();
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_COM] == true)
    {
        JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM2, STATE_SET_CTRL_GIMBAL_YAW_FOLLOW_MODE, 1, 2);
    }
    
    if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_COM4] == true)
    {
        JIG_TEST_gimbal_FSTD_control_angle(true, &mavlink_gimbal_COM4, STATE_MOVE_GIMBAL_YAW_FOLLOW_MODE_CW, 1, 4);
    }
    else if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_AUX] == true)
    {
        JIG_TEST_gimbal_FSTD_aux_test();
    }
    else if(FSTD->mode_test_process[JIG_TEST_GIMBAL_MODE_VIBRATE] == true)
    {
        JIG_TEST_gimbal_FSTD_vibrate();
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

/** @brief gimbal_FSTD_get_gimbal_ready
    @return none
*/
static bool JIG_TEST_gimbal_FSTD_get_gimbal_ready(void)
{
    bool ret = false;
    
    
    uint8_t calib_motor_state = JIG_TEST_mavlink_gimbal_get_state_calib_motor();
    uint8_t calib_imu_state = JIG_TEST_mavlink_gimbal_get_state_calib_imu();
    
    if(calib_motor_state == 2 && calib_imu_state == 2)
    {
        ret = true;
    }
    
    return ret;
}


/** @brief gimbal_FSTD_control_gimbal_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_back_to_scan_new_barCode(void)
{
    /// reset all availables cho lan test sau
    if(gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD == true)
    {
        /// khi test xong nhan nut de test gimbal khac
        if(JIG_TEST_button_state_back_to_scan_new_barCode() == true)
        {
            gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD = false;
            
            memset(&gimbal_FSTD, 0, sizeof(JIG_TEST_gimbal_FSTD_private_t));
            memset(&gimbal_FSTD_comm, 0, sizeof(JIG_TEST_gimbal_FSTD_t));
        }
        else
        {
            if(gimbal_FSTD.mode_test < JIG_TEST_GIMBAL_MODE_DONE)
            {
                //// trong qua trinh test raspberry send reset
                gimbal_FSTD_comm.reciver_reset_msg_when_run_test = true;
                
                gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD = false;
                
                memset(&gimbal_FSTD, 0, sizeof(JIG_TEST_gimbal_FSTD_private_t));
                memset(&gimbal_FSTD_comm, 0, sizeof(JIG_TEST_gimbal_FSTD_t));
            }
        }
    }
}

/** @brief gimbal_FSTD_control_gimbal_process
    @return none
*/
static void JIG_TEST_gimbal_FSTD_control_gimbal_process(void)
{
    static uint32_t count_state;
    static uint32_t count_state_com2_error;
    char *str = "\n\n              JIG_TEST_PROCESS --->";

    /// send & read Data from COM2, COM4 gimbal
//    if(gimbal_FSTD.control_lb2_dual_mode == false) 
        JIG_TEST_mavlink_gimbal_process();
    
    /// tinh tong thoi gian test
    if(gimbal_FSTD.state_FSTD != JIG_TEST_GIMBAL_FSTD_STATE_DONE)
    {
        if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_TOTAL_TIME_TEST))
        {
            gimbal_FSTD_comm.total_time ++;
        }
    }

    if(JIG_TEST_comm_raspberry_get_heartbeat_ready() == true)
    {
        /// check user login
        if(raspberry_global.user_login == JIG_TEST_CLOUD_LOGINED)
        {
            /// gimbal FSTD test process state
            if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_IDLE)
            {
                /// next state
                gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_WAIT_COM2;
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_WAIT_COM2)
            {
                if(gimbal_FSTD.is_COM2_ready == false)
                {
                    /// wait connect with gimbal COM2
                    gimbal_FSTD.is_COM2_ready = JIG_TEST_gimbal_FSTD_timeOut_connection_gimbal_COM2(&mavlink_gimbal_COM2, &gimbal_FSTD);
                }
                else if(gimbal_FSTD.is_COM2_ready == true)
                {
                    /// get state calib motor & imu
                    gimbal_FSTD.gimbal_ready = JIG_TEST_gimbal_FSTD_get_gimbal_ready();
                    
                    if(gimbal_FSTD.gimbal_ready == true)
                    {
                        /// set home gimbal
                        uint16_t command = mavlink_gimbal_COM2.ack.command;
                        
                        if(command == COMMAND_SETTING_MODE)
                        {
                            /// next state
                            gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_SETTING_PARAM;
                            
                            /// reset all led
                            HAL_GPIO_WritePin(red_GPIO_Port, red_Pin | blue_Pin | green_Pin, GPIO_PIN_SET);
                            
                            /// reset command
                            mavlink_gimbal_COM2.ack.command = 0;
                            
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_SETTING_PARAM\n------------------------------------\n");
                        }
                        else
                        {
        //                    JIG_TEST_mavlink_gimbal_set_home();
                            
                            if(gimbal_FSTD.mavlink_msg_manager == FSTD_MAVLINK_MSG_SET_NONE)
                            {
                                gimbal_FSTD.mavlink_msg_manager = FSTD_MAVLINK_MSG_SET_HOME;
                            }
                            
                            if(++count_state > 10000)
                            {
                                count_state = 0;

                                JIG_TEST_console_write(str);
                                JIG_TEST_console_write("set home gimbal\n");
                            }
                        
                        }
                    }
                    else
                    {
                        if(get_timeOut(500, JIG_TEST_GIMBAL_FSTD_GIMBAL_STARTUP_CALIB))
                        {
                            char *str_motor = "-GIMBAL_CALIB_MOTOR-";
                            char *str_imu = "-GIMBAL_CALIB_IMU-";
                            
                            JIG_TEST_console_write(str);
                            
                            if(JIG_TEST_mavlink_gimbal_get_state_calib_motor() == 1)
                            {
                                JIG_TEST_console_write(str_motor);
                            }
                            
                            if(JIG_TEST_mavlink_gimbal_get_state_calib_imu() == 1)
                            {
                                JIG_TEST_console_write(str_imu);
                            }
                            
                            /// toggle all led
                            HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin | blue_Pin | green_Pin);
                            
                            /// dem so lan dma1_stream3 khong nhan duoc data
                            gimbal_FSTD.timeOut_dma1_stream3_process ++;
                        }
                    }
                    
                }
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_SETTING_PARAM)
            {
                
                /// toogle led green 0.3s status setting param
                gimbal_FSTD.led_status.time_led_green = 300;
                if(get_timeOut(gimbal_FSTD.led_status.time_led_green, LED_GREEN_TOGGLE))
                {
                    JIG_TEST_GIMBAL_FSTD_LED_GREEN_TOGGLE;
                }
                
                /// write param to gimbal
                if(gimbal_FSTD.is_settingParam == false)
                {
                    gimbal_FSTD.is_settingParam = JIG_TEST_gimbal_FSTD_set_param();
                }
                else
                {
                    /// compare param read from gimbal
                    if(gimbal_FSTD.is_compareParam == false)
                    {
                        gimbal_FSTD.is_compareParam = JIG_TEST_gimbal_FSTD_compare_param_set(&mavlink_gimbal_COM2);
                        
        //                /// check timeOut compare param
        //                if(get_timeOut(5000, JIG_TEST_GIMBAL_FSTD_TIMEOUT_COMPARE_PARAM))
        //                {
        //                    gimbal_FSTD.is_settingParam = false;
        //                    
        //                    JIG_TEST_console_write(str);
        //                    JIG_TEST_console_write("timeOut compare param ---> retry setting param\n");
        //                }
                    }
                    else
                    {
                        HAL_Delay(100);
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_WAIT_START_FROM_RASP\n------------------------------------\n");
                        
                        /// next state
                        gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_WAIT_START_FROM_RASP;
                        
                        /// reset mavlink gimbal param index cho lan request param sau
                        mavlink_gimbal_COM2.param_value.param_index = 0;
                        
                        /// OFF led green
                        JIG_TEST_GIMBAL_FSTD_LED_GREEN_OFF;
                    }
                }
                
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_WAIT_START_FROM_RASP)
            {
                # if (GIMBAL_FSTD_AUTO_TEST == 1)
                
                    if(gimbal_FSTD_comm.scan_barCode_done == false)
                    {
                        if(raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_START] == true)
                        {
                            static bool start_test = false;
                            
                            /// reset task timeOut heartbeat
                            JIG_TEST_gimbal_FSTD_timeOut_reset(HEARTBEAT);
                            
                            /// storage count start test
                            if(start_test == false)
                            {
                                start_test = true;
                                
                                raspberry_global.count_start_test++;
                                JIG_TEST_rtc_storage_register_value(raspberry_global.count_start_test, LL_RTC_BKP_DR10);
                            }
                            
                            /// sau khi scan barCode xong reset sys de test gimbal
                            /// ghi nho da scan barcode va raspberry da send start roi
                            gimbal_FSTD_comm.scan_barCode_done = true;
                            JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.scan_barCode_done, LL_RTC_BKP_DR11);
                            
                            /// storage gimbal id
                            gimbal_FSTD_comm.display_gimbal_id_storage = mavlink_gimbal_COM2.vehicle_system_id;
                            JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.display_gimbal_id_storage, LL_RTC_BKP_DR12);
                            
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_WAIT_RESET_SYS_SECTION_MORE\n------------------------------------\n"); 
                    
                            NVIC_SystemReset();
                        }
                        else
                        {
                            static uint8_t count_toggle_led = 0;
                            
                            /// toggle led waitting start from raspberry
                            if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_WAITTING_START))
                            {
                                count_toggle_led ++;
                                
                                if(count_toggle_led % 2 == 0)
                                {
                                    HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
                                    HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_RESET);
                                }
                                else
                                {
                                    HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_RESET);
                                    HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
                                }
                                
                                /// turn off gimbal waitting scan barcode
                                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                                
                                /// write to console wait for start from raspberry
                                JIG_TEST_console_write(str);
                                JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_WAIT_FOR_START\n------------------------------------\n");
                            }
                        }
                    }
                    else
                    {
                        /// sau khi reset sys thay fflag scan barCode done thi next state
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_CONTROL\n------------------------------------\n");
                        
                        /// reset flag scan barCode done vaf storage vao register cho lan test sau
                        gimbal_FSTD_comm.scan_barCode_done = false;
                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.scan_barCode_done, LL_RTC_BKP_DR11);
                        
                        /// reset flag gimbal_FSTD_comm.reciver_reset_msg_when_run_test
                        gimbal_FSTD_comm.reciver_reset_msg_when_run_test = false;
                        /// reset flag user login 
//                        gimbal_FSTD_comm.user_logined = false;
//                        JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.user_logined, LL_RTC_BKP_DR13);
                        
                        /// next state
                        gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_CONTROL;
                    }
                    
                #else
                    /// next state
                    gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_CONTROL;
                #endif
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_CONTROL)
            {
                
                /// get mode test global
                gimbal_FSTD_comm.mode_test = gimbal_FSTD.mode_test;
                
                ///////////////////////////////////////////////////// COUNT CONTROL TEST /////////////////////////////////////////
                if(get_timeOut(1000, JIG_TEST_GIMBAL_FSTD_MODE_TEST) && gimbal_FSTD.is_test_running == true)
                {
                    gimbal_FSTD.time_run_test ++;
                    
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
                
                #if (DEBUG_ONLY == 1)
                    gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_LOOP;
                #endif
                
                ////////////////////////////////////// CHECK MODE TEST ////////////////////////////////////////////////////
                if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_IDLE)
                {
                    /// next state
                    gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_SBUS;
                    
                    /// write console
                    JIG_TEST_console_write(str);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_SBUS\n");
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_SBUS)
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD
                        , JIG_TEST_GIMBAL_MODE_SBUS, JIG_TEST_GIMBAL_MODE_READ_SBUS, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_READ_SBUS] = false;
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_PPM;
                        
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_PPM\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_PPM)
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD
                        , JIG_TEST_GIMBAL_MODE_PPM, JIG_TEST_GIMBAL_MODE_READ_PPM, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_PPM] = false;
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_CAN;
                        
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_CAN\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_CAN)
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD, JIG_TEST_GIMBAL_MODE_CAN
                        , JIG_TEST_GIMBAL_MODE_READ_CAN, 15))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_CAN] = false;
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_COM;
                        
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_COM2\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_COM)
                {
                    uint8_t gyro_fillter = 0;
                    
                    /// find IMU name
                    if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_BMI160)
                    {
                        gyro_fillter = 2;
                    }

                    if(JIG_TEST_mavlink_gimbal_get_sensor_name() == GREMSY_SENSOR_ICM42688)
                    {
                        gyro_fillter = 4;
                    }
                    
                     /// setting lai param de test vibration
                    if(gimbal_FSTD.vibrate.enable_set_param == false)
                    {
                        /// setting param
                        uint8_t Stiffness_tilt = 70;
                        uint8_t Stiffness_roll = 80;
                        uint8_t Stiffness_pan = 90;
        //                uint8_t Stiffness_tilt = 20;
        //                uint8_t Stiffness_roll = 20;
        //                uint8_t Stiffness_pan = 30;
                        
                        uint8_t hold_strength_tilt = 40;
                        uint8_t hold_strength_roll = 40;
                        uint8_t hold_strength_pan = 40;
                        
                        param_gimbal[10].value = Stiffness_pan;
                        param_gimbal[11].value = Stiffness_roll;
                        param_gimbal[12].value = Stiffness_tilt;
                        
                        param_gimbal[13].value = hold_strength_tilt;
                        param_gimbal[14].value = hold_strength_roll;
                        param_gimbal[15].value = hold_strength_pan;
                        
                        param_gimbal[20].value = gyro_fillter;
                        
                        gimbal_FSTD.vibrate.enable_set_param = JIG_TEST_gimbal_FSTD_set_param();
                        
                        /// reset flag param compare
                        gimbal_FSTD.vibrate.enable_param_compare  = false;
                    }
                    else
                    {
                        if(gimbal_FSTD.vibrate.enable_param_compare == false)
                        {
                            gimbal_FSTD.vibrate.enable_param_compare = JIG_TEST_gimbal_FSTD_compare_param_set(&mavlink_gimbal_COM2);
                        }
                        else
                        {
                            if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD, JIG_TEST_GIMBAL_MODE_COM
                                , JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                            {
                                /// reset many available for after test
                                gimbal_FSTD.is_test_running = false;
                                gimbal_FSTD.time_run_test = 0;
                                gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_COM] = false;
                                
                                // next state
                                gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_COM4;
                                
                                /// write console
                                JIG_TEST_console_write(str);
                                JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_COM4\n");
                            }
                        }
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_COM4)
                {
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD, JIG_TEST_GIMBAL_MODE_COM4
                        , JIG_TEST_GIMBAL_MODE_READ_COM, 8))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_COM4] = false;
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_AUX;
                        
        //                /// off motor gimbal next state test aux
        //                JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
        //                
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_AUX\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_AUX)
                {
                    
        //            if(++ count_state > 200000)
        //            {
        //                count_state = 0;
        //                
        //                /// kiem tra mode gimbal --->  neu gimbal dang ON thi OFF
        //                if(mavlink_gimbal_COM2.status.mode != 0)
        //                {
        //                    /// off motor gimbal in mode test aux
        //                    JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
        //                }
        //            }
                    
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD, JIG_TEST_GIMBAL_MODE_AUX
                        , JIG_TEST_GIMBAL_MODE_READ_COM, 6))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_AUX] = false;
                        
                        /// on motor gimbal next state test vibrate
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_VIBRATE;
                        
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_VIBRATE\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_VIBRATE)
                {
                    
                    if(++ count_state > 200000)
                    {
                        count_state = 0;
                        
                        /// kiem tra mode gimbal --->  neu gimbal dang OFF thi ON
                        if(mavlink_gimbal_COM2.status.mode == 0)
                        {
                            /// on motor gimbal in mode test vibration
                            JIG_TEST_mavlink_gimbal_set_control_motor(TURN_ON);
                        }
                    }
                    
                    if(JIG_TEST_gimbal_FSTD_mode_test_process(&gimbal_FSTD, JIG_TEST_GIMBAL_MODE_VIBRATE
                        , JIG_TEST_GIMBAL_MODE_READ_COM, 12))
                    {
                        /// reset many available for after test
                        gimbal_FSTD.is_test_running = false;
                        gimbal_FSTD.time_run_test = 0;
                        gimbal_FSTD.mode_test_process[JIG_TEST_GIMBAL_MODE_VIBRATE] = false;
                        
                        // next state
                        gimbal_FSTD.mode_test = JIG_TEST_GIMBAL_MODE_DONE;
                        
                        /// write console
                        JIG_TEST_console_write(str);
                        JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_DONE\n");
                    }
                }
                else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_DONE)
                {
                    char buff[100];
                    if(gimbal_FSTD.re_test == false)
                    {
                        gimbal_FSTD.re_test = true;
                        
                        JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                            
                        HAL_Delay(20);
                            
                        /// write console
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_SBUS : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_SBUS]);
                        JIG_TEST_console_write(buff);
                            
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_PPM : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_PPM]);
                        JIG_TEST_console_write(buff);
                            
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_CAN : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_CAN]);
                        JIG_TEST_console_write(buff);
                            
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_COM2 : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_COM]);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_COM4 : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_COM4]);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_AUX : %d | %d\n", gimbal_FSTD.aux_test_result, gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_AUX]);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_VIBRATE : %d\n", gimbal_FSTD.mode_test_result[JIG_TEST_GIMBAL_MODE_VIBRATE]);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        /// get result usb speed test
                        gimbal_FSTD_comm.usb_speed_result       = (uint8_t)JIG_TEST_gimbal_FAC30K_get_usb_speed_result();
                        gimbal_FSTD_comm.value_read_speed       = JIG_TEST_comm_raspberry_USB_speed_get_read_speed();
                        gimbal_FSTD_comm.value_write_speed      = JIG_TEST_comm_raspberry_USB_speed_get_write_speed();
                        gimbal_FSTD_comm.value_ref_read_speed   = JIG_TEST_comm_raspberry_USB_speed_get_ref_read_speed();
                        gimbal_FSTD_comm.value_ref_write_speed  = JIG_TEST_comm_raspberry_USB_speed_get_ref_write_speed();
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "JIG_TEST_GIMBAL_MODE_USB: %d\n", gimbal_FSTD_comm.usb_speed_result);//raspberry_global.value_speed_test[2]);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        JIG_TEST_console_write(str);
                        sprintf(buff, "Total time: %5d \n Total_timeOut_task : %3d\n", gimbal_FSTD_comm.total_time, (uint8_t)TIME_OUT_TOTAL_TASK);
                        JIG_TEST_console_write(buff);
                        
                        HAL_Delay(20);
                        
                        /// reset mavlink gimbal param index cho lan request param sau
                        mavlink_gimbal_COM2.param_value.param_index = 0;
                        
                        gimbal_FSTD.state_FSTD = JIG_TEST_GIMBAL_FSTD_STATE_DONE;
                        
                        /// sau khi hien thi ket qua thi kiem tra motor off chua
                        if(mavlink_gimbal_COM2.status.mode != 0)
                        {
                            JIG_TEST_mavlink_gimbal_set_control_motor(TURN_OFF);
                        }
                        
                    }
                }
                
                
                
                #if (DEBUG_ONLY == 1)
                    else if(gimbal_FSTD.mode_test == JIG_TEST_GIMBAL_MODE_LOOP)
                    {

                    }
                #endif
                /// run all mode test
                JIG_TEST_gimbal_FSTD_all_mode_control_process(&gimbal_FSTD);
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_ERROR)
            {
                char buff[100];
                
                /// toggle led red 1s state ERROR
                gimbal_FSTD.led_status.time_led_red = 1000;
                if(get_timeOut(gimbal_FSTD.led_status.time_led_red, LED_RED_TOGGLE))
                {
                    JIG_TEST_GIMBAL_FSTD_LED_RED_TOGGLE;
                    
                    if(++count_state_com2_error > 2)
                    {
                        count_state_com2_error = 0;
                        
                        /// next state DONE for gimbal_FSTD_comm use display Error
                        gimbal_FSTD_comm.mode_test = JIG_TEST_GIMBAL_MODE_ERROR;
                    }
                    
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    sprintf(buff, "ERROR gimbal COM2 connection !!! : %d\n", gimbal_FSTD_comm.mode_test);
                    JIG_TEST_console_write(buff);
                }
                
                /// reset all availables cho lan test sau
                if(gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD == true)
                {
                    if(JIG_TEST_button_state_back_to_scan_new_barCode() == true)
                    {
                        gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD = false;
                        
                        memset(&gimbal_FSTD, 0, sizeof(JIG_TEST_gimbal_FSTD_private_t));
                        memset(&gimbal_FSTD_comm, 0, sizeof(JIG_TEST_gimbal_FSTD_t));
                    }
                    else
                    {
                        if(++ count_state > 100000)
                        {
                            count_state = 0;
                            
                            /// write console state jig test
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("JIG_TEST_STATE_ERROR\nPRESS BUTTON 2 TIME ---> BACK TO SCAN NEW BARCODE\n");
                    
                            HAL_GPIO_TogglePin(green_GPIO_Port, green_Pin);
                        }
                    }
                }
            }
            else if(gimbal_FSTD.state_FSTD == JIG_TEST_GIMBAL_FSTD_STATE_DONE)
            {
                static bool first;
                static bool final_get_param;
                static bool enable_console_printf_done_mode_test;// = false;
                
                char buff[100];
                
                if(first == false)
                {
                    first = true;
                    
                    JIG_TEST_console_write(str);
                    JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_DONE !!!\n");
                    
                    /// get result to global
                    for(uint8_t i = 0; i < ((uint8_t)JIG_TEST_GIMBAL_MODE_TOTAL); i++)
                    {
                        gimbal_FSTD_comm.mode_test_result[i] = gimbal_FSTD.mode_test_result[i];
                    }
                }
                else
                {
                    if(enable_console_printf_done_mode_test == false)
                    {
                        enable_console_printf_done_mode_test = true;
                        
                        if(++ count_state > 800000)
                        {
                            count_state = 0;
                            
                            JIG_TEST_console_write(str);
                            JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_STATE_DONE !!!\n");
                        }
                    }
                    else
                    {

                    }
                    
                    /// request param final, use send raspberry
                    if(final_get_param == false)
                    {
                        final_get_param = JIG_TEST_gimbal_FSTD_request_param_gimbal(&mavlink_gimbal_COM2);
                        
                        /// copy param value de send cho raspberry
                        if(final_get_param == true)
                        {
                            for(uint8_t i = 0; i < JIG_TEST_COMM_RASPBERRY_MAX_PARAM_INDEX; i++)
                            {
                                raspberry_global.param_value[i] = gimbal_user_profile_ship[i + 36].value_param_get;
                                
                                JIG_TEST_console_write(str);
                                
                                sprintf(buff, "raspberry param value : %d | id : %s | %.f | %.f"
                                , i
                                , gimbal_user_profile_ship[i + 36].param_id
                                , raspberry_global.param_value[i]
                                , gimbal_user_profile_ship[i + 36].value_param_get);
                                JIG_TEST_console_write(buff);
                                HAL_Delay(20);
                            }
                            
                            /// ser flag read param done
                            raspberry_global.read_param_done = true;
                        }
                    }
                    else
                    {
                        if(++ count_state > 100000)
                        {
                            char buff[300];
                            
                            count_state = 0;
                            
                            /// write console state jig test
                            JIG_TEST_console_write(str);
                            sprintf(buff, "JIG_TEST_STATE_DONE\nPRESS BUTTON 1 TIME ---> RUN FEED BACK IMU\nPRESS BUTTON 2 TIME ---> BACK TO SCAN NEW BARCODE | reset all avalable : %d\n "
                            , gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD);
                            JIG_TEST_console_write(buff);
                    
                            HAL_GPIO_TogglePin(green_GPIO_Port, green_Pin);
                        }
                    }
                    
                    /// move gimbal test feed back imu
                    JIG_TEST_gimbal_FSTD_run_feed_back_imu();
                    
                }
            }
        }
    }
    
    /// reset all availables befor back to scan new barcode
    JIG_TEST_gimbal_FSTD_back_to_scan_new_barCode();
    
    //// handle all mavlink message
    JIG_TEST_gimbal_FSTD_all_msg_mavlink_handle(&gimbal_FSTD);
}

/** @brief gimbal_FSTD_control_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_control_process(void)
{
    char *str = "\n\n              JIG_TEST_PROCESS --->";
    
    /// reset time process
    calculator_reset_time(&gimbal_FSTD.timeOut._time_process);
    
    /// main jig test process
    JIG_TEST_gimbal_FSTD_control_gimbal_process();
    
    #if (GIMBAL_FSTD_JIG_TEST == 0)
    
        JIG_TEST_comm_raspberry_process();
        #if (GIMBAL_FSTD_AUTO_TEST == 1) 
            JIG_TEST_comm_raspberry_cloud_data_process();
        #endif
    #else
        #if (GIMBAL_FSTD_AUTO_TEST == 1) 
            JIG_TEST_comm_raspberry_cloud_data_process();
        #endif
    #endif
    
    /// waitting for init after reset
    if(gimbal_FSTD_comm.wait_init_after_reset == false)
    {
        if(JIG_TEST_gimbal_FSTD_get_timeOut(2000, JIG_TEST_GIMBAL_FSTD_WAIT_INIT_AFTER_RESET))
        {
            gimbal_FSTD_comm.wait_init_after_reset = true;
            
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("JIG_TEST_GIMBAL_FSTD_WAIT_INIT_AFTER_RESET\n");
        }
    }

    if(JIG_TEST_gimbal_FSTD_get_timeOut(500, DISPLAY_PROCESS))
    {
        JIG_TEST_display_process();
    }

    /// button prcess
    JIG_TEST_button_process();
    
    /// calculation time process
    gimbal_FSTD.timeOut.time_process = calculator_get_time_us(&gimbal_FSTD.timeOut._time_process);
}

/** @brief gimbal_FSTD_display_process
    @return none
*/
void JIG_TEST_gimbal_FSTD_display_process(void)
{
    
}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


