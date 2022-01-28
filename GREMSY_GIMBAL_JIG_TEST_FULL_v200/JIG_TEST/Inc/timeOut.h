/**
******************************************************************************
* @file timeOut.h
* @author 
* @version 
* @date 
* @brief This file contains all the functions prototypes
for the Gremsy
* common firmware library.
*
************************************************************
******************
* @par
* COPYRIGHT NOTICE: (c) 2016 Gremsy. All rights reserved.
*
* The information contained herein is confidential
* property of Company. The use, copying, transfer or
* disclosure of such information is prohibited except
* by express written agreement with Company.
*
************************************************************
******************
*/
/* Define to prevent recursive inclusion
------------------------------------------------------------------------------*/
#ifndef _TIMEOUT_H_
#define _TIMEOUT_H_
#ifdef __cplusplus
extern "C" {
#endif
/* Includes------------------------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "stdbool.h"
#include "JIG_TEST_config.h"
/* Exported types------------------------------------------------------------------------------*/
#if (USE_LIBRARY_V1 == 1)
    typedef enum
    {
        LED_BLUE_TOGGLE,
        LED_GREEN_TOGGLE,
        LED_RED_TOGGLE,
        CAN_DJI_TX_ERROR,
        CAN_DJI_RX_ERROR,
        CAN_DJI_GIMBAL_MESSAGE_TRANMITS,
        CAN_DJI_GIMBAL_MESSAGE_REMOTE,
        SBUS_WRITE_PACKET,
        SBUS_CONTROL_TYPE_LOOP_CW,
        SBUS_CONTROL_TYPE_LOOP_CCW,
        PPM_CONTROL_TYPE_LOOP_CW,
        PPM_CONTROL_TYPE_LOOP_CCW,
        DISPLAY_ERROR_DEVICE_READY,
        DISPLAY_UPDATE_SCREEN,
        JIG_TEST_TIMEOUT_GIMBAL_MODE,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_SBUS,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_PPM,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_CAN,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_CONTROL_COM2,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_CONTROL_COM4,
        JIG_TEST_TIMEOUT_GIMBAL_MODE_AUX,
        JIG_TEST_TIMEOUT_DISPLAY_MODE_ALL,
        JIG_TEST_HEARTBEAT_TIMEOUT,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM2,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM4,
        JIG_TEST_GIMBAL_FSTD_TIMEOUT_HEARTBEAT,
        JIG_TEST_GIMBAL_FSTD_TIMEOUT_ANGLE,
        JIG_TEST_GIMBAL_FSTD_MOTOR_CONTROL,
        JIG_TEST_GIMBAL_FSTD_PRINTF_ACK,
        JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL,
        JIG_TEST_GIMBAL_FSTD_MODE_TEST,
        JIG_TEST_COMM_RASPBERRY_SEND_HB,
        JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_STATE,
        JIG_TEST_GIMBAL_FSTD_RETURN_HOME,
        JIG_TEST_GIMBAL_FSTD_SET_MODE_RATIO,
        JIG_TEST_GIMBAL_FSTD_REQUEST_MODE_RATIO,
        JIG_TEST_GIMBAL_FSTD_PRINTF_MODE_TEST,
        JIG_TEST_GIMBAL_FSTD_TEST_AUX,
        JIG_TEST_GIMBAL_FSTD_RESULT_AUX,
        JIG_TEST_GIMBAL_FSTD_GIMBAL_STARTUP_CALIB,
        JIG_TEST_GIMBAL_FSTD_IMU_ANGLE,
        JIG_TEST_GIMBAL_FSTD_WAITTING_CALIB_STATUS,
        JIG_TEST_GIMBAL_FSTD_TOTAL_TIME_TEST,
        JIG_TEST_GIMBAL_FSTD_TIMEOUT_COMPARE_PARAM,
        JIG_TEST_GIMBAL_FSTD_WAIT_FOR_DEBUG,
        JIG_TEST_GIMBAL_FSTD_WAITTING_START,
        JGI_TEST_COMM_RASPBERRY_SEND_ACCEPT_RST,
        JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT,
        JIG_TEST_BUTTON_SELECT_2_MODE,
        JGI_TEST_DISPLAY_LOGINED,
        
        TIME_OUT_TOTAL_TASK,
        
    }timeOut_task_t;
#endif

#if (USE_LIBRARY_V2 == 1)
    typedef enum
    {
        LED_RED_TOGGLE,
        LED_BLUE_TOGGLE,
        LED_GREEN_TOGGLE,
        SBUS_WRITE_PACKET,
        SBUS_CONTROL_TYPE_LOOP_CW,
        SBUS_CONTROL_TYPE_LOOP_CCW,
        PPM_CONTROL_TYPE_LOOP_CW,
        PPM_CONTROL_TYPE_LOOP_CCW,
        CAN_DJI_TX_ERROR,
        CAN_DJI_RX_ERROR,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM2,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM4,
        JIG_TEST_BUTTON_SELECT_2_MODE,
        JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_STATE,
        JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_RESULT,
        JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT,
        JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP,
        DISPLAY_ERROR_DEVICE_READY,
        JIG_TEST_GIMBAL_FSTD_TIME_OUT_HEARTBEAT_COM2,
        JIG_TEST_DISPLAY_PROCESS,
        JIG_TEST_GIMBAL_FSTD_PRINTF_MODE_TEST,
        JIG_TEST_GIMBAL_FSTD_MODE_TEST,
        JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL,
        JIG_TEST_GIMBAL_FSTD_TEST_AUX,
        JIG_TEST_GIMBAL_FSTD_RESULT_AUX,
        JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR,
        JIG_TEST_AUX_GPIO_RESULT_OK,
        JIG_TEST_USB_SPEED_SEND_END,
        JIG_TEST_TIMEOUT_SETTING_PARAM_GIMBAL,
        JIG_TEST_USB_SPEED_SEND_START,
        JIG_TEST_SEND_JIG_RESETTING,
        JIG_TEST_SEND_REQUEST_PARAM_RADIO_TYPE,
        JIG_TEST_DEBUG_STATE_AND_MODE,
        JIG_TEST_TIMEOUT_MODE_DONE,
        JIG_TEST_TIMEOUT_PROFILE_SHIP,
        
        TIME_OUT_TOTAL_TASK,
    }timeOut_task_t;
#endif

#if (COMM_ESP32 == 1)
    typedef enum
    {
        LED_RED_TOGGLE,
        LED_BLUE_TOGGLE,
        LED_GREEN_TOGGLE,
        SBUS_WRITE_PACKET,
        SBUS_CONTROL_TYPE_LOOP_CW,
        SBUS_CONTROL_TYPE_LOOP_CCW,
        PPM_CONTROL_TYPE_LOOP_CW,
        PPM_CONTROL_TYPE_LOOP_CCW,
        CAN_DJI_TX_ERROR,
        CAN_DJI_RX_ERROR,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM2,
        JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM4,
        JIG_TEST_BUTTON_SELECT_2_MODE,
        JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_STATE,
        JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_RESULT,
        JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT,
        JIG_TEST_GIMBAL_FSTD_CONTROL_LOOP,
        DISPLAY_ERROR_DEVICE_READY,
        JIG_TEST_GIMBAL_FSTD_TIME_OUT_HEARTBEAT_COM2,
        JIG_TEST_DISPLAY_PROCESS,
        JIG_TEST_GIMBAL_FSTD_PRINTF_MODE_TEST,
        JIG_TEST_GIMBAL_FSTD_MODE_TEST,
        JIG_TEST_GIMBAL_FSTD_ANGLE_CONTROL,
        JIG_TEST_GIMBAL_FSTD_TEST_AUX,
        JIG_TEST_GIMBAL_FSTD_RESULT_AUX,
        JIG_TEST_GIMBAL_FSTD_COM2_HEARTBEAT_ERROR,
        JIG_TEST_AUX_GPIO_RESULT_OK,
        JIG_TEST_USB_SPEED_SEND_END,
        JIG_TEST_TIMEOUT_SETTING_PARAM_GIMBAL,
        JIG_TEST_USB_SPEED_SEND_START,
        JIG_TEST_SEND_JIG_RESETTING,
        JIG_TEST_SEND_REQUEST_PARAM_RADIO_TYPE,
        JIG_TEST_DEBUG_STATE_AND_MODE,
        JIG_TEST_TIMEOUT_MODE_DONE,
        JIG_TEST_TIMEOUT_PROFILE_SHIP,
        
        TIME_OUT_TOTAL_TASK,
    }timeOut_task_t;
#endif
/* Exported constants------------------------------------------------------------------------------*/
/* Exported macro------------------------------------------------------------------------------*/
/* Exported functions------------------------------------------------------------------------------*/
bool get_timeOut(uint32_t time, timeOut_task_t task);
void reset_timeOut(timeOut_task_t task);

/** @brief calculator_get_time_us
    @return time_us
*/
uint32_t calculator_get_time_us(uint32_t *time);

/** @brief calculator_get_time_ms
    @return time_ms
*/
uint32_t calculator_get_time_ms(uint32_t *time);

/** @brief calculator_reset_time
    @return time_ms
*/
void calculator_reset_time(uint32_t *time);

/** @brief timeOut_configuration
    @return none
*/
void timeOut_configuration(void);
#ifdef __cplusplus
}
#endif
#endif /* _TIMEOUT_H_ */
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


