/**
  ******************************************************************************
  * @file JIG_TEST_comm_raspberry.c
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
#include "JIG_TEST_comm_raspberry.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_gimbal_FSTD.h"
#include "timeOut.h"
#include "main.h"
/* Private typedef------------------------------------------------------------------------------*/

typedef enum _JIG_TEST_comm_raspberry_USB_speed_test_state_t
{
    JIG_TEST_USB_SPEED_STATE_IDLE,
    JIG_TEST_USB_SPEED_STATE_STANDBY,
    JIG_TEST_USB_SPEED_STATE_RUNNING,
    JIG_TEST_USB_SPEED_STATE_DONE,
    
}JIG_TEST_comm_raspberry_USB_speed_test_state_t;

typedef enum _JIG_TEST_comm_raspberry_USB_speed_test_read_state_t
{
    JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_RESULT,
    JIG_TEST_USB_SPEED_READ_STATE_STATUS,
    JIG_TEST_USB_SPEED_READ_STATE_START,
    JIG_TEST_USB_SPEED_READ_STATE_END,
    JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED,
    JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED,

    
}JIG_TEST_comm_raspberry_USB_speed_test_read_state_t;

typedef enum _JIG_TEST_comm_raspberry_cloud_get_status_t
{
    JIG_TEST_CLOUD_DATA_GET_STATUS_NONE,
    JIG_TEST_CLOUD_DATA_GET_STATUS_STANDBY,
    JIG_TEST_CLOUD_DATA_GET_STATUS_RUNNING,
    JIG_TEST_CLOUD_DATA_GET_STATUS_TEST_DONE,
    
}JIG_TEST_comm_raspberry_cloud_get_status_t;

typedef struct _JIG_TEST_comm_raspberry_USB_speed_test_t
{
    JIG_TEST_comm_raspberry_USB_speed_test_state_t state;
    
    float value_speed_test[8];
    JIG_TEST_comm_raspberry_USB_speed_test_result_t result;
    
    bool is_sendRequest;
    bool is_DoneTest;
    bool is_send_runTest;
    bool is_send_End;
    bool id_send_status;
    bool disable_send_state;
    bool enable_write_console_usb_speed_result;
    
}JIG_TEST_comm_raspberry_USB_speed_test_t;

typedef struct _JIG_TEST_comm_raspberry_private_t
{
    JIG_TEST_comm_raspberry_USB_speed_test_t usb_speed;

}JIG_TEST_comm_raspberry_private_t;

/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

JIG_TEST_comm_raspberry_private_t comm_raspberry;

//static const char JIG_TEST_comm_raspberry_usb_speed_id[8][16] = 
//{
//  "READSPEED",
//  "WRITESPEED",
//  "RESULT",
//  "STATUS",
//  "START",
//  "END",
//  "REF_READ",
//  "REF_WRITE"
//};

char *str_usb_speed = "\nUSB_SPEED_TEST --->";

extern JIG_TEST_mavlink_gimbal_t            mavlink_comm_rapberry;
extern JIG_TEST_mavlink_comm_respberry_t    raspberry_msg;
extern JIG_TEST_gimbal_FSTD_t               gimbal_FSTD_comm;


JIG_TEST_comm_raspberry_global_t            raspberry_global;

/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
/** @group JIG_TEST_COMM_RASPBERRY_CONFIGURATION
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_CONFIGURATION
#define JIG_TEST_COMM_RASPBERRY_CONFIGURATION

/** @brief comm_raspberry_configuration
    @return none
*/
void JIG_TEST_comm_raspberry_configuration(void)
{
    comm_raspberry.usb_speed.id_send_status = true;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_COMMUNICATION
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_COMMUNICATION
#define JIG_TEST_COMM_RASPBERRY_COMMUNICATION

/** @brief comm_raspberry_USB_speed_test_get_state
    @return JIG_TEST_comm_raspberry_USB_speed_test_state_t 
*/
static JIG_TEST_comm_raspberry_USB_speed_test_state_t JIG_TEST_comm_raspberry_USB_speed_test_get_state(JIG_TEST_mavlink_gimbal_t *mavlink_channel)
{
    JIG_TEST_comm_raspberry_USB_speed_test_state_t state = JIG_TEST_USB_SPEED_STATE_IDLE;
    
    raspberry_msg.enable_send_request_state = true;//JGI_TEST_mavlink_comm_raspberry_send_request_param_read(3, "STATUS");//(int16_t)JIG_TEST_USB_SPEED_READ_STATE_STATUS, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_STATUS]);
    
    if(mavlink_channel->param_value.param_index == 3)
    {
        if(mavlink_channel->param_value.param_value == 0)
        {
            state = JIG_TEST_USB_SPEED_STATE_STANDBY;
        }
        else if(mavlink_channel->param_value.param_value == 1)
        {
            state = JIG_TEST_USB_SPEED_STATE_RUNNING;
        }
        else if(mavlink_channel->param_value.param_value == 2)
        {
            state = JIG_TEST_USB_SPEED_STATE_DONE;
        }
    }
    
    return state;
}

/** @brief comm_raspberry_USB_speed_test_get_result
    @return JIG_TEST_comm_raspberry_USB_speed_test_result_t
*/

/*
static JIG_TEST_comm_raspberry_USB_speed_test_result_t JIG_TEST_comm_raspberry_USB_speed_test_get_result(JIG_TEST_mavlink_gimbal_t *mavlink_channel)
{
    JIG_TEST_comm_raspberry_USB_speed_test_result_t result;
    
    if(mavlink_channel->param_value.param_index == 2)
    {
        if(mavlink_channel->param_value.param_value == 0)
        {
            result = JIG_TEST_USB_SPEED_RESULT_NO_USB_FOUND;
        }
        else if(mavlink_channel->param_value.param_value == 1)
        {
            result = JIG_TEST_USB_SPEED_RESULT_LOW_SPEED;
        }
        else if(mavlink_channel->param_value.param_value == 2)
        {
            result = JIG_TEST_USB_SPEED_RESULT_PASSED;
        }
    }
    
    return result;
}
*/

/** @brief comm_raspberry_USB_speed_test_send_request_SPEED_TEST
    @return none
*/

static void JIG_TEST_comm_raspberry_USB_speed_test_send_request_SPEED_TEST(JIG_TEST_comm_raspberry_private_t *comm_raspberry)
{
    static uint8_t request_read_state = 0;
    
//    if(read_state == JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED)
    if(request_read_state == 0)
    {
        if(comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] == 0.00f)
        {
            raspberry_msg.enable_send_request_read_speed = true;
        }
        else
        {
            raspberry_msg.enable_send_request_read_speed = false;
            
            /// next state
            request_read_state = 1;//read_state = JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED;
        }
        //JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]);
        
    }
    else if(request_read_state == 1)//if(read_state == JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED)
    {
        if(comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] == 0.00f)
        {
            raspberry_msg.enable_send_request_write_speed = true;
        }
        else
        {
            raspberry_msg.enable_send_request_write_speed = false;
            
            /// next state
            request_read_state = 2;//read_state = JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED;
        }
        //JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]);
    }
    else if(request_read_state == 2)//if(read_state == JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED)
    {
        if(comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] == 0.00f)
        {
            raspberry_msg.enable_send_request_ref_read_speed = true;
        }
        else
        {
            raspberry_msg.enable_send_request_ref_read_speed = false;
            
            /// next state
            request_read_state = 3;//read_state = JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED;
        }
        //JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]);
    }
    else if(request_read_state == 3)//if(read_state == JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED)
    {
        if(comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] == 0.00f)
        {
            raspberry_msg.enable_send_request_ref_write_speed = true;
        }
        else
        {
            raspberry_msg.enable_send_request_ref_write_speed = false;
            
            /// next state
            request_read_state = 4;//read_state = JIG_TEST_USB_SPEED_READ_STATE_RESULT;
        }
        //JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]);
        
    }   
    else if(request_read_state == 4)//if(read_state == JIG_TEST_USB_SPEED_READ_STATE_RESULT)
    {
        if(comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_NONE)
        {
            raspberry_msg.enable_send_request_result = true;
        }
        else
        {
            raspberry_msg.enable_send_request_result = false;
            
            request_read_state = 5;
        }
        //JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_RESULT, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_RESULT]);
    }
    else if(request_read_state == 5)//if(read_state == JIG_TEST_USB_SPEED_READ_STATE_STATUS)
    {
        raspberry_msg.enable_send_request_result = false;
    }
}

/** @brief comm_raspberry_USB_speed_send_start
    @return none
*/
static void JIG_TEST_comm_raspberry_USB_speed_send_start(void)
{
    raspberry_msg.enable_send_start = true;//JGI_TEST_mavlink_comm_raspberry_send_request_param_read((int16_t)JIG_TEST_USB_SPEED_READ_STATE_START, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_START]);
}

/** @brief comm_raspberry_USB_speed_send_end
    @return none
*/
static void JIG_TEST_comm_raspberry_USB_speed_send_end(void)
{
    raspberry_msg.enable_send_end = true;//JGI_TEST_mavlink_comm_raspberry_send_request_param_read(5, "ENDTEST");//(int16_t)JIG_TEST_USB_SPEED_READ_STATE_END, (char *)JIG_TEST_comm_raspberry_usb_speed_id[JIG_TEST_USB_SPEED_READ_STATE_END]);
}

/** @brief comm_raspberry_USB_speed_send_end
    @return none
*/
static bool JIG_TEST_comm_raspberry_USB_speed_check_speed_value(JIG_TEST_comm_raspberry_private_t *comm_raspberry)
{
    bool ret = false;
    
    if( comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] != 0.00f
        && comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] != 0.00f
        && comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] != 0.00f
        && comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] != 0.00f
        && comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] != 0.00f
    )
    {
        JIG_TEST_comm_raspberry_USB_speed_send_end();
        
        /// disable send request
        comm_raspberry->usb_speed.is_sendRequest = false;
        
        /// enable send status
        raspberry_msg.enable_send_request_state = true;
        
        /// enable get status
        comm_raspberry->usb_speed.id_send_status = true;
        
        /// reset state
        comm_raspberry->usb_speed.state = JIG_TEST_USB_SPEED_STATE_IDLE;
        
        /// reset param index
        mavlink_comm_rapberry.param_value.param_index = 0;
        
        /// reset param value
        mavlink_comm_rapberry.param_value.param_value = 0;
        
        /// enable_write_console_usb_speed_result
        comm_raspberry->usb_speed.enable_write_console_usb_speed_result = true;
        
        ///
        raspberry_global.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] = comm_raspberry->usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT];
        
        ret = true;
    }
    else
    {
        JIG_TEST_comm_raspberry_USB_speed_test_send_request_SPEED_TEST(comm_raspberry);
    }
    
    return ret;
}

/** @brief comm_raspberry_USB_speed_get_read_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_read_speed(void)
{
    return comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED];
}

/** @brief comm_raspberry_USB_speed_get_write_speed
    @return result : write_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_write_speed(void)
{
    return comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED];
}

/** @brief comm_raspberry_USB_speed_get_ref_read_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_ref_read_speed(void)
{
    return comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED];
}

/** @brief comm_raspberry_USB_speed_get_ref_write_speed
    @return result : read_speed
*/
float JIG_TEST_comm_raspberry_USB_speed_get_ref_write_speed(void)
{
    return comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED];
}

#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_SEND_TO_CLOUD
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_SEND_TO_CLOUD
#define JIG_TEST_COMM_RASPBERRY_SEND_TO_CLOUD

/** @brief comm_raspberry_cloud_data_get_status
    @return JIG_TEST_comm_raspberry_cloud_send_status_t
*/
static JIG_TEST_comm_raspberry_cloud_send_status_t JIG_TEST_comm_raspberry_get_status_from_cloud_data(void)
{
    JIG_TEST_comm_raspberry_cloud_send_status_t status = JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.custom_mode == 0)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE;
        
        /// set flag start stop system for display 
        gimbal_FSTD_comm.start_stopSystem = 0;
    }
    else if(mavlink_comm_rapberry.heartbeat.custom_mode == 1)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_STATUS_START;
        
        /// set flag start stop system for display 
        gimbal_FSTD_comm.start_stopSystem = 1;
    }
    else if(mavlink_comm_rapberry.heartbeat.custom_mode == 2)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP;
        
        /// set flag start stop system for display 
        gimbal_FSTD_comm.start_stopSystem = 2;
    }
    else if(mavlink_comm_rapberry.heartbeat.custom_mode == 3)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_STATUS_RESET_SYSTEM;
    }
    
    
    return status;
}

/** @brief comm_raspberry_cloud_login
    @return JIG_TEST_comm_raspberry_cloud_login_t
*/
static JIG_TEST_comm_raspberry_cloud_login_t JIG_TEST_comm_raspberry_cloud_login(void)
{
    JIG_TEST_comm_raspberry_cloud_login_t login = JIG_TEST_CLOUD_LOGIN_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.base_mode == 1)
    {
        login = JIG_TEST_CLOUD_WAIT_FOR_LOGIN;
    }
    else if(mavlink_comm_rapberry.heartbeat.base_mode == 2)
    {
        login = JIG_TEST_CLOUD_LOGINED;
    }
    
    return login;
}

/** @brief comm_raspberry_get_result_push_data
    @return JIG_TEST_comm_raspberry_push_cloud_result
*/
static JIG_TEST_comm_raspberry_push_cloud_result JIG_TEST_comm_raspberry_get_result_push_data(void)
{
    JIG_TEST_comm_raspberry_push_cloud_result result = RASPBERRY_PUSH_CLOUD_RESULT_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.system_status == 0)
    {
        result = RASPBERRY_PUSH_CLOUD_RESULT_NONE;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 1)
    {
        result = RASPBERRY_PUSH_CLOUD_RESULT_OK;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 2)
    {
        result = RASPBERRY_PUSH_CLOUD_RESULT_FAIL;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 3)
    {
        result = RASPBERRY_PUSH_CLOUD_BARCODE_READY;
    }
    
    return result;
}

/** @brief comm_raspberry_cloud_data_send_value_jig_test
    @return none
*/
static bool JIG_TEST_comm_raspberry_cloud_data_send_value_jig_test(void)
{
    bool ret = false;
    
    if(mavlink_comm_rapberry.param_request_read.param_index == 1)
    {
        int16_t value = (int16_t)raspberry_global.param_value[1];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 2)
    {
        int16_t value = (int16_t)raspberry_global.param_value[2];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 3)
    {
        int16_t value = (int16_t)raspberry_global.param_value[3];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 4)
    {
        int16_t value = (int16_t)raspberry_global.param_value[4];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 5)
    {
        int16_t value = (int16_t)raspberry_global.param_value[5];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else     if(mavlink_comm_rapberry.param_request_read.param_index == 6)
    {
        int16_t value = (int16_t)raspberry_global.param_value[6];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
//    else if(mavlink_comm_rapberry.param_request_read.param_index == 7)
//    {
//        /// setting value msg send to raspberry
//        raspberry_msg.enable.send_param_value = true;
//        raspberry_msg.enable.param_value = 32121;
//    }
//    else if(mavlink_comm_rapberry.param_request_read.param_index == 8)
//    {
//        /// setting value msg send to raspberry
//        raspberry_msg.enable.send_param_value = true;
//        raspberry_msg.enable.param_value = 45667;
//    }
//    else if(mavlink_comm_rapberry.param_request_read.param_index == 9)
//    {
//        /// setting value msg send to raspberry
//        raspberry_msg.enable.send_param_value = true;
//        raspberry_msg.enable.param_value = -654;
//    }
//    else if(mavlink_comm_rapberry.param_request_read.param_index == 10)
//    {
//        /// setting value msg send to raspberry
//        raspberry_msg.enable.send_param_value = true;
//        raspberry_msg.enable.param_value = -789;
//    }
    
    return ret;
}

/** @brief JIG_TEST_comm_raspberry_send_status_to_cloud_data
    @return none
*/
static void JIG_TEST_comm_raspberry_send_status_to_cloud_data(void)
{
    char *cloud_data = "\nCLOUD_DATA --->";
    char buff[150];
    static uint32_t count_console = 0;
    static uint32_t count_console_result = 0;
    JIG_TEST_comm_raspberry_cloud_send_status_t status = JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE;
    JIG_TEST_comm_raspberry_push_cloud_result result = RASPBERRY_PUSH_CLOUD_RESULT_NONE;
    
    status = JIG_TEST_comm_raspberry_get_status_from_cloud_data();
    
    /// reciever status from raspberry
    if(status == JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE)
    {

        /// jig test standby
        mavlink_comm_rapberry.heartbeat.system_status_send = 1;
        

        /// set flag status for jig test
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE] = true;
        
        /// reset cac co con lai
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP] = false;
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_START] = false;
        
        /// write to console
        if(++count_console > 150000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE : %d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE],  mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(status == JIG_TEST_CLOUD_DATA_SEND_STATUS_START)
    {
//        /// sau khi test xong nhan duoc STOP va nhan reSTART reset sys
//        if(raspberry_msg.enable.stop == true)
//        {
//            NVIC_SystemReset();
//        }
        
        /// reset barcode ready for display
        gimbal_FSTD_comm.barcode_ready = false;
//        gimbal_FSTD_comm.display_barcode_done = true;
        
        /// kiem tra trang thai test send status cho Pi4
        if(gimbal_FSTD_comm.mode_test == JIG_TEST_GIMBAL_MODE_DONE)
        {
            if(raspberry_global.read_param_done == true)
            {
                mavlink_comm_rapberry.heartbeat.system_status_send = 3;
                
                /// send param value with param request read from raspberry
                JIG_TEST_comm_raspberry_cloud_data_send_value_jig_test();
            }
        }
        else if(gimbal_FSTD_comm.mode_test == JIG_TEST_GIMBAL_MODE_ERROR)
        {
            mavlink_comm_rapberry.heartbeat.system_status_send = 4;
        }
        else 
        {
            mavlink_comm_rapberry.heartbeat.system_status_send = 2;
            
            /// reset param request read index
            mavlink_comm_rapberry.param_request_read.param_index = 0;
        }

        /// set flag status for jig test
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_START] = true;
        
        /// reset cac co con lai
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP] = false;
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE] = false;
        
        /// write to console
        if(++count_console > 150000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_STATUS_START : %d | mode_test : %3d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_START]
            , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(status == JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP)
    {
        mavlink_comm_rapberry.heartbeat.system_status_send = 1;
        
        /// chuyen sang jig test standby
        raspberry_msg.enable.stop = true;
        
        
        /// set flag status for jig test
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP] = true;
        
        /// reset cac co con lai
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_START] = false;
        raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_NONE] = false;
        
        /// write to console
        if(++count_console > 150000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP : %d | mode_test : %3d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP]
            , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(status == JIG_TEST_CLOUD_DATA_SEND_STATUS_RESET_SYSTEM)
    {
        mavlink_comm_rapberry.heartbeat.system_status_send = 5;
        
        if(get_timeOut(1000, JGI_TEST_COMM_RASPBERRY_SEND_ACCEPT_RST))
        {
            /// chi reset sys lan dau chay sau khi cap dien
            if(raspberry_global.count_start_test == 0)
            {
//                NVIC_SystemReset();
            }
            else
            {
                /// reset availables o nhung lan chay sau
                JIG_TEST_console_write(cloud_data);
                JIG_TEST_console_write("JIG_TEST_CLOUD_DATA_SEND_STATUS_RESET_AVAILABLES\n");
                
                /// set flag reset all availables
                gimbal_FSTD_comm.reset_all_availables_gimbal_FSTD = true;
            }
        }
        else
        {
            if(++count_console > 10000)
            {
                count_console = 0;
                
                JIG_TEST_console_write(cloud_data);
                sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_STATUS_RESET_SYSTEM : %d | mode_test : %3d | %d | %d\n"
                , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP]
                , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
                JIG_TEST_console_write(buff);
            }
        }
    }
    
    
    
    
    /// reciever result push data from raspberry
    result = JIG_TEST_comm_raspberry_get_result_push_data();
    
    if(result == RASPBERRY_PUSH_CLOUD_RESULT_NONE)
    {
    
    }
    else if(result == RASPBERRY_PUSH_CLOUD_RESULT_OK)
    {
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
        
        /// get result push data cloud for display
        gimbal_FSTD_comm.mode_test_result[JIG_TEST_GIMBAL_MODE_DONE] = true;
        
        /// get result for display
        gimbal_FSTD_comm.result_pushData = 1;
        
        if(++count_console_result > 50000)
        {
            count_console_result = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "RASPBERRY_PUSH_CLOUD_RESULT_OK : %d | mode_test : %3d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP]
            , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(result == RASPBERRY_PUSH_CLOUD_RESULT_FAIL)
    {
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
        
        /// get result push data cloud for display
        gimbal_FSTD_comm.mode_test_result[JIG_TEST_GIMBAL_MODE_DONE] = false;
        
        /// get result for display
        gimbal_FSTD_comm.result_pushData = 2;
        
        if(++count_console_result > 50000)
        {
            count_console_result = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "RASPBERRY_PUSH_CLOUD_RESULT_FAIL : %d | mode_test : %3d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP]
            , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(result == RASPBERRY_PUSH_CLOUD_BARCODE_READY)
    {
        /// get result barcode ready for display
        gimbal_FSTD_comm.barcode_ready = true;
        
        if(++count_console_result > 50000)
        {
            count_console_result = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "RASPBERRY_PUSH_CLOUD_BARCODE_READY : %d | mode_test : %3d | %d | %d\n"
            , raspberry_global.flag_status_raspberry_cloud_data[JIG_TEST_CLOUD_DATA_SEND_STATUS_STOP]
            , gimbal_FSTD_comm.mode_test, mavlink_comm_rapberry.heartbeat.system_status_send, mavlink_comm_rapberry.heartbeat.custom_mode);
            JIG_TEST_console_write(buff);
        }
    }
}

/** @brief JIG_TEST_comm_raspberry_cloud_data_process
    @return none
*/
void JIG_TEST_comm_raspberry_cloud_data_process(void)
{
    static uint32_t count_console = 0;
    char *cloud_data = "\nLOGIN --->";
    char buff[300];
    JIG_TEST_comm_raspberry_cloud_login_t login = JIG_TEST_CLOUD_LOGIN_NONE;
    
    /// get login
    login = JIG_TEST_comm_raspberry_cloud_login();
    
    /// get login for global
    raspberry_global.user_login = login;
    
    if(login == JIG_TEST_CLOUD_LOGINED)
    {
        /// get status from raspberry and send status test to raspberry
        JIG_TEST_comm_raspberry_send_status_to_cloud_data();
        
        if(++count_console > 500000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_LOGINED: mode_test : %3d | system_status_send %d | custom_mode %d | base_mode %d\n"
            , gimbal_FSTD_comm.mode_test
            , mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else
    {
        if(++count_console > 50000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_WAIT_FOR_LOGIN: mode_test : %3d | system_status_send %d | custom_mode %d | base_mode %d\n"
            , gimbal_FSTD_comm.mode_test
            , mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
        }
    }

    
    if(gimbal_FSTD_comm.mode_test != JIG_TEST_GIMBAL_MODE_DONE)
    {

    }
    else
    {
    
    }
    
//    if(++count_state > 200000)
//    {   
//        count_state = 0;
//        
//        JIG_TEST_console_write(cloud_data);
//        sprintf(buff, "raspberry heartbeat custom_mode : %d | system_status_send : %3d | %f | %f | %f | %f | %f | %f | back_standby : %d\n"
//        , mavlink_comm_rapberry.heartbeat.custom_mode, mavlink_comm_rapberry.heartbeat.system_status_send
//        , raspberry_global.param_value[1]
//        , raspberry_global.param_value[2]
//        , raspberry_global.param_value[3]
//        , raspberry_global.param_value[4]
//        , raspberry_global.param_value[5]
//        , raspberry_global.param_value[6]
//        , raspberry_msg.enable.back_to_standby);
//        JIG_TEST_console_write(buff);
//    }
}

#endif
/**
    @}
*/


/** @group JIG_TEST_COMM_RASPBERRY_PROCESS
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_PROCESS
#define JIG_TEST_COMM_RASPBERRY_PROCESS

/** @brief comm_raspberry_get_heartbeat_ready
    @return none
*/
bool JIG_TEST_comm_raspberry_get_heartbeat_ready(void)
{
    char *comm_ras = "\nRASPBERRY_HEARTBEAT --->";
    static uint32_t count_console;
    static uint8_t timeOut_heartbeat = 0;
    
    bool ret = false;
    
    if(mavlink_comm_rapberry.seen_heartbeat == false)
    {
        if(get_timeOut(1000, JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT))
        {
            if(timeOut_heartbeat ++ > 1)
            {
                
                timeOut_heartbeat = 0;
                
                //// mavlink serialPort5 re init
                JIG_TEST_mavlink_serialPort5_Reinit();
                
                /// write to console not raspberry heartbeat
                JIG_TEST_console_write(comm_ras);
                JIG_TEST_console_write("not found heartbeat from raspberry... Try again\n");
                
                if(raspberry_global.count_timeOut_connect++ > 20 * 2)
                {
                    /// not found heartbeat
                    raspberry_global.disconnect = true;
                }
            }
        }
    }
    else
    {
        /// reset count heartbeat timeOut
        raspberry_global.count_timeOut_connect = 0;
        
        raspberry_global.disconnect = false;
        
        ret = true;
        
        if(++count_console > 500000)
        {
            count_console = 0;
            
            /// write to console got raspberry heartbeat
            JIG_TEST_console_write(comm_ras);
            JIG_TEST_console_write("got heartbeat from raspberry\n");
        }
    }
    
    return ret;
}

/** @brief comm_raspberry_process
    @return none
*/
static bool JIG_TEST_comm_raspberry_USB_speed_write_result_to_console(void)
{
    bool ret = true;
    static uint32_t count_console;
    static uint8_t state_console;
    char buff[100];
    
    if(++count_console > 5000)
    {
        count_console = 0;
        
        state_console ++;
    }
    
    if(state_console == 0)
    {
        JIG_TEST_console_write("-----------------------USB_SPEED_TEST---------------------\n");
        /// next state
        state_console = 1;
    }
    else if(state_console == 1)
    {
        sprintf(buff, "read_speed %f\n", comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]);
        JIG_TEST_console_write(str_usb_speed);
        JIG_TEST_console_write(buff);
        /// next state
        state_console = 2;
    }
    else if(state_console == 2)
    {
        sprintf(buff, "write_speed %f\n", comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]);
        JIG_TEST_console_write(str_usb_speed);
        JIG_TEST_console_write(buff);
        /// next state
        state_console = 3;
    }
    else if(state_console == 3)
    {
        sprintf(buff, "ref_read_speed %f\n", comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]);
        JIG_TEST_console_write(str_usb_speed);
        JIG_TEST_console_write(buff);
        /// next state
        state_console = 4;
    }
    else if(state_console == 4)
    {

        /// next state
        state_console = 5;
    }
    else if(state_console == 5)
    {
        sprintf(buff, "ref_write_speed %f\n", comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]);
        JIG_TEST_console_write(str_usb_speed);
        JIG_TEST_console_write(buff);
        /// next state
        state_console = 6;
    }
    else if(state_console == 6)
    {
        sprintf(buff, "result %f\n", comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT]);
        JIG_TEST_console_write(str_usb_speed);
        JIG_TEST_console_write(buff);
        /// next state
        state_console = 7;
    }
    else if(state_console == 7)
    {
        JIG_TEST_console_write("---------------------------------------------------------\n");
        /// next state
        state_console = 8;
        
        ret = false;
    }
    
    return ret;
        
}

/** @brief comm_raspberry_process
    @return none
*/
void JIG_TEST_comm_raspberry_process(void)
{
    static uint32_t count_console;
    
//    JIG_TEST_mavlink_gimbal_process();
    
    //////////////// GET STATE //////////////////////////////////////////////////////
    if(comm_raspberry.usb_speed.id_send_status == true)
    {
        comm_raspberry.usb_speed.state = JIG_TEST_comm_raspberry_USB_speed_test_get_state(&mavlink_comm_rapberry);
        
        if(++count_console > 200000)
        {
            count_console = 0;
            
//            sprintf(buff, "SEND request state to RASPBERRY %d\n", (uint8_t)comm_raspberry.usb_speed.state);
//            JIG_TEST_console_write(str_usb_speed);
//            JIG_TEST_console_write(buff);
        }
        
        if(comm_raspberry.usb_speed.enable_write_console_usb_speed_result == true)
        {
            comm_raspberry.usb_speed.enable_write_console_usb_speed_result = JIG_TEST_comm_raspberry_USB_speed_write_result_to_console();
        }
    }
    
    //////////////// CHECK STATE //////////////////////////////////////////////////////
    if(comm_raspberry.usb_speed.state == JIG_TEST_USB_SPEED_STATE_STANDBY)
    {
        if(comm_raspberry.usb_speed.disable_send_state == false)
        {
            comm_raspberry.usb_speed.disable_send_state = true;
            
            raspberry_msg.enable_send_request_state = false;
            
            JIG_TEST_comm_raspberry_USB_speed_send_start();
        }
    }
    else if(comm_raspberry.usb_speed.state == JIG_TEST_USB_SPEED_STATE_RUNNING)
    {
        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] = 0.00f;
        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] = 0.00f;
        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] = 0.00f;
        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] = 0.00f;
    }
    else if(comm_raspberry.usb_speed.state == JIG_TEST_USB_SPEED_STATE_DONE)
    {
        
        comm_raspberry.usb_speed.is_sendRequest = true;
        
        comm_raspberry.usb_speed.id_send_status = false;
        
    }
    
    //////////////// SEND REQUEST SPEED //////////////////////////////////////////////////////
    if(comm_raspberry.usb_speed.is_sendRequest == true)
    {
        raspberry_msg.enable_send_request_state = false;
        
        comm_raspberry.usb_speed.is_DoneTest = JIG_TEST_comm_raspberry_USB_speed_check_speed_value(&comm_raspberry);
        
//        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]         = JIG_TEST_comm_raspberry_USB_speed_test_get_speed_value((uint16_t)JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED, &comm_raspberry);
//        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]        = JIG_TEST_comm_raspberry_USB_speed_test_get_speed_value((uint16_t)JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED, &comm_raspberry);
//        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]     = JIG_TEST_comm_raspberry_USB_speed_test_get_speed_value((uint16_t)JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED, &comm_raspberry);
//        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]    = JIG_TEST_comm_raspberry_USB_speed_test_get_speed_value((uint16_t)JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED, &comm_raspberry);
//        comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT]             = JIG_TEST_comm_raspberry_USB_speed_test_get_result(&mavlink_comm_rapberry);
    
        if(mavlink_comm_rapberry.param_value.param_index != 3)
        {
            if(comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] == 0 && mavlink_comm_rapberry.param_value.param_index == 0)
            {
                comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] = mavlink_comm_rapberry.param_value.param_value;
                mavlink_comm_rapberry.param_value.param_value = 0;
            }
            
            if(comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] == 0 && mavlink_comm_rapberry.param_value.param_index == 1)
            {
                comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] = mavlink_comm_rapberry.param_value.param_value;
                mavlink_comm_rapberry.param_value.param_value = 0;
            }
            
            if(comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] == 0 && mavlink_comm_rapberry.param_value.param_index == 6)
            {
                comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] = mavlink_comm_rapberry.param_value.param_value;
                mavlink_comm_rapberry.param_value.param_value = 0;
            }
            
            if(comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] == 0 && mavlink_comm_rapberry.param_value.param_index == 7)
            {
                comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] = mavlink_comm_rapberry.param_value.param_value;
//                mavlink_comm_rapberry.param_value.param_value = 0;
            }
            
            if(comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == 0 && mavlink_comm_rapberry.param_value.param_index == 2)
            {
                comm_raspberry.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] = mavlink_comm_rapberry.param_value.param_value;
                mavlink_comm_rapberry.param_value.param_value = 0;
                raspberry_msg.enable_send_request_result = false;
            }
        }
    }
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


