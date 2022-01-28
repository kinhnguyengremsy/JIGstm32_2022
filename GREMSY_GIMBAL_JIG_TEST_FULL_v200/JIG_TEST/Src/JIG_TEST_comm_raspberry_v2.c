/**
  ******************************************************************************
  * @file JIG_TEST_comm_raspberry_v2.c
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
#include "JIG_TEST_comm_raspberry_v2.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_gimbal_FSTD_v2.h"
#include "JIG_TEST_rtc.h"
#include "timeOut.h"
#include "main.h"
#include "string.h"
#include "math.h"
/* Private typedef------------------------------------------------------------------------------*/

typedef struct
{
    bool disconnect;
    uint8_t count_timeOut_connect;
    
}JIG_TEST_comm_raspberry_private_t;

/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

JIG_TEST_comm_raspberry_private_t           comm_raspberry_private;

extern JIG_TEST_mavlink_gimbal_t            mavlink_comm_rapberry;
extern JIG_TEST_mavlink_comm_respberry_t    raspberry_msg;
extern JIG_TEST_gimbal_FSTD_v2_global_t     gimbal_FSTD_global;
/* Private function prototypes------------------------------------------------------------------------------*/
/** @brief usb_speed_get_feedback_command_end
    @return none
*/
static bool JIG_TEST_usb_speed_get_feedback_command_end(void);

/** @brief usb_speed_test_send_stop
    @return none
*/
static void JIG_TEST_usb_speed_test_send_stop(void);
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_CONFIGURATION
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_CONFIGURATION
#define JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_CONFIGURATION

/** @brief comm_raspberry_v2_cloudData_configuration
    @return none
*/
void JIG_TEST_comm_raspberry_v2_cloudData_configuration(void)
{

}


#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_SEND_CMD
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_SEND_CMD
#define JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_SEND_CMD

/** @brief cloud_Data_get_command_login
    @return JIG_TEST_cloud_Data_send_command_login_t
*/
static JIG_TEST_cloud_Data_send_command_login_t JIG_TEST_cloud_Data_get_command_login(void)
{
    JIG_TEST_cloud_Data_send_command_login_t login = JIG_TEST_CLOUD_DATA_SEND_CMD_LOGIN_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.base_mode == 1)
    {
        login = JIG_TEST_CLOUD_DATA_SEND_CMD_WAIT_LOGIN;
    }
    else if(mavlink_comm_rapberry.heartbeat.base_mode == 2)
    {
        login = JIG_TEST_CLOUD_DATA_SEND_CMD_LOGINED;
    }
    
    return login;
}

/** @brief cloud_Data_get_command_result
    @return JIG_TEST_cloud_Data_send_command_result_t
*/
static JIG_TEST_cloud_Data_send_command_result_t JIG_TEST_cloud_Data_get_command_result(void)
{
    JIG_TEST_cloud_Data_send_command_result_t result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.system_status == 1)
    {
        result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 2)
    {
        result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 3)
    {
        result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_BARCODE_READY;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 4)
    {
        result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_TIMEOUT;
    }
    else if(mavlink_comm_rapberry.heartbeat.system_status == 5)
    {
        result = JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_CANT_READ_UUID;
    }
    
    return result;
}

/** @brief cloud_Data_get_command_status
    @return JIG_TEST_cloud_Data_send_command_status_t
*/
static JIG_TEST_cloud_Data_send_command_status_t JIG_TEST_cloud_Data_get_command_status(void)
{
    JIG_TEST_cloud_Data_send_command_status_t status = JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_NONE;
    
    if(mavlink_comm_rapberry.heartbeat.custom_mode == 1)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_START;
    }
    else if(mavlink_comm_rapberry.heartbeat.custom_mode == 2)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_STOP;
    }
    else if(mavlink_comm_rapberry.heartbeat.custom_mode == 3)
    {
        status = JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_RESET;
    }
    
    return status;
}


#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_SEND_TO_CLOUD_DATA
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_SEND_TO_CLOUD_DATA
#define JIG_TEST_COMM_RASPBERRY_V2_SEND_TO_CLOUD_DATA

/** @brief 
    @return 
*/



#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_PROCESS
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_PROCESS
#define JIG_TEST_COMM_RASPBERRY_V2_CLOUD_DATA_PROCESS

static bool JIG_TEST_comm_raspberry_get_heartbeat_ready(void)
{
    char *comm_ras = "\nRASPBERRY_HEARTBEAT --->";
    static uint32_t count_console;
    static uint8_t timeOut_heartbeat = 0;
    
    bool ret = false;
    
    if(mavlink_comm_rapberry.seen_heartbeat == false)
    {
//        if(get_timeOut(1000, JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT))
//        {
            if(timeOut_heartbeat ++ > 1)
            {
                
                timeOut_heartbeat = 0;
                
                //// mavlink serialPort5 re init
                JIG_TEST_mavlink_serialPort5_Reinit();
                
                /// write to console not raspberry heartbeat
                JIG_TEST_console_write(comm_ras);
                JIG_TEST_console_write("not found heartbeat from raspberry... Try again\n");
                
                if(comm_raspberry_private.count_timeOut_connect++ > 20 * 2)
                {
                    /// not found heartbeat
                    comm_raspberry_private.disconnect = true;
                }
            }
//        }
    }
    else
    {
        
        /// reset heartbeat
        mavlink_comm_rapberry.seen_heartbeat = false;
        
        /// reset count heartbeat timeOut
        comm_raspberry_private.count_timeOut_connect = 0;
        
        comm_raspberry_private.disconnect = false;
        
        ret = true;
        
        if(++count_console > 200000)
        {
            count_console = 0;
            
            /// write to console got raspberry heartbeat
            JIG_TEST_console_write(comm_ras);
            JIG_TEST_console_write("got heartbeat from raspberry\n");
        }
    }
    
    return ret;
}

/** @brief comm_raspberry_cloud_data_send_value_jig_test
    @return none
*/
static bool JIG_TEST_comm_raspberry_v2_cloud_data_send_value_jig_test(void)
{
    bool ret = false;
    
    if(mavlink_comm_rapberry.param_request_read.param_index == 1)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[1];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 2)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[2];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 3)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[3];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 4)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[4];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else if(mavlink_comm_rapberry.param_request_read.param_index == 5)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[5];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    else     if(mavlink_comm_rapberry.param_request_read.param_index == 6)
    {
        int16_t value = (int16_t)gimbal_FSTD_global.param_value[6];
        
        /// setting value msg send to raspberry
        raspberry_msg.enable.send_param_value = true;
        raspberry_msg.enable.param_value = value;
    }
    
    return ret;
}

/** @brief comm_raspberry_v2_cloudData_get_command
    @return none
*/
static void JIG_TEST_comm_raspberry_v2_cloudData_get_command(void)
{
    char *get_from_cloud_data = "\nGET_FROM_CLOUD_DATA ---> ";
    char buff[200];
    static uint32_t count_console = 0;

    /// kiem tra cloud data login
    if(JIG_TEST_cloud_Data_get_command_login() == JIG_TEST_CLOUD_DATA_SEND_CMD_LOGINED)
    {
        gimbal_FSTD_global.cloudData_command.get_login = true;
    }
    else if(JIG_TEST_cloud_Data_get_command_login() == JIG_TEST_CLOUD_DATA_SEND_CMD_WAIT_LOGIN)
    {
        gimbal_FSTD_global.cloudData_command.get_login = false;
    }
    
    /// kiem tra barcode scan ready
    if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK)
    {
        gimbal_FSTD_global.cloudData_command.result_pushData = (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK;
        
        if(++count_console > 50000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(get_from_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL)
    {
        gimbal_FSTD_global.cloudData_command.result_pushData = (uint8_t)JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL;
        
        if(++count_console > 50000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(get_from_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
        }
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_BARCODE_READY)
    {
        gimbal_FSTD_global.cloudData_command.get_bardCode = true;
        
        if(++count_console > 150000)
        {
            count_console = 0;
            
          #if (USE_CONSOLE_COMM_RAS_CLOUD == 1)
            JIG_TEST_console_write(get_from_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_BARCODE_READY : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
          #endif
        }
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_TIMEOUT)
    {
        /// jig timeOut, no Done signal
        gimbal_FSTD_global.cloudData_command.jig_timeOut = true;
        
        /// next state error
        gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
        
        if(++count_console > 150000)
        {
            count_console = 0;
            
            JIG_TEST_console_write(get_from_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_TIMEOUT : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
        }
        
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_CANT_READ_UUID)
    {
        
        /// cant read uuid
        gimbal_FSTD_global.cloudData_command.cant_read_uuid = true;
        
        /// send end usb speed test
        /// ktra da nhan duoc command end from Pi
        if(gimbal_FSTD_global.usb_speed.send_end_done == false && gimbal_FSTD_global.mode_test >= JIG_TEST_GIMBAL_V2_STATE_WAIT_START)
        {
            if(get_timeOut(100, JIG_TEST_USB_SPEED_SEND_END)) /// send end cham lai
            {
                JIG_TEST_console_write(get_from_cloud_data);
                sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_CANT_READ_UUID : %d | %d | %d\n"
                , mavlink_comm_rapberry.heartbeat.system_status_send
                , mavlink_comm_rapberry.heartbeat.custom_mode
                , mavlink_comm_rapberry.heartbeat.base_mode);
                JIG_TEST_console_write(buff);
                
                /// send stop
                JIG_TEST_usb_speed_test_send_stop();
                
                if(JIG_TEST_usb_speed_get_feedback_command_end() == true)
                {
                    gimbal_FSTD_global.usb_speed.send_end_done = true;
                    
                    JIG_TEST_console_write("\nUSB_SPEED ---> send end done\n");
                    
                    /// next state error
                    gimbal_FSTD_global.state_test = JIG_TEST_GIMBAL_V2_STATE_ERROR;
                }
                
            }
        }
    }
    
    /// kiem tra status from raspberry
    if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_START)
    {
        gimbal_FSTD_global.cloudData_command.get_start_stop = true;
        
        /// reset flag reset 
        gimbal_FSTD_global.cloudData_command.get_reset = false;
    }
    else if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_STOP)
    {
        gimbal_FSTD_global.cloudData_command.get_start_stop = false;
        
        /// reset flag reset 
        gimbal_FSTD_global.cloudData_command.get_reset = false;
        
        /// set flag auto back scan new barCode
        gimbal_FSTD_global.cloudData_command.auto_back = true;
    }
    else if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_RESET)
    {
        gimbal_FSTD_global.cloudData_command.get_reset = true;
        
        /// reset flag start stop
        gimbal_FSTD_global.cloudData_command.get_start_stop = false;
    }
}

/** @brief comm_raspberry_v2_send_command_to_cloudData
    @return none
*/
static void JIG_TEST_comm_raspberry_v2_send_command_status_to_cloudData(void)
{
    char *send_to_cloud_data = "\nSEND_TO_CLOUD_DATA ---> ";
    char buff[200];
    static uint32_t count_console = 0;
    static uint8_t count_jig_resetting;
    
    if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_NONE)
    {
        /// jig test : standby
        mavlink_comm_rapberry.heartbeat.system_status_send = 1;
        
        if(++count_console > 150000)
        {
            count_console = 0;
          #if (USE_CONSOLE_COMM_RAS_CLOUD == 1)
            JIG_TEST_console_write(send_to_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_NONE : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
          #endif
        }
        
        /// reset count reset cmd
        count_jig_resetting = 0;
    }
    else if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_START)
    {
        /// send test done khi jig test run mode DONE
        if(gimbal_FSTD_global.mode_test == JIG_TEST_GIMBAL_V2_MODE_DONE)
        {
            /// kiem tra flag read param done
            if(gimbal_FSTD_global.read_param_done == true)
            {
                /// jig test : test done
                mavlink_comm_rapberry.heartbeat.system_status_send = 3;
                
                /// cho phep send param value khi co message param request
                JIG_TEST_comm_raspberry_v2_cloud_data_send_value_jig_test();
            }
        }
        else if(gimbal_FSTD_global.mode_test == JIG_TEST_GIMBAL_V2_MODE_ERROR)
        {
            /// kiem tra flag read param done
            if(gimbal_FSTD_global.read_param_done == true)
            {
                /// jig test : test fail
                mavlink_comm_rapberry.heartbeat.system_status_send = 4;
                
                /// cho phep send param value khi co message param request
                JIG_TEST_comm_raspberry_v2_cloud_data_send_value_jig_test();
            }
        }
        else
        {
            /// jig test : running test
            mavlink_comm_rapberry.heartbeat.system_status_send = 2;
            
            /// reset param request read index
            mavlink_comm_rapberry.param_request_read.param_index = 0;
        }
        
        if(++count_console > 150000)
        {
            count_console = 0;
          
          #if (USE_CONSOLE_COMM_RAS_CLOUD == 1)
            JIG_TEST_console_write(send_to_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_START : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
          #endif
        }
        
        /// reset count reset cmd
        count_jig_resetting = 0;
    }
    else if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_STOP)
    {
        /// jig test : standby
        mavlink_comm_rapberry.heartbeat.system_status_send = 1;
        
        gimbal_FSTD_global.cloudData_command.get_start_stop = false;
        
        if(++count_console > 150000)
        {
            count_console = 0;
            
          #if (USE_CONSOLE_COMM_RAS_CLOUD == 1)
            JIG_TEST_console_write(send_to_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_STOP : %d | %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode);
            JIG_TEST_console_write(buff);
          #endif
        }
        
        /// reset count reset cmd
        count_jig_resetting = 0;
    }
    else if(JIG_TEST_cloud_Data_get_command_status() == JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_RESET)
    {
        if(gimbal_FSTD_global.state_test <= JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM)
        mavlink_comm_rapberry.heartbeat.system_status_send = 5;
        
        /// jig test : resetting
        if(gimbal_FSTD_global.usb_speed.send_end_done == true)
        mavlink_comm_rapberry.heartbeat.system_status_send = 5;
        
        if(gimbal_FSTD_global.state_test == JIG_TEST_GIMBAL_V2_STATE_DONE)
        mavlink_comm_rapberry.heartbeat.system_status_send = 5;
        
        if(gimbal_FSTD_global.state_test == JIG_TEST_GIMBAL_V2_STATE_ERROR)
        mavlink_comm_rapberry.heartbeat.system_status_send = 5;
        
        gimbal_FSTD_global.cloudData_command.get_reset = true;
        
        if(get_timeOut(500, JIG_TEST_SEND_JIG_RESETTING))
        {
            JIG_TEST_console_write(send_to_cloud_data);
            sprintf(buff, "JIG_TEST_CLOUD_DATA_SEND_CMD_STATUS_RESET : %d | %d | %d | reset : %d | %d\n"
            ,  mavlink_comm_rapberry.heartbeat.system_status_send
            , mavlink_comm_rapberry.heartbeat.custom_mode
            , mavlink_comm_rapberry.heartbeat.base_mode
            , count_jig_resetting
            , gimbal_FSTD_global.usb_speed.send_end_done);
            JIG_TEST_console_write(buff);

            /// kiem tra : jig mode DONE nhan duoc reset cua Pi thi reset mem
            if(gimbal_FSTD_global.state_test == JIG_TEST_GIMBAL_V2_STATE_ERROR)
            {
                if(gimbal_FSTD_global.usb_speed.send_end_done == true)
                count_jig_resetting ++;
                
                if(count_jig_resetting >= 5)
                {
//                    NVIC_SystemReset();
                }
            }
        }    
    }
}

/** @brief comm_raspberry_v2_send_command_result_to_cloudData
    @return none
*/
static void JIG_TEST_comm_raspberry_v2_send_command_result_to_cloudData(void)
{
    if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_OK)
    {
        /// jig test : jig got result from Pi
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_PUSH_FAIL)
    {
        /// jig test : jig got result from Pi
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_CANT_READ_UUID)
    {
        /// jig test : jig got result from Pi
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_TIMEOUT)
    {
        /// jig test : jig got result from Pi
        mavlink_comm_rapberry.heartbeat.system_status_send = 6;
    }
    else if(JIG_TEST_cloud_Data_get_command_result() == JIG_TEST_CLOUD_DATA_SEND_CMD_RESULT_BARCODE_READY)
    {
        
    }
}

/** @brief comm_raspberry_v2_cloudData_process
    @return none
*/
void JIG_TEST_comm_raspberry_v2_cloudData_process(void)
{
    /// get command from cloud data
    JIG_TEST_comm_raspberry_v2_cloudData_get_command();
    
    /// send command status to cloud data
    JIG_TEST_comm_raspberry_v2_send_command_status_to_cloudData();
    
    /// send command result to cloud data
    JIG_TEST_comm_raspberry_v2_send_command_result_to_cloudData();
}


#endif
/**
    @}
*/


/** @group JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_GET_CMD
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_GET_CMD
#define JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_GET_CMD

/** @brief usb_speed_get_status
    @return JIG_TEST_usb_speed_status_t
*/
static JIG_TEST_usb_speed_status_t JIG_TEST_usb_speed_get_status(JIG_TEST_mavlink_gimbal_t *mavlink_channel)
{
    JIG_TEST_usb_speed_status_t status = JIG_TEST_USB_SPEED_STATUS_STANDBY;
    
    /// enable send request status
    raspberry_msg.enable_send_request_state = true;
    
    if(mavlink_channel->param_value.param_index == 3)
    {
        if(mavlink_channel->param_value.param_value == 0)
        {
            status = JIG_TEST_USB_SPEED_STATUS_STANDBY;
        }
        else if(mavlink_channel->param_value.param_value == 1)
        {
            status = JIG_TEST_USB_SPEED_STATUS_RUNNING;
        }
        else if(mavlink_channel->param_value.param_value == 2)
        {
            status = JIG_TEST_USB_SPEED_STATUS_DONE;
        }
    }
    
    return status;
}

/** @brief usb_speed_get_feedback_command_start
    @return none
*/
static bool JIG_TEST_usb_speed_get_feedback_command_start(void)
{
    bool ret = false;
    
    if(mavlink_comm_rapberry.param_value.param_index == 4)
    {
        ret = true;
    }
    
    return ret;
}

/** @brief usb_speed_get_feedback_command_end
    @return none
*/
static bool JIG_TEST_usb_speed_get_feedback_command_end(void)
{
    bool ret = false;
    
    if(mavlink_comm_rapberry.param_value.param_index == 5)
    {
        ret = true;
    }
    
    return ret;
}

/** @brief usb_speed_get_value_speed
    @return none
*/
static bool JIG_TEST_usb_speed_get_value_speed(float *get_value, uint16_t param_index)
{
    bool ret = false;
    
    if(*get_value == 0.00f)
    {
        if(mavlink_comm_rapberry.param_value.param_index == param_index)
        {
            /// get speed
            *get_value = mavlink_comm_rapberry.param_value.param_value;
            
            /// reset param value
            mavlink_comm_rapberry.param_value.param_value = 0;
            
            ret = true;
        }
    }
    
    return ret;
}
#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_SEND_CMD
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_SEND_CMD
#define JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_SEND_CMD

/** @brief usb_speed_test_send_start
    @return none
*/
static void JIG_TEST_usb_speed_test_send_start(void)
{
    JIG_TEST_console_write("send start test\n");
    
    raspberry_msg.enable_send_start = true;
}

/** @brief usb_speed_test_send_stop
    @return none
*/
static void JIG_TEST_usb_speed_test_send_stop(void)
{
    JIG_TEST_console_write("\nUSB_SPEED --->send stop test\n");
    
    raspberry_msg.enable_send_end = true;
}

/** @brief usb_speed_send_request_value_speed
    @return none
*/
static void JIG_TEST_usb_speed_send_request_value_speed(void)
{
    static uint8_t state = 0;
    static uint32_t count_console = 0;
    char *request_speed = "\nUSB_SPEED_REQUEST --->";
    
    switch(state)
    {
        
        case 0:
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == 0.00f)
            {
                /// enable request speed
                raspberry_msg.enable_send_request_result = true;
                
                if(++count_console > 50000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_result\n");
                }
            }
            else
            {
                raspberry_msg.enable_send_request_result = false;
                
                JIG_TEST_console_write(request_speed);
                JIG_TEST_console_write("enable_send_request_result - DONE\n");
                
                /// next state
                state = 1;
            }
        }break;
        case 1:
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] == 0.00f)
            {
                /// enable request speed
                raspberry_msg.enable_send_request_read_speed = true;
                
                if(++count_console > 50000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_read_speed\n");
                }
            }
            else
            {
                raspberry_msg.enable_send_request_read_speed = false;
                
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_read_speed - DONE\n");
                
                /// next state
                state = 2;
            }
        }break;
        case 2:
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] == 0.00f)
            {
                /// enable request speed
                raspberry_msg.enable_send_request_write_speed = true;
                
                if(++count_console > 50000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_write_speed\n");
                }
            }
            else
            {
                raspberry_msg.enable_send_request_write_speed = false;
                
                JIG_TEST_console_write(request_speed);
                JIG_TEST_console_write("enable_send_request_write_speed - DONE\n");
                
                /// next state
                state = 3;
            }
        }break;
        case 3:
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] == 0.00f)
            {
                /// enable request speed
                raspberry_msg.enable_send_request_ref_read_speed = true;
                
                if(++count_console > 50000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_ref_read_speed\n");
                }
            }
            else
            {
                raspberry_msg.enable_send_request_ref_read_speed = false;
                
                JIG_TEST_console_write(request_speed);
                JIG_TEST_console_write("enable_send_request_ref_read_speed - DONE\n");
                
                /// next state
                state = 4;
            }
        }break;
        case 4:
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] == 0.00f)
            {
                /// enable request speed
                raspberry_msg.enable_send_request_ref_write_speed = true;
                
                if(++count_console > 50000)
                {
                    count_console = 0;
                    
                    JIG_TEST_console_write(request_speed);
                    JIG_TEST_console_write("enable_send_request_ref_write_speed\n");
                }
            }
            else
            {
                raspberry_msg.enable_send_request_ref_write_speed = false;
                
                JIG_TEST_console_write(request_speed);
                JIG_TEST_console_write("enable_send_request_ref_write_speed - DONE\n");
                
                /// next state
                state = 5;
            }
        }break;
        case 5:
        {
            raspberry_msg.enable_send_request_ref_write_speed = false;
            
            /// next state
            state = 6;
        }break;
    }
}

/** @brief usb_speed_get_status
    @return JIG_TEST_usb_speed_status_t
*/
static bool JIG_TEST_usb_speed_check_value_speed(void)
{
    bool ret = false;
    
    if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] != 1)
    {
        if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED] != 0.00f
        && gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED] != 0.00f
        && gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED] != 0.00f
        && gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED] != 0.00f
        )
        {
            /// reset flag send request result
            raspberry_msg.enable_send_request_result = false;
            raspberry_msg.enable_send_request_ref_write_speed = false;
            
            /// ktra da nhan duoc command end from Pi
            if(get_timeOut(100, JIG_TEST_USB_SPEED_SEND_END)) /// send end cham lai
            {
                if(JIG_TEST_usb_speed_get_feedback_command_end() == true)
                {
                    gimbal_FSTD_global.usb_speed.send_end_done = true;
                    
                    JIG_TEST_console_write("\nUSB_SPEED ---> send end done\n");
                    
                    ret = true;
                }
                else
                {
                    /// send stop
                    JIG_TEST_usb_speed_test_send_stop();
                }
            }
        }
        else
        {
            /// khi jig chay den mode nay moi request value speed usb
            if((uint8_t)gimbal_FSTD_global.mode_test > (uint8_t)JIG_TEST_GIMBAL_V2_MODE_PPM)
            {
                /// send request all speed and result
                JIG_TEST_usb_speed_send_request_value_speed();
            }
        }
    }
    else
    {
        /// reset flag send request result
        raspberry_msg.enable_send_request_result = false;
        
        /// ktra da nhan duoc command end from Pi
        if(get_timeOut(100, JIG_TEST_USB_SPEED_SEND_END)) /// send end cham lai
        {
            if(JIG_TEST_usb_speed_get_feedback_command_end() == true)
            {
                gimbal_FSTD_global.usb_speed.send_end_done = true;
                
                JIG_TEST_console_write("\nUSB_SPEED ---> send end done ---------- NO USB -----------\n");
                
                ret = true;
            }
            else
            {
                /// send stop
                JIG_TEST_usb_speed_test_send_stop();
            }
        }
    }
    
    return ret;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_PROCESS
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_PROCESS
#define JIG_TEST_COMM_RASPBERRY_V2_USB_SPEED_PROCESS

/** @brief usb_speed_process
    @return none
*/
void JIG_TEST_usb_speed_process(void)
{
    char *usb_speed = "\nUSB_SPEED --->";
    static uint32_t count_console = 0;
    
//    if(gimbal_FSTD_global.usb_speed.enable_test == true)
    if(gimbal_FSTD_global.usb_speed.test_done == false && (uint8_t)gimbal_FSTD_global.state_test > (uint8_t)JIG_TEST_GIMBAL_V2_STATE_WAIT_START)
    {
        JIG_TEST_usb_speed_status_t status = JIG_TEST_USB_SPEED_STATUS_STANDBY;
        
        /// get status usb speed test from Pi
        status = JIG_TEST_usb_speed_get_status(&mavlink_comm_rapberry);
        
        gimbal_FSTD_global.usb_speed.status = (uint8_t)status;
        
        switch((uint8_t)status)
        {
            case JIG_TEST_USB_SPEED_STATUS_STANDBY:
            {
                if(gimbal_FSTD_global.usb_speed.send_start == false)
                {
                    if(get_timeOut(500, JIG_TEST_USB_SPEED_SEND_START))
                    {
                        JIG_TEST_console_write(usb_speed);
                        
                        /// kiem tra da nhan duoc command send start from Pi
                        if(JIG_TEST_usb_speed_get_feedback_command_start() == true)
                        {
                            JIG_TEST_console_write("JIG_TEST_USB_SPEED_STATUS_STANDBY | send start done\n");
                            
                            gimbal_FSTD_global.usb_speed.send_start = true;
                            
                            /// storage start usb to backup resister
                            JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_global.usb_speed.send_start, LL_RTC_BKP_DR14);
                        }
                        else
                        {
                            JIG_TEST_console_write("JIG_TEST_USB_SPEED_STATUS_STANDBY | ");
                            
                            /// send start test to Pi
                            JIG_TEST_usb_speed_test_send_start();
                        }
                    }
                }
            }break;
            case JIG_TEST_USB_SPEED_STATUS_RUNNING:
            {
                if(++count_console > 150000)
                {
                    count_console = 0;
                    
                    /// reset all value speed
                    memset(&gimbal_FSTD_global.usb_speed.value_speed_test, 0.00, 8);
                    
                    JIG_TEST_console_write(usb_speed);
                    JIG_TEST_console_write("JIG_TEST_USB_SPEED_STATUS_RUNNING\n");
                }
                
            }break;
            case JIG_TEST_USB_SPEED_STATUS_DONE:
            {
                gimbal_FSTD_global.usb_speed.test_done = true;
                
                /// test done reset flag enable test ---> request value speed and result
                gimbal_FSTD_global.usb_speed.enable_test = false;
                
                /// storage start usb to backup resister
                JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_global.usb_speed.send_start, LL_RTC_BKP_DR14);
                JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_global.usb_speed.test_done, LL_RTC_BKP_DR15);
                
                status = JIG_TEST_USB_SPEED_STATUS_RETRY;
            }break;
            case JIG_TEST_USB_SPEED_STATUS_RETRY:
            {
            }break;
        }
    }
    
    /// test done --> send request value speed
    if(gimbal_FSTD_global.usb_speed.test_done == true && gimbal_FSTD_global.usb_speed.send_end_done == false)
    {
        if(JIG_TEST_usb_speed_check_value_speed() == true)
        {
//            gimbal_FSTD_global.usb_speed.test_done = false;
            
            gimbal_FSTD_global.usb_speed.send_start = false;
            
            /// storage start usb to backup resister
            JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_global.usb_speed.send_start, LL_RTC_BKP_DR14);
//            JIG_TEST_rtc_storage_register_value((uint32_t)gimbal_FSTD_global.usb_speed.test_done, LL_RTC_BKP_DR15);
        }
        else
        {
            /// get value speed
            JIG_TEST_usb_speed_get_value_speed(
            &gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED], 
            JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED);
            
            JIG_TEST_usb_speed_get_value_speed(
            &gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED], 
            JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED);
            
            JIG_TEST_usb_speed_get_value_speed(
            &gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED], 
            JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED);
            
            JIG_TEST_usb_speed_get_value_speed(
            &gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED], 
            JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED);
            
            JIG_TEST_usb_speed_get_value_speed(
            &gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT], 
            JIG_TEST_USB_SPEED_READ_STATE_RESULT);
        }
    }

    static uint32_t usb_speed_console = 0;
    
    if(HAL_GetTick() - usb_speed_console > 5000)
    {
        char buff[200];
        
        usb_speed_console = HAL_GetTick();
        
        JIG_TEST_console_write(usb_speed);
        sprintf(buff, " | enable_test : %d | send_end_done : %d | send_start : %d | test_done : %d | status : %d\n"
        , gimbal_FSTD_global.usb_speed.enable_test
        , gimbal_FSTD_global.usb_speed.send_end_done
        , gimbal_FSTD_global.usb_speed.send_start
        , gimbal_FSTD_global.usb_speed.test_done
        ,gimbal_FSTD_global.usb_speed.status
        );
        JIG_TEST_console_write(buff);
        
        if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] != 0)
        {
            char buff1[200];
            
            JIG_TEST_console_write(usb_speed);
            sprintf(buff1, " | read speed : %f | write speed : %f | ref read speed : %f | ref write speed : %f\n"
            , gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]
            , gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]
            , gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]
            , gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]
            );
            JIG_TEST_console_write(buff1);
        }
    }
    
}


#endif
/**
    @}
*/


/** @group JIG_TEST_COMM_RASPBERRY_V2_HEARTBEAT_TIMEOUT_PROCESS
    @{
*/#ifndef JIG_TEST_COMM_RASPBERRY_V2_HEARTBEAT_TIMEOUT_PROCESS
#define JIG_TEST_COMM_RASPBERRY_V2_HEARTBEAT_TIMEOUT_PROCESS

/** @brief comm_raspberry_v2_heartbeat_timeOut_process
    @return none
*/
void JIG_TEST_comm_raspberry_v2_heartbeat_timeOut_process(void)
{
    /// kiem tra heartbeat raspberry
    if(get_timeOut(1500, JGI_TEST_COMM_RASPBERRY_HEARTBEAT_TIMEOUT))
    gimbal_FSTD_global.cloudData_command.get_heartbeatReady = JIG_TEST_comm_raspberry_get_heartbeat_ready();
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


