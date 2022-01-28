/**
  ******************************************************************************
  * @file JIG_TEST_mavlink_gimbal.c
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
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_console.h"
#include "mavlinkProtocol.h"
#include "timeOut.h"
#include "main.h"
#include "string.h"
/* Private typedef------------------------------------------------------------------------------*/

/* Private define------------------------------------------------------------------------------*/

#ifndef PI
#define PI                  3.141592654
#endif

#define PI2ANGLE            (180.0/PI)

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM2;
JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM4;
JIG_TEST_mavlink_gimbal_t mavlink_comm_rapberry;

JIG_TEST_mavlink_comm_respberry_t raspberry_msg;

mav_state_t mav_state_COM2;
mav_state_t mav_state_COM4;
mav_state_t mav_state_comm_raspberry;

#if (COMM_ESP32 == 1)
		mavlink_msg_heartbeat_t controlJig;
#endif
/* Private function prototypes------------------------------------------------------------------------------*/

/** @brief mavlink_gimbal_message_reciver_handle
    @return none
*/
static void JIG_TEST_mavlink_gimbal_message_reciver_handle(mav_state_t *mav, JIG_TEST_mavlink_gimbal_t *gimbal_channel);

/**
 * @brief  mavlink_gimbal_send_heartbeat
 * @param channel : mavlink channel
 * @return none
 */
static void JIG_TEST_mavlink_gimbal_send_heartbeat(mavlink_channel_t channel);

/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_MAVLINK_GIMBAL_CONFIGURATION
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_CONFIGURATION
#define JIG_TEST_MAVLINK_GIMBAL_CONFIGURATION

/** @brief mavlink_gimbal_configuration
    @return 
*/
void JIG_TEST_mavlink_gimbal_configuration(void)
{
    mavlinkProtocol_init();
}

/** @brief mavlink_gimbal_Reinit
    @return 
*/
void JIG_TEST_mavlink_serialPort3_Reinit(void)
{
    mavlinkProtocol_serialPort3_Deinit();
    HAL_Delay(10);
    
    mavlinkProtocol_serialPort3_init();
}

/** @brief mavlink_raspberry_Reinit
    @return 
*/
void JIG_TEST_mavlink_serialPort5_Reinit(void)
{
    mavlinkProtocol_serialPort5_Deinit();
    HAL_Delay(10);
    
    mavlinkProtocol_serialPort5_init();
}

#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_GIMBAL_SEND_DATA
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_SEND_DATA
#define JIG_TEST_MAVLINK_GIMBAL_SEND_DATA

/** @brief mavlink_gimbal_sendData
    @return none
*/
static void JIG_TEST_mavlink_gimbal_sendData(void)
{
    static uint8_t count_send_hearbeat;
    static bool enable_send;
    
    if(get_timeOut(500, JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM2))
    {
        /// to heartbeat to gimbal COM2
        JIG_TEST_mavlink_gimbal_send_heartbeat(MAVLINK_COMM_1);
        
        /// count up heartbeat gimbal COM2
        mavlink_gimbal_COM2.heartbeat.count ++;
    }
    
    if(get_timeOut(505, JIG_TEST_MAVLINK_GIMBAL_SEND_HB_COM4))
    {
        enable_send = true;
        
        count_send_hearbeat ++;
        if(count_send_hearbeat > 1) count_send_hearbeat = 0;
    }
    
    if(enable_send == true)
    {
        if(count_send_hearbeat == 1)
        {
            /// to heartbeat to gimbal COM4
            JIG_TEST_mavlink_gimbal_send_heartbeat(MAVLINK_COMM_2);
            
            enable_send = false;
        }
        else
        {
            /// to heartbeat to raspberry
            JIG_TEST_mavlink_gimbal_send_heartbeat(MAVLINK_COMM_3);
            
            enable_send = false;
            
//            JIG_TEST_console_write((uint8_t *)"SEND heartbeat to RASPBERRY \n");
        }
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_GIMBAL_READ_DATA
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_READ_DATA
#define JIG_TEST_MAVLINK_GIMBAL_READ_DATA


/** @brief mavlink_gimbal_readData
    @return none
*/
static void JIG_TEST_mavlink_gimbal_readData(void)
{
    /// read Data from COM2 gimbal
    if(mavlinkProtocol_serialPort3_readData(&mav_state_COM2) == 1)
    {
        JIG_TEST_mavlink_gimbal_message_reciver_handle(&mav_state_COM2, &mavlink_gimbal_COM2);
    }
    
    /// read Data from COM4 gimbal
    if(mavlinkProtocol_serialPort4_readData(&mav_state_COM4) == 1)
    {
        JIG_TEST_mavlink_gimbal_message_reciver_handle(&mav_state_COM4, &mavlink_gimbal_COM4);
    }
    
    /// read Data from COM2 gimbal
    if(mavlinkProtocol_serialPort5_readData(&mav_state_comm_raspberry) == 1)
    {
        JIG_TEST_mavlink_gimbal_message_reciver_handle(&mav_state_comm_raspberry, &mavlink_comm_rapberry);
    }
}


#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_GIMBAL_MESSAGE_HANDLE
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_MESSAGE_HANDLE
#define JIG_TEST_MAVLINK_GIMBAL_MESSAGE_HANDLE

/**
 * @brief  mavlink_gimbal_send_heartbeat
 * @param channel : mavlink channel
 * @return none
 */
static void JIG_TEST_mavlink_gimbal_send_heartbeat(mavlink_channel_t channel)
{
    mavlink_message_t       msg;
    mavlink_heartbeat_t     heartbeat;
    uint16_t                len = 0;
    
    if(channel == MAVLINK_COMM_1)
    {
        heartbeat.type          = MAV_TYPE_ONBOARD_TESTER; 
        heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
        heartbeat.base_mode     = 0;
        heartbeat.custom_mode   = 0; 
        heartbeat.system_status = mavlink_gimbal_COM2.heartbeat.system_status_send;//MAV_STATE_ACTIVE;
    }
    else if(channel == MAVLINK_COMM_2)
    {
        heartbeat.type          = MAV_TYPE_ONBOARD_TESTER; 
        heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
        heartbeat.base_mode     = 0;
        heartbeat.custom_mode   = 0; 
        heartbeat.system_status = MAV_STATE_ACTIVE;
    }
    else if(channel == MAVLINK_COMM_3)
    {
        #if (COMM_ESP32 == 1)
    
            heartbeat.type          = mavlink_comm_rapberry.heartbeat.type; 
            heartbeat.autopilot     = controlJig.autopilot;
            heartbeat.base_mode     = mavlink_comm_rapberry.heartbeat.base_mode;
            heartbeat.custom_mode   = controlJig.custom_mode; 
            heartbeat.system_status = controlJig.system_status;
    
        #else
            heartbeat.type          = MAV_TYPE_ONBOARD_TESTER; 
            heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
            heartbeat.base_mode     = 0;
            heartbeat.custom_mode   = 0; 
            heartbeat.system_status = mavlink_comm_rapberry.heartbeat.system_status_send;//MAV_STATE_ACTIVE;
        #endif
    }
    
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_heartbeat_encode_chan(  JIG_TEST_ID,
                                        MAV_COMP_ID_SYSTEM_CONTROL,
                                        channel,
                                        &msg,
                                        &heartbeat);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}


/** @brief mavlink_gimbal_message_reciver_handle
    @return none
*/
static void JIG_TEST_mavlink_gimbal_send_param(mavlink_channel_t channel, float param_value, char *param_id)
{
    uint16_t len;
    mavlink_message_t msg;
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;

    mavlink_msg_param_set_pack(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &msg, SYSID_ONBOARD, MAV_COMP_ID_GIMBAL,
                               param_id, (float)param_value, MAVLINK_TYPE_UINT16_T);
    
    chan_status->current_tx_seq = saved_seq;

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {    
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_param_value
    @return none
*/
static void JIG_TEST_mavlink_gimbal_param_value(mavlink_channel_t channel, int16_t param_index, int16_t value, char* param_id)
{
    mavlink_message_t msg;
    mavlink_param_value_t param_value;
    uint16_t len = 0;
    
    mav_array_memcpy(param_value.param_id, param_id, sizeof(char) * 16);
    
    param_value.param_count = 0;
    param_value.param_index = param_index;
    param_value.param_type = 0;
    param_value.param_value = value;
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_param_value_encode_chan(SYSID_ONBOARD,
                                        MAV_COMP_ID_SYSTEM_CONTROL,
                                        channel,
                                        &msg,
                                        &param_value);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}


/** @brief mavlink_gimbal_param_request_read
    @return none
*/
static void JIG_TEST_mavlink_gimbal_param_request_read(mavlink_channel_t channel, int16_t param_index, char* param_id)
{
    mavlink_message_t msg;
    mavlink_param_request_read_t request_read;
    uint16_t len = 0;
    
    mav_array_memcpy(request_read.param_id, param_id, sizeof(char) * 16);
    request_read.param_index = param_index;
    request_read.target_component = MAV_COMP_ID_GIMBAL;
    request_read.target_system = mavlink_gimbal_COM2.vehicle_system_id;
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_param_request_read_encode_chan( SYSID_ONBOARD,
                                                MAV_COMP_ID_SYSTEM_CONTROL,
                                                channel,
                                                &msg,
                                                &request_read);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_control_motor
    @return none
*/
static void JIG_TEST_mavlink_gimbal_control_motor(mavlink_channel_t channel, control_gimbal_motor_t type)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command            = MAV_CMD_USER_1;
    command_long.param7             = type;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(   SYSID_ONBOARD,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}


/** @brief mavlink_gimbal_move
    @return none
*/
static void JIG_TEST_mavlink_gimbal_move(mavlink_channel_t channel, int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command            = MAV_CMD_DO_MOUNT_CONTROL;
    command_long.param1             = tilt;
    command_long.param2             = roll;
    command_long.param3             = pan;
    command_long.param4             = 0;
    command_long.param5             = 0;
    command_long.param6             = rotationMode;
    command_long.param7             = MAV_MOUNT_MODE_MAVLINK_TARGETING;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(   SYSID_ONBOARD,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_mode
    @return none
*/
static void JIG_TEST_mavlink_gimbal_mode(mavlink_channel_t channel, control_gimbal_mode_t mode)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command            = MAV_CMD_USER_2;
    command_long.param7             = mode;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    channel,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}


/** @brief mavlink_gimbal_home
    @return none
*/
static void JIG_TEST_mavlink_gimbal_home(mavlink_channel_t channel)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command            = MAV_CMD_USER_2;
    command_long.param6             = 3;

    command_long.param7             = 0x04;

    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    command_long.target_system      = MAV_COMP_ID_SYSTEM_CONTROL;
    
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(   SYSID_ONBOARD,
                                            MAV_COMP_ID_SYSTEM_CONTROL,
                                            channel,
                                            &msg,
                                            &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_mode
    @return none
*/
static void JIG_TEST_mavlink_gimbal_remoteControl(mavlink_channel_t channel, remote_control_gimbal_t command)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    
    command_long.command            = MAV_CMD_DO_MOUNT_CONFIGURE;
    command_long.param1             = command;

    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    channel,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    
    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}


/** @brief mavlink_gimbal_reboot
    @return none
*/
static void JIG_TEST_mavlink_gimbal_reboot(mavlink_channel_t channel)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;

    uint8_t systemid    = SYSID_ONBOARD;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = ONBOARD_CHANNEL;
    
    // Reverse tilt and Pan with the Pixhawk
    command_long.command            = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    command_long.param1             = 0;
    command_long.param2             = 0;
    command_long.param3             = 0;
    command_long.param4             = 1;
    command_long.param5             = 0;
    command_long.param6             = 0;
    command_long.param7             = 0;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    
    if(len > 0)
    {
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/** @brief mavlink_gimbal_message_reciver_handle
    @return none
*/
static void JIG_TEST_mavlink_gimbal_message_reciver_handle(mav_state_t *mav, JIG_TEST_mavlink_gimbal_t *gimbal_channel)
{
    switch(mav->rxmsg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&mav->rxmsg, &heartbeat);
            
            /// lay gia tri heartbeat vao struct jig test gimbal
            if(gimbal_channel->seen_heartbeat == false)
            {
                gimbal_channel->seen_heartbeat = true;
            }
            
            gimbal_channel->vehicle_component_id = mav->rxmsg.compid;
            gimbal_channel->vehicle_system_id = mav->rxmsg.sysid;
            gimbal_channel->system_type = heartbeat.type;
            
            gimbal_channel->heartbeat.autopilot         = heartbeat.autopilot;
            gimbal_channel->heartbeat.base_mode         = heartbeat.base_mode;
            gimbal_channel->heartbeat.custom_mode       = heartbeat.custom_mode;
            gimbal_channel->heartbeat.mavlink_version   = heartbeat.mavlink_version;
            gimbal_channel->heartbeat.system_status     = heartbeat.system_status;
            gimbal_channel->heartbeat.type              = heartbeat.type;
            
            gimbal_channel->heartbeat.flag_heartbeat    = true;
            
            if(mav->rxmsg.sysid == 0x01)
            {
                char buff[100];
                sprintf(buff, "[reciever heartbeat esp32] mode : %d | type : %d\n", mavlink_comm_rapberry.heartbeat.base_mode, mavlink_comm_rapberry.heartbeat.type);
                JIG_TEST_console_write(buff);
            }
            
//            HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin | blue_Pin | green_Pin);
//            gimbal_channel->heartbeat.system_status_send ++;
            
        }break;
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t              packet;
            mavlink_msg_sys_status_decode(&mav->rxmsg, &packet);
            
            /* Get voltage battery*/
            gimbal_channel->status.voltage_battery   = packet.voltage_battery;
            
            gimbal_channel->status.load              = packet.load;

            /* Check gimbal's motor */
            if(packet.errors_count1 & 0x10)
            {
                gimbal_channel->status.state = GIMBAL_STATE_ON;
                
                /* Check gimbal is follow mode*/
                if(packet.errors_count1 & 0x01)
                {
                    gimbal_channel->status.mode = GIMBAL_STATE_FOLLOW_MODE;
                }
                else
                {
                    gimbal_channel->status.mode = GIMBAL_STATE_LOCK_MODE;
                }
            }
            /* Check gimbal is initializing*/
            else if(packet.errors_count1 & 0x20)
            {
                gimbal_channel->status.state = GIMBAL_STATE_INIT;
            }
            else if(packet.errors_count1 & 0x04)
            {
                /* Check gimbal is error state*/
                gimbal_channel->status.state = GIMBAL_STATE_ERROR;
            }
            else if(!(packet.errors_count1 & 0x00))
            {
                gimbal_channel->status.state    = GIMBAL_STATE_OFF;
                gimbal_channel->status.mode     = GIMBAL_STATE_OFF;
            }
            else if((packet.errors_count1 & GIMBAL_STATE_MAPPING_MODE) == GIMBAL_STATE_MAPPING_MODE)
            {
                gimbal_channel->status.state    = GIMBAL_STATE_MAPPING_MODE;
            }
            
            /* Check gimbal's sensor status */
            if(packet.errors_count2 & 0x01)
            {
                gimbal_channel->status.sensor |= SENSOR_IMU_ERROR;
            }
            if(packet.errors_count2 & 0x02)
            {
                gimbal_channel->status.sensor |= SENSOR_EN_TILT;
            }
            if(packet.errors_count2 & 0x04)
            {
                gimbal_channel->status.sensor |= SENSOR_EN_ROLL;
            }
            if(packet.errors_count2 & 0x08)
            {
                gimbal_channel->status.sensor |= SENSOR_EN_PAN;
            }
            else 
            {
                gimbal_channel->status.sensor = SENSOR_OK;
            }
            
            /// kiem tra gimbal StartUp calib

            if((packet.errors_count1 & GIMBAL_STATE_SENSOR_CALIB) == GIMBAL_STATE_SENSOR_CALIB)
            {
                gimbal_channel->status.startUpCalib_running = 1;
            }
            else
            {
                gimbal_channel->status.startUpCalib_running = 2;
            }
            
            if((packet.errors_count1 & 0x20) == 0x20)
            {
                gimbal_channel->status.first_calib_motor = 1;
            }
            else
            {
                gimbal_channel->status.first_calib_motor = 2;
            }
            
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            gimbal_channel->status.seq = chan_status->current_rx_seq;
        }break;
        case MAVLINK_MSG_ID_MOUNT_STATUS:
        {
            mavlink_mount_status_t packet = {0};
            mavlink_msg_mount_status_decode(&mav->rxmsg, &packet);
            
            /* Get encoder value */
            gimbal_channel->encoder_val.pitch   = packet.pointing_a;
            gimbal_channel->encoder_val.roll    = packet.pointing_b;
            gimbal_channel->encoder_val.yaw     = packet.pointing_c;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            gimbal_channel->encoder_val.seq = chan_status->current_rx_seq;
        }break;
        case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
        {
            mavlink_mount_orientation_t          packet = {0};
            mavlink_msg_mount_orientation_decode(&mav->rxmsg, &packet);
            
            /* Get attitude value */
            gimbal_channel->mount_val.time_boot_ms   = packet.time_boot_ms;
            gimbal_channel->mount_val.pitch          = packet.pitch;
            gimbal_channel->mount_val.roll           = packet.roll;
            gimbal_channel->mount_val.yaw            = packet.yaw;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            gimbal_channel->mount_val.seq = chan_status->current_rx_seq;
        }break;
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_attitude_t          packet = {0};
            mavlink_msg_attitude_decode(&mav->rxmsg, &packet);
            
            /* Get attitude value */
            gimbal_channel->attitude.pitch          = packet.pitch * PI2ANGLE;
            gimbal_channel->attitude.roll           = packet.roll * PI2ANGLE;
            gimbal_channel->attitude.yaw            = packet.yaw * PI2ANGLE;
            gimbal_channel->attitude.pitchspeed     = packet.pitchspeed * PI2ANGLE;
            gimbal_channel->attitude.rollspeed      = packet.rollspeed * PI2ANGLE;
            gimbal_channel->attitude.yawspeed       = packet.yawspeed * PI2ANGLE;
            gimbal_channel->attitude.time_boot_ms   = packet.time_boot_ms;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            gimbal_channel->attitude.seq = chan_status->current_rx_seq;
        }break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            mavlink_param_value_t          packet = {0};
            mavlink_msg_param_value_decode(&mav->rxmsg, &packet);
            
            for(uint8_t i = 0; i < strlen(gimbal_channel->param_value.param_id); i++)
            {
                gimbal_channel->param_value.param_id[i] = 0;
            }
            
            gimbal_channel->param_value.param_count = packet.param_count;
            memcpy(gimbal_channel->param_value.param_id, packet.param_id, strlen(packet.param_id));

            gimbal_channel->param_value.param_index = packet.param_index;
            gimbal_channel->param_value.param_type = packet.param_type;
            gimbal_channel->param_value.param_value = packet.param_value;
        }break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t         packet = {0};
            mavlink_msg_command_ack_decode(&mav->rxmsg, &packet);
            
            gimbal_channel->ack.command = packet.command;
            gimbal_channel->ack.progress = packet.progress;
            gimbal_channel->ack.result = packet.result;
        }break;
        case MAVLINK_MSG_ID_RAW_IMU:
        {
            mavlink_raw_imu_t         packet = {0};
            mavlink_msg_raw_imu_decode(&mav->rxmsg, &packet);
            
            gimbal_channel->raw_imu.time_usec = packet.time_usec;
            gimbal_channel->raw_imu.xacc = packet.xacc;
            gimbal_channel->raw_imu.xgyro = packet.xgyro;
            gimbal_channel->raw_imu.xmag = packet.xmag;
            gimbal_channel->raw_imu.yacc = packet.yacc;
            gimbal_channel->raw_imu.ygyro = packet.ygyro;
            gimbal_channel->raw_imu.ymag = packet.ymag;
            gimbal_channel->raw_imu.zacc = packet.zacc;
            gimbal_channel->raw_imu.zgyro = packet.zgyro;
            gimbal_channel->raw_imu.zmag = packet.zmag;
            
            gimbal_channel->raw_imu.id = packet.id;
            
            gimbal_channel->raw_imu.flag_raw_imu_message = true;
            
        }break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_param_request_read_t        packet = {0};
            mavlink_msg_param_request_read_decode(&mav->rxmsg, &packet);
            
            mav_array_memcpy(gimbal_channel->param_request_read.param_id, packet.param_id, sizeof(char) * 16);
            
            gimbal_channel->param_request_read.param_index      = packet.param_index;
            gimbal_channel->param_request_read.target_component = packet.target_component;
            gimbal_channel->param_request_read.target_system    = packet.target_system;
        }break;

        default:
        {
        
        }break;
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_GIMBAL_CONTROL_COM2
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_CONTROL_COM2
#define JIG_TEST_MAVLINK_GIMBAL_CONTROL_COM2

/** @brief mavlink_gimbal_set_param
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_param(float param_value, char *param_id)
{
    JIG_TEST_mavlink_gimbal_send_param(MAVLINK_COMM_1, param_value, param_id);
}

/** @brief mavlink_gimbal_send_param_request_read
    @return none
*/
void JIG_TEST_mavlink_gimbal_send_param_request_read(int16_t param_index, char *param_id)
{
    JIG_TEST_mavlink_gimbal_param_request_read(MAVLINK_COMM_1, param_index, param_id);
}

/** @brief mavlink_gimbal_set_control_motor
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_control_motor(control_gimbal_motor_t type)
{
    JIG_TEST_mavlink_gimbal_control_motor(MAVLINK_COMM_1, type);
}

/** @brief mavlink_gimbal_set_move
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode)
{
    JIG_TEST_mavlink_gimbal_move(MAVLINK_COMM_1, tilt, roll, pan, rotationMode);
}

/** @brief mavlink_gimbal_com4_set_move
    @return none
*/
void JIG_TEST_mavlink_gimbal_com4_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode)
{
    JIG_TEST_mavlink_gimbal_move(MAVLINK_COMM_2, tilt, roll, pan, rotationMode);
}

/** @brief mavlink_gimbal_set_mode
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_mode(control_gimbal_mode_t mode)
{
    JIG_TEST_mavlink_gimbal_mode(MAVLINK_COMM_1, mode);
}

/** @brief mavlink_gimbal_set_mode
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_home(void)
{
    JIG_TEST_mavlink_gimbal_home(MAVLINK_COMM_1);
}

/** @brief mavlink_gimbal_set_rc_input
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_rc_input(remote_control_gimbal_t command)
{
    JIG_TEST_mavlink_gimbal_remoteControl(MAVLINK_COMM_1, command);
}

/** @brief mavlink_gimbal_com4_set_rc_input
    @return none
*/
void JIG_TEST_mavlink_gimbal_com4_set_rc_input(remote_control_gimbal_t command)
{
    JIG_TEST_mavlink_gimbal_remoteControl(MAVLINK_COMM_2, command);
}

/** @brief mavlink_gimbal_set_reboot
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_reboot(void)
{
    JIG_TEST_mavlink_gimbal_reboot(MAVLINK_COMM_1);
}

/** @brief get_state_calib_motor
    @return none
*/
uint8_t JIG_TEST_mavlink_gimbal_get_state_calib_motor(void)
{
    return mavlink_gimbal_COM2.status.first_calib_motor;
}

/** @brief get_state_calib_imu
    @return none
*/
uint8_t JIG_TEST_mavlink_gimbal_get_state_calib_imu(void)
{
    return mavlink_gimbal_COM2.status.startUpCalib_running;
}

/** @brief mavlink_gimbal_get_sensor_name
    @return JIG_TEST_mavlink_gimbal_sensor_check_t
*/
JIG_TEST_mavlink_gimbal_sensor_check_t JIG_TEST_mavlink_gimbal_get_sensor_name(JIG_TEST_mavlink_gimbal_t* mavlinkChannel)
{
    JIG_TEST_mavlink_gimbal_sensor_check_t sensor = GREMSY_SENSOR_ERROR;
    
    if(mavlinkChannel->raw_imu.id == 1)
    {
        sensor = GREMSY_SENSOR_BMI160;
    }
    else if(mavlinkChannel->raw_imu.id == 2)
    {
        sensor = GREMSY_SENSOR_ICM42688;
    }
    
    return sensor;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_COMMUNICATION_RASPBERRY
    @{
*/#ifndef JIG_TEST_MAVLINK_COMMUNICATION_RASPBERRY
#define JIG_TEST_MAVLINK_COMMUNICATION_RASPBERRY

#define JIG_TEST_USB_SPEED_MSG_READ_SPEED       0
#define JIG_TEST_USB_SPEED_MSG_WRITE_SPEED      1
#define JIG_TEST_USB_SPEED_MSG_RESULT           2
#define JIG_TEST_USB_SPEED_MSG_STATUS           3
#define JIG_TEST_USB_SPEED_MSG_START            4
#define JIG_TEST_USB_SPEED_MSG_END              5
#define JIG_TEST_USB_SPEED_MSG_REF_READ_SPEED   6
#define JIG_TEST_USB_SPEED_MSG_REF_WRITE_SPEED  7

/** @brief mavlink_comm_raspberry_send_heartbeat
    @return none
*/
void JIG_TEST_mavlink_comm_raspberry_send_heartbeat(void)
{
    JIG_TEST_mavlink_gimbal_send_heartbeat(MAVLINK_COMM_3);
}

/** @brief mavlink_comm_raspberry_send_start
    @return none
*/
static void JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(uint8_t param_index)
{
    JIG_TEST_mavlink_gimbal_param_request_read(MAVLINK_COMM_3, param_index, NULL);
}

/** @brief mavlink_comm_raspberry_send_start
    @return none
*/
static void JIG_TEST_mavlink_comm_raspberry_send_param_value(int16_t param_value)
{
    JIG_TEST_mavlink_gimbal_param_value(MAVLINK_COMM_3
                                        , mavlink_comm_rapberry.param_request_read.param_index
                                        , param_value
                                        , mavlink_comm_rapberry.param_request_read.param_id);
}

/** @brief mavlink_comm_raspberry_sendData
    @return none
*/
static void JIG_TEST_mavlink_comm_raspberry_sendData(void)
{
    /// send end test usb (chi send 1 lan)
    if(raspberry_msg.enable_send_end == true)
    {
        raspberry_msg.enable_send_end = false;
        
        JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_END);
    }
    
    /// send start test usb (chi send 1 lan)
    if(raspberry_msg.enable_send_start == true)
    {
        raspberry_msg.enable_send_start = false;
        
        /// set flag send request status
        raspberry_msg.enable_send_request_state = true;
        
        JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_START);
    }
    
//    /// send msg request status to Pi moi 1000ms mot lan
//    if(get_timeOut(1000, JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_STATE))
//    {
//        if(raspberry_msg.enable_send_request_state == true)
//        {
//            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_STATUS);
//        }
//    }
    
    /// send msg request result to Pi moi 200ms mot lan
    if(get_timeOut(200, JIG_TEST_COMM_RASPBERRY_SEND_REQUEST_RESULT))
    {
        /// send msg request read speed
        if(raspberry_msg.enable_send_request_read_speed == true)
        {
            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_READ_SPEED);
        }
        
        /// send msg request write speed
        if(raspberry_msg.enable_send_request_write_speed == true)
        {
            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_WRITE_SPEED);
        }
        
        /// send msg request ref read speed
        if(raspberry_msg.enable_send_request_ref_read_speed == true)
        {
            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_REF_READ_SPEED);
        }
        
        /// send msg request ref write speed
        if(raspberry_msg.enable_send_request_ref_write_speed == true)
        {
            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_REF_WRITE_SPEED);
        }
        
        /// send msg request result
        if(raspberry_msg.enable_send_request_result == true)
        {
            JIG_TEST_mavlink_comm_raspberry_send_request_param_read_msg(JIG_TEST_USB_SPEED_MSG_RESULT);
        }
//        HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
    }
    
    if(raspberry_msg.enable.send_param_value == true)
    {
        raspberry_msg.enable.send_param_value = false;
        
        JIG_TEST_mavlink_comm_raspberry_send_param_value(raspberry_msg.enable.param_value);
        
        /// xoa param index de cho raspberry request tiep
        mavlink_comm_rapberry.param_request_read.param_index = 0;
    }
    
}

///** @brief mavlink_gimbal_process
//    @return none
//*/
//uint8_t JIG_TEST_mavlink_gimbal_get_heartbeat_custom

#endif
/**
    @}
*/

/** @group JIG_TEST_MAVLINK_GIMBAL_PROCESS
    @{
*/#ifndef JIG_TEST_MAVLINK_GIMBAL_PROCESS
#define JIG_TEST_MAVLINK_GIMBAL_PROCESS

/** @brief mavlink_gimbal_process
    @return none
*/
void JIG_TEST_mavlink_gimbal_process(void)
{
    /// send data to gimbal
    JIG_TEST_mavlink_gimbal_sendData();
    
    /// read Data from gimbal
    JIG_TEST_mavlink_gimbal_readData();
    
    /// send data to raspberry
    JIG_TEST_mavlink_comm_raspberry_sendData();
}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


