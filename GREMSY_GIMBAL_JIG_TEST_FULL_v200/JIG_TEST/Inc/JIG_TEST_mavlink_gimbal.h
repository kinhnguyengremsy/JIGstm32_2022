/** 
  ******************************************************************************
  * @file    JIG_TEST_mavlink_gimbal.h
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

#ifndef __JIG_TEST_MAVLINK_GIMBAL_H
#define __JIG_TEST_MAVLINK_GIMBAL_H

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
/**
 * @brief Gimbal rotation mode, specifies control style.
 */
typedef enum {
    GIMBAL_ROTATION_MODE_RELATIVE_ANGLE = 0, /*!< Relative angle rotation mode, represents rotating gimbal specified angles based on current angles. */
    GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE = 1, /*!< Absolute angle rotation mode, represents rotating gimbal to specified angles in the ground coordinate. */
    GIMBAL_ROTATION_MODE_SPEED = 2, /*!< Speed rotation mode, specifies rotation speed of gimbal in the ground coordinate. */
} E_GimbalRotationMode;

/**
 * @brief control_motor_t
 * Command control motor is on/off
 */
typedef enum _control_gimbal_motor_t
{
    TURN_OFF    = 0,
    TURN_ON     = 1
} control_gimbal_motor_t;

/**
 * @brief remote_control_gimbal_t
 * Command remote control gimbal
 */
typedef enum _remote_control_gimbal_t
{
    MAVLINK_CONTROL_MODE       = 2,
    REMOTE_CONTROL_MODE        = 3 //MAV_MOUNT_MODE_RC_TARGETING
} remote_control_gimbal_t;

/**
 * @brief gimbal_state_t
 * State of gimbal
 */
typedef enum _gimbal_state
{
    GIMBAL_STATE_OFF            = 0x00,     /*< Gimbal is off*/
    GIMBAL_STATE_INIT           = 0x01,     /*< Gimbal is initializing*/
    GIMBAL_STATE_ON             = 0x02,     /*< Gimbal is on */
    GIMBAL_STATE_MAPPING_MODE   = 0x03,
    GIMBAL_STATE_LOCK_MODE      = 0x04,     
    GIMBAL_STATE_FOLLOW_MODE    = 0x08,
    GIMBAL_STATE_SEARCH_HOME    = 0x10,
    GIMBAL_STATE_SET_HOME       = 0x20,
    GIMBAL_STATE_ERROR          = 0x40,
    GIMBAL_STATE_SENSOR_CALIB  = 0x400,
    
} gimbal_state_t;

/**
 * @brief gimbal_state_error_01_t
 * Error of gimbal
 */
typedef enum _gimbal_state_error_01_t
{
    GIMBAL_STATUS_SENSOR_ERROR         = 0x04,
    GIMBAL_STATUS_STARTUP_ERROR        = 0x800,
    GIMBAL_STATUS_MOTOR_PHASE_ERROR    = 0x4000,
    GIMBAL_STATUS_MOTOR_ANGLE_ERROR    = 0x8000,
    
}gimbal_state_error_01_t;

/**
 * @brief gimbal_state_error_02_t
 * Error of gimbal
 */
typedef enum _gimbal_state_error_02_t
{
    GIMBAL_STATUS_IMU_ERROR                = 0x01,
    
    GIMBAL_STATUS_INVERTED_ERROR           = 0x10,
    GIMBAL_STATUS_PAN_SEARCH_HOME_ERROR    = 0x20,
    
    GIMBAL_STATUS_ANGLE_TILT_ERROR         = 0x80,
    GIMBAL_STATUS_ANGLE_ROLL_ERROR         = 0x100,
    GIMBAL_STATUS_ANGLE_PAN_ERROR          = 0x200,
    
    GIMBAL_STATUS_MOVE_TILT_ERROR          = 0x400,
    GIMBAL_STATUS_MOVE_ROLL_ERROR          = 0x800,
    GIMBAL_STATUS_MOVE_PAN_ERROR           = 0x1000,
    
    GIMBAL_STATUS_MOTOR_FRAME_ERROR        = 0x2000,
    
}gimbal_state_error_02_t;

/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef enum _sensor_state
{
    SENSOR_OK                   = 0x00,     /* Gimbal's sensor is healthy */
    SENSOR_IMU_ERROR            = 0x01,     /* IMU error*/
    SENSOR_EN_TILT              = 0x02,     /* Encoder sensor is error at tilt axis*/
    SENSOR_EN_ROLL              = 0x03,     /* Encoder sensor is error at roll axis*/
    SENSOR_EN_PAN               = 0x04,     /* Encoder sensor is error at pan axis*/
}sensor_state_;

/**
 * @brief gimbal_angle_state_t
 * State of gimbal's angle
 */
typedef enum _gimbal_angle_state
{
    GIMBAL_PAN_SEARCH_HOME_ERROR        = 0x01,     /* Gimbal's sensor is healthy */
    GIMBAL_ANGLE_TILT_ERROR             = 0x02,     /* angle tilt is error at tilt axis*/
    GIMBAL_ANGLE_ROLL_ERROR             = 0x03,     /* angle roll is error at roll axis*/
    GIMBAL_ANGLE_PAN_ERROR              = 0x04,     /* angle pan is error at roll axis*/

}gimbal_angle_state;

/**
 * @brief gimbal_motor_error_t
 * State of gimbal's angle
 */
typedef enum _gimbal_motor_error_t
{
    GIMBAL_MOTOR_TILT_ERROR        = 0x01,     /* Gimbal's sensor is healthy */
    GIMBAL_MOTOR_ROLL_ERROR        = 0x02,     /* move tilt is error at tilt axis*/
    GIMBAL_MOTOR_PAN_ERROR         = 0x03,     /* move roll is error at roll axis*/
    GIMBAL_MOTOR_FRAME_ERROR      = 0x04,     /* move pan is error at roll axis*/

}gimbal_motor_error_t;

/**
 * @brief gimbal_move_state_t
 * State of gimbal's angle
 */
typedef enum _gimbal_move_state
{
    GIMBAL_MOVE_TILT_ERROR        = 0x01,     /* Gimbal's sensor is healthy */
    GIMBAL_MOVE_ROLL_ERROR        = 0x02,     /* move tilt is error at tilt axis*/
    GIMBAL_MOVE_PAN_ERROR         = 0x03,     /* move roll is error at roll axis*/

}gimbal_move_state;

/**
 * @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _control_gimbal_mode_t
{
    LOCK_MODE   = 1,
    FOLLOW_MODE = 2,
    MAPPING_MODE = 3,
} control_gimbal_mode_t;

/**
 * @brief _control_gimbal_axis_input_mode
 * Command control gimbal input mode for each axis
 */
typedef enum _control_gimbal_axis_input_mode
{
    CTRL_ANGLE_BODY_FRAME       = 0,
    CTRL_ANGULAR_RATE           = 1,
    CTRL_ANGLE_ABSOLUTE_FRAME   = 2,
} control_gimbal_axis_input_mode_t;

// dinh nghia cac loai imu duoc su dung
typedef enum
{
    GREMSY_SENSOR_ERROR = 0,
    GREMSY_SENSOR_BMI160,
    GREMSY_SENSOR_ICM42688,
    
}JIG_TEST_mavlink_gimbal_sensor_check_t;

/**
 * @brief _control_gimbal_axis_mode_t
 * Command control gimbal for each axis
 */
typedef struct _control_gimbal_axis_mode_t
{
    /* stabilize? (1 = yes, 0 = no)*/
    uint8_t stabilize;   
    
    control_gimbal_axis_input_mode_t    input_mode;
    
}control_gimbal_axis_mode_t;

/**
 * @brief gimbal_state_t
 * State of gimbal's sensor
 */
typedef struct _gimbal_status_t
{
    uint16_t    load; /*< [ms] Maximum usage the mainloop time. Values: [0-1000] - should always be below 1000*/
    uint16_t    voltage_battery; /*< [V] Battery voltage*/
    uint8_t     sensor; /*< Specific sensor occur error (encorder, imu) refer sensor_state_*/
    uint16_t    state;  /* System state of gimbal. Refer gimbal_state_t*/
    uint8_t     mode;   /*< Gimbal mode is running*/
    uint32_t    seq;
    uint8_t        startUpCalib_running;
    bool        startUpCalib_Done;
    uint8_t        first_calib_motor;
} gimbal_status_t;

/**    
 * @brief  This structure stores axes values
 */

typedef struct _gimbal_value
{
    int16_t        pitch;
    int16_t        roll;
    int16_t        yaw;
    uint8_t        seq;
} gimbal_value_t;

typedef struct _gimbal_attitude
{
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float roll; /*< [deg] Roll in global frame (set to NaN for invalid).*/
    float pitch; /*< [deg] Pitch in global frame (set to NaN for invalid).*/
    float yaw; /*< [deg] Yaw relative to vehicle(set to NaN for invalid).*/
    float yaw_absolute; /*< [deg] Yaw in absolute frame, North is 0 (set to NaN for invalid).*/
    uint8_t        seq;

}gimbal_mount_t;

typedef struct _gimbal_attitude_t 
{
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float   roll; /*< [rad] Roll angle (-pi..+pi)*/
    float   pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    float   yaw; /*< [rad] Yaw angle (-pi..+pi)*/
    float   rollspeed; /*< [rad/s] Roll angular speed*/
    float   pitchspeed; /*< [rad/s] Pitch angular speed*/
    float   yawspeed; /*< [rad/s] Yaw angular speed*/
    uint8_t seq;
} gimbal_attitude_t;

typedef struct param_value_t 
{
    float param_value; /*<  Onboard parameter value*/
    uint16_t param_count; /*<  Total number of onboard parameters*/
    uint16_t param_index; /*<  Index of this onboard parameter*/
    char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
    uint8_t param_type; /*<  Onboard parameter type.*/
    
    uint8_t debug_param_index; /// programmer debug
    int sendMsg_request;
    
    uint8_t param_readCount;
}param_value_t;

typedef struct command_ack_t 
{
    bool enable; /* cho phep nhan cmd ack */
    uint16_t command; /*<  Command ID (of acknowledged command).*/
    uint8_t result; /*<  Result of command.*/
    uint8_t progress; /*<  WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS.*/
    int32_t result_param2; /*<  WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.*/
    uint8_t target_system; /*<  WIP: System which requested the command to be executed*/
    uint8_t target_component; /*<  WIP: Component which requested the command to be executed*/
}command_ack_t;

typedef struct mavlink_gimbal_raw_imu_t 
{
    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
    int16_t xacc; /*<  X acceleration (raw)*/
    int16_t yacc; /*<  Y acceleration (raw)*/
    int16_t zacc; /*<  Z acceleration (raw)*/
    int16_t xgyro; /*<  Angular speed around X axis (raw)*/
    int16_t ygyro; /*<  Angular speed around Y axis (raw)*/
    int16_t zgyro; /*<  Angular speed around Z axis (raw)*/
    int16_t xmag; /*<  X Magnetic field (raw)*/
    int16_t ymag; /*<  Y Magnetic field (raw)*/
    int16_t zmag; /*<  Z Magnetic field (raw)*/
    uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
    
    bool flag_raw_imu_message;
}mavlink_gimbal_raw_imu_t;

typedef struct _mavlink_msg_param_request_read_t 
{
    int16_t param_index; /*<  Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)*/
    uint8_t target_system; /*<  System ID*/
    uint8_t target_component; /*<  Component ID*/
    char param_id[16]; /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
}mavlink_msg_param_request_read_t;

typedef struct _mavlink_msg_heartbeat_t 
{
    bool flag_heartbeat;
    uint16_t count;
    
    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type; /*<  Type of the MAV (quadrotor, helicopter, etc.)*/
    uint8_t autopilot; /*<  Autopilot type / class.*/
    uint8_t base_mode; /*<  System mode bitmap.*/
    uint8_t system_status; /*<  System status flag.*/
    uint8_t system_status_send; /*<  System status flag.*/
    uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
    
}mavlink_msg_heartbeat_t;

/**
 * @brief  This struct will contain all information 
 * related to gimbal which sending/received data gTune
 */
typedef struct _JIG_TEST_mavlink_gimbal_t
{
    uint32_t        time_update;    // Time update has been checked 50Hz
    uint32_t        msg_idx;        // Sequense to send message

    bool         seen_heartbeat; // Check heartbeat of the Gimbal has seen
    
    /* heartbeat */
    mavlink_msg_heartbeat_t heartbeat;
    
    uint8_t         vehicle_system_id;      // gimbal system
    uint8_t         vehicle_component_id;   // Gimbal component
    uint8_t         system_type;            // Type of system: 26
    
    /* System status of gimbal*/
    gimbal_status_t     status; 
    
    /* Encoder count of gimbal*/
    gimbal_value_t      encoder_val;
    
    /* Gimbal mount orientation*/
    gimbal_mount_t      mount_val;
    
    /* Gimbal attitude*/
    gimbal_attitude_t   attitude;
    
    /* param value */
    param_value_t param_value;
    
    /* command ack */
    command_ack_t ack;
    
    /* raw imu */
    mavlink_gimbal_raw_imu_t raw_imu;
    
    /* param request read */
    mavlink_msg_param_request_read_t param_request_read;
    
    /// flag setting mapping mode
    bool is_set_mapping;
    control_gimbal_mode_t gimbal_mode;
    
}JIG_TEST_mavlink_gimbal_t;

typedef struct
{
    bool send_param_value;
    uint16_t param_value;
    bool stop;
    uint8_t back_to_standby;
    
}JIG_TEST_mavlink_comm_raspberry_enable_send_param_value;

typedef struct
{
    bool enable_msg_heartbeat;
    bool enable_send_start;
    bool enable_send_end;
    bool enable_send_request_result;
    bool enable_send_request_state;
    bool enable_send_request_read_speed;
    bool enable_send_request_write_speed;
    bool enable_send_request_ref_read_speed;
    bool enable_send_request_ref_write_speed;
    bool enable_send_request_result_speed;
    
    JIG_TEST_mavlink_comm_raspberry_enable_send_param_value enable;
    
    mavlink_msg_param_request_read_t param_request_read;
    
}JIG_TEST_mavlink_comm_respberry_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/** @brief mavlink_gimbal_configuration
    @return 
*/
void JIG_TEST_mavlink_gimbal_configuration(void);

/** @brief mavlink_gimbal_Reinit
    @return 
*/
void JIG_TEST_mavlink_serialPort3_Reinit(void);

/** @brief mavlink_raspberry_Reinit
    @return 
*/
void JIG_TEST_mavlink_serialPort5_Reinit(void);

/** @brief mavlink_gimbal_process
    @return none
*/
void JIG_TEST_mavlink_gimbal_process(void);

/** @brief mavlink_gimbal_set_param
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_param(float param_value, char *param_id);

/** @brief mavlink_gimbal_send_param_request_read
    @return none
*/
void JIG_TEST_mavlink_gimbal_send_param_request_read(int16_t param_index, char *param_id);

/** @brief mavlink_gimbal_set_control_motor
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_control_motor(control_gimbal_motor_t type);

/** @brief mavlink_gimbal_set_move
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode);

/** @brief mavlink_gimbal_com4_set_move
    @return none
*/
void JIG_TEST_mavlink_gimbal_com4_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode);

/** @brief mavlink_gimbal_set_mode
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_mode(control_gimbal_mode_t mode);

/** @brief mavlink_gimbal_set_mode
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_home(void);

/** @brief mavlink_gimbal_set_rc_input
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_rc_input(remote_control_gimbal_t command);

/** @brief mavlink_gimbal_com4_set_rc_input
    @return none
*/
void JIG_TEST_mavlink_gimbal_com4_set_rc_input(remote_control_gimbal_t command);

/** @brief mavlink_gimbal_set_reboot
    @return none
*/
void JIG_TEST_mavlink_gimbal_set_reboot(void);

/** @brief mavlink_comm_raspberry_send_heartbeat
    @return none
*/
void JIG_TEST_mavlink_comm_raspberry_send_heartbeat(void);

/** @brief get_state_calib_motor
    @return none
*/
uint8_t JIG_TEST_mavlink_gimbal_get_state_calib_motor(void);

/** @brief get_state_calib_imu
    @return none
*/
uint8_t JIG_TEST_mavlink_gimbal_get_state_calib_imu(void);

/** @brief mavlink_gimbal_get_sensor_name
    @return JIG_TEST_mavlink_gimbal_sensor_check_t
*/
JIG_TEST_mavlink_gimbal_sensor_check_t JIG_TEST_mavlink_gimbal_get_sensor_name(JIG_TEST_mavlink_gimbal_t* mavlinkChannel);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_MAVLINK_GIMBAL_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

