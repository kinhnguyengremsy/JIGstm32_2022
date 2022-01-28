/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gGimbal.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains expand of gMavlink
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
/* Exported Define------------------------------------------------------------*/

typedef enum
{
    SET_PARAM_STATE_JIG_TEST,
    SET_PARAM_STATE_CHECK_NOISE,
    SET_PARAM_STATE_GET_PARAM_PROCESS,
    SET_PARAM_STATE_GET_PARAM_DONE
    
}set_param_gimbal_state_t;

typedef enum
{
    STANBY = 0x00,
    RUNNING,
    DONE,
    
}usb_speedTest_status_t;

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
 * @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _control_gimbal_mode_t
{
    LOCK_MODE   = 1,
    FOLLOW_MODE = 2,
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

/**
 * @brief gimbal_state_t
 * State of gimbal
 */
typedef enum _gimbal_state
{
    GIMBAL_STATE_OFF            = 0x00,     /*< Gimbal is off*/
    GIMBAL_STATE_INIT           = 0x01,     /*< Gimbal is initializing*/
    GIMBAL_STATE_ON             = 0x02,     /*< Gimbal is on */
    GIMBAL_STATE_LOCK_MODE      = 0x04,     
    GIMBAL_STATE_FOLLOW_MODE    = 0x08,
    GIMBAL_STATE_SEARCH_HOME    = 0x10,
    GIMBAL_STATE_SET_HOME       = 0x20,
    GIMBAL_STATE_ERROR          = 0x40,
    GIMBAL_STATE_SENSOR_CALIB  = 0x400,
    
} gimbal_state_t;

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
    PAN_SEARCH_HOME_ERROR        = 0x01,     /* Gimbal's sensor is healthy */
    ANGLE_TILT_ERROR             = 0x02,     /* angle tilt is error at tilt axis*/
    ANGLE_ROLL_ERROR             = 0x03,     /* angle roll is error at roll axis*/
    ANGLE_PAN_ERROR              = 0x04,     /* angle pan is error at roll axis*/

}gimbal_angle_state;

/**
 * @brief gimbal_move_state_t
 * State of gimbal's angle
 */
typedef enum _gimbal_move_state
{
    MOVE_TILT_ERROR        = 0x01,     /* Gimbal's sensor is healthy */
    MOVE_ROLL_ERROR        = 0x02,     /* move tilt is error at tilt axis*/
    MOVE_PAN_ERROR         = 0x03,     /* move roll is error at roll axis*/
    MOTOR_FRAME_ERROR      = 0x04,     /* move pan is error at roll axis*/

}gimbal_move_state;


typedef enum _version
{
    VERSION_ALPHA   = 0x01,
    VERSION_BETA    = 0x02,
    VERSION_PREVIEW = 0x03,
    VERSION_OFFICAL = 0x00,
} _version_t;


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
    bool        startUpCalib_running;
    bool        startUpCalib_Done;
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

typedef struct
{
    float valueWrite;
    float valueRead;
    float ref_valueWrite;
    float ref_valueRead;
    uint8_t result;
    
    bool is_sendRequest;
    bool is_DoneTest;
    bool is_send_runTest;
    bool is_send_End;
    usb_speedTest_status_t status;
}usb_speedTest_t;

typedef struct mavlink_gimbal_debug_t 
{
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float value; /*<  DEBUG value*/
    uint8_t ind; /*<  index of debug variable*/
}mavlink_gimbal_debug_t;

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
}mavlink_gimbal_raw_imu_t;

/**
 * @brief  This struct will contain all information 
 * related to gimbal which sending/received data gTune
 */
typedef struct _gGimbal_t
{
    uint32_t        time_update;    // Time update has been checked 50Hz
    uint32_t        msg_idx;        // Sequense to send message

    uint8_t         seen_heartbeat; // Check heartbeat of the Gimbal has seen
    
    uint8_t         vehicle_system_id;      // gimbal system
    uint8_t         vehicle_component_id;   // Gimbal component
    uint8_t         system_type;            // Type of system: 26
    
    /* callback functions to send to gimbal*/
//    void            (*send_heartbeat)(m);    
    void            (*set_motor)(control_gimbal_motor_t type);
    void            (*set_mode)(control_gimbal_mode_t mode);
    
    /* Function support set control for each axis */
    void            (*set_axis_mode)(control_gimbal_axis_mode_t tilt,
                                     control_gimbal_axis_mode_t roll,
                                     control_gimbal_axis_mode_t pan);
    
    /*input that degree or degree/s */
    void            (*set_move)(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode); 
    
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
    
    /* gimbal usb SpeedTest */
    usb_speedTest_t usb_speed;
    
    /// bien send param done
    bool is_send_param_Done;
    
    /// count wait heartbeat
    uint8_t heartbeatTimeOut_count;
    
    /// flag timeOut heartbeat
    bool heartbeatTimeOut;
    
    /// send count
    uint8_t moveRight, moveLeft;
    
    /// count send param
    set_param_gimbal_state_t param_gimbalCount;
    
    /* gimbal debug */
    mavlink_gimbal_debug_t gimbalDebug;
    
    /* raw imu */
    mavlink_gimbal_raw_imu_t raw_imu;
    
    /* Firmware version of gimbal*/
    uint16_t            firmware_version;
}gGimbal_t;

/**
 * @brief  This is function initialize COM platform, protocol
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gGremsy_init(void);

/**
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gGremsy_read_data(void);

/**
 * @brief  This is function send data to gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gGremsy_send_data(void);

/**
 * @brief  The function will run example 
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gGimbal_display(void);

void gimbal_set_home(void);
void gGimbal_control(void);
void gGimbal_AUX_test(void);
void gGimbal_Console(uint8_t *str);
void GREMSY_AC30000_JIG_TEST_aux_test_process(void);
void GREMSY_AC30000_JIG_TEST_aux_read_Callback(uint16_t GPIO_Pin);
uint32_t gSystick_timebase_get(uint32_t* lastTime);
void gSystick_timebase_reset(uint32_t* lastTime);
void GREMSY_AC30000_JIG_TEST_usb_speedTest_process(void);
bool GREMSY_AC30000_JIG_TEST_send_usb_runTest(bool enable);
bool GREMSY_AC30000_JIG_TEST_send_endTest(bool enable);
void GREMSY_AC30000_JIG_TEST_send_param_gimbal(void);
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE****/
