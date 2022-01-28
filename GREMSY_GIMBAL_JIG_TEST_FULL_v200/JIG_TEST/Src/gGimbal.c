/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gGimbal.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
 * @brief   This file contains expand of hal_dma
 *
 ******************************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "gGimbal.h"

#include "gProtocol.h"

#include "main.h"

#include "stdlib.h"
#include "timeOut.h"
/* Private define-------------------------------------------------------------*/

#ifndef PI
#define PI                  3.141592654
#endif

#define PI2ANGLE            (180.0/PI)
#define NUMBER_OF_PARAM     24
/* Private Typedef------------------------------------------------------------*/
/* Private variable- ---------------------------------------------------------*/
/*variables interface with gTune*/
gGimbal_t           comm_channel_2;
gGimbal_t           comm_channel_4;
mav_state_t         mav;
mav_state_t         mav1;
protocol_t          proto;
protocol_t          proto1;
extern UART_HandleTypeDef huart1;

uint32_t last_time_send_heartbeat;
#if 1
char buffDbg[200];

#endif

typedef enum _sdk_process_state
{
    STATE_IDLE,
    STATE_GIMBAL_MOVE_RIGHT,
    STATE_GIMBAL_MOVE_LEFT,
    
}sdk_process_state_t; 

typedef enum
{
    
    STATE_READ_SPEED = 0x00,
    STATE_WRITE_SPEED,
    STATE_RESULT,
    STATE_STATUS,
    STATE_START,
    STATE_END,
    STATE_REF_READ,
    STATE_REF_WRITE,
    
}usb_speedTest_resultSpeed_t;

typedef struct 
{
    sdk_process_state_t state;
} sdk_process_t;



static const char g_gremsy_param_name[78][16] = 
{
    "VERSION_X",
    "SERIAL_NUMBER",
    "PITCH_P",//2
    "PITCH_I",//3
    "PITCH_D",//4
    "ROLL_P",//5
    "ROLL_I",//6
    "ROLL_D",//7
    "YAW_P",//8
    "YAW_I",//9
    "YAW_D",//10
    "PITCH_POWER",//11
    "ROLL_POWER",//12
    "YAW_POWER",//13
    "PITCH_FOLLOW",
    "ROLL_FOLLOW",
    "YAW_FOLLOW",
    "PITCH_FILTER",
    "ROLL_FILTER",
    "YAW_FILTER",
    "GYRO_TRUST",
    "NPOLES_PITCH",//21
    "NPOLES_ROLL",//22
    "NPOLES_YAW",//23
    "DIR_MOTOR_PITCH",
    "DIR_MOTOR_ROLL",
    "DIR_MOTOR_YAW",
    "MOTOR_FREQ",
    "RADIO_TYPE",  // 28
    "GYRO_LPF",
    "TRAVEL_MIN_PIT",
    "TRAVEL_MAX_PIT",
    "TRAVEL_MIN_ROLL",
    "TRAVEL_MAX_ROLL",
    "TRAVEL_MIN_YAW",
    "TRAVEL_MAX_YAW",
    "RC_PITCH_LPF",
    "RC_ROLL_LPF",
    "RC_YAW_LPF",
    "SBUS_PITCH_CHAN",//39
    "SBUS_ROLL_CHAN",//40
    "SBUS_YAW_CHAN",//41
    "SBUS_MODE_CHAN",//42
    "ACCX_OFFSET",
    "ACCY_OFFSET",
    "ACCZ_OFFSET",
    "GYROX_OFFSET",
    "GYROY_OFFSET",
    "GYROZ_OFFSET",
    "USE_GPS",
    "SKIP_GYRO_CALIB", // 50
    "RC_PITCH_TRIM",
    "RC_ROLL_TRIM",
    "RC_YAW_TRIM",
    "RC_PITCH_MODE",
    "RC_ROLL_MODE",
    "RC_YAW_MODE",
    "TILT_WINDOW",
    "PAN_WINDOW",
    "SBUS_FUNC_CHAN",
    "RC_PITCH_SPEED",//60
    "RC_ROLL_SPEED",//61
    "RC_YAW_SPEED",//62
    "JOY_AXIS",//63
    "TRIM_HOZ",
    "TRIM_TILT",
    "TRIM_PAN",
    "VERSION_Y",
    "VERSION_Z",
    "TRAVEL_MIN_PAN",
    "TRAVEL_MAX_PAN",
    "GIMBAL_OVAL",
    "HEARTBEAT_EMIT", 
    "STATUS_RATE",
    "ENC_CNT_RATE",
    "ENC_TYPE_SEND",
    "ORIEN_RATE",
    "IMU_RATE",
};

struct param_gimbal_set
{
    uint16_t value;
    uint16_t value_param_get;
    uint8_t index;
    char* param_id;
}param_gimbal[NUMBER_OF_PARAM] =
{
    {.value = 3, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 9, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
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
    {.value = 2, .index = 29, .param_id = "GYRO_LPF"}, // gyro filter
    {.value = 4, .index = 9, .param_id = "YAW_I"},
    {.value = 120, .index = 3, .param_id = "PITCH_I"},
    {.value = 210, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 15, .index = 77, .param_id = "IMU_RATE"},
};

sdk_process_t sdk;
uint32_t            last_time_send;
uint32_t            last_time_control;
uint32_t            last_time_display;
bool gimbal_control_result;
bool buttonFeedbackIMU;
uint8_t feedBackIMUCount;
uint8_t value_param_get;
/* Private function- ---------------------------------------------------------*/

/* gimbal send msg */
void send_heartbeat(mavlink_channel_t channel);
void send_param(void);
void send_raw_imu(void);
void send_attitude(void);
void send_system_status(void);
void send_rc_channel(void);
void send_auth_key(void);
void send_encoder_values(void);
void send_req_data_stream_attitude(void);
void send_debug_values(void);

void control_gimbal_motor(control_gimbal_motor_t type);
void control_gimbal_mode(control_gimbal_mode_t type);
void control_gimbal_axis_mode(control_gimbal_axis_mode_t tilt,
                                control_gimbal_axis_mode_t roll,
                                control_gimbal_axis_mode_t pan);

void control_gimbal_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode);
void send_request_param_read(mavlink_channel_t channel, char* param_id, uint8_t param_index);
void send_param_gimbal(uint16_t param_value, mavlink_channel_t channel, char* param_id);
void control_gimbal_set_Reboot(mavlink_channel_t channel);
void send_paramRequest_read(mavlink_channel_t channel, uint8_t param_index, char* param_id);

void handle_message(mav_state_t* mav, gGimbal_t *comm_channel);
/* Exported functions --------------------------------------------------------*/


uint32_t systickTimeWhile[256];

uint32_t gSystick_time_get(uint32_t* lastTime)
{
    uint32_t time = HAL_GetTick() - *lastTime;
    *lastTime = HAL_GetTick();

    return time;
}

void gSystick_timebase_reset(uint32_t* lastTime)
{
    *lastTime = HAL_GetTick();
}

uint32_t gSystick_timebase_get(uint32_t* lastTime)
{
    uint32_t time = HAL_GetTick() - *lastTime;
    return time;
}

bool gSystick_time_while(uint16_t time, uint8_t task)
{
    uint32_t temp = HAL_GetTick() - systickTimeWhile[task];

    if(temp >= time){
        systickTimeWhile[task] = HAL_GetTick();
        
        return true;
    }

    return false;
}

/**
 * @brief gGremsy_init
 * The function shall initialize protocol base and callback function
 * @param NONE
 * @return NONE
 */
void gGremsy_init(void)
{
    /* Init protocol base*/
    gProtocol_init(&proto);
    
    gGimbal_Console((uint8_t *)"GREMSY JIG TEST AC30000 !!! \n");
    
    comm_channel_2.set_motor               = control_gimbal_motor;
    comm_channel_2.set_mode                = control_gimbal_mode;
    comm_channel_2.set_axis_mode           = control_gimbal_axis_mode;
    comm_channel_2.set_move                = control_gimbal_set_move;
}

/**
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gGremsy_read_data(void)
{
   // Process read data
    if(gProtocol_read_data(&mav) == 1)
    {
        handle_message(&mav, &comm_channel_2);
    }
    
    if(gProtocol_serialPort_COM4_read_data(&mav1) == 1)
    {
        handle_message(&mav1, &comm_channel_4);
    }
}

/**
 * @brief  Function handle 
 */
static usb_speedTest_status_t GREMSY_AC30000_usb_speedTest_get_status(void)
{
    usb_speedTest_status_t status = STANBY;
    
    if(comm_channel_4.param_value.param_index == 3)
    {
        if(comm_channel_4.param_value.param_value == 0)
        {
            status = STANBY;
        }
        else if(comm_channel_4.param_value.param_value == 1)
        {
            status = RUNNING;
        }
        else if(comm_channel_4.param_value.param_value == 2)
        {
            status = DONE;
        }
    }
    
    return status;
}

usb_speedTest_resultSpeed_t resultState;
static void GREMSY_AC30000_JIG_TEST_usb_speedTest_sendRequestParamResult(void)
{
    if(resultState == STATE_READ_SPEED)
    {
        comm_channel_4.param_value.debug_param_index = STATE_READ_SPEED;
        
        if(comm_channel_4.usb_speed.valueRead == 0)
        send_request_param_read(MAVLINK_COMM_2, "READSPEED", comm_channel_4.param_value.debug_param_index);
        
        resultState = STATE_WRITE_SPEED;
    }
    else if(resultState == STATE_WRITE_SPEED)
    {
        comm_channel_4.param_value.debug_param_index = STATE_WRITE_SPEED;
        
        if(comm_channel_4.usb_speed.valueWrite == 0)
        send_request_param_read(MAVLINK_COMM_2, "WRITESPEED", comm_channel_4.param_value.debug_param_index);
        
        resultState = STATE_RESULT;
    }
    else if(resultState == STATE_RESULT)
    {
        comm_channel_4.param_value.debug_param_index = STATE_RESULT;
        if(comm_channel_4.usb_speed.result == 0)
        send_request_param_read(MAVLINK_COMM_2, "RESULT", comm_channel_4.param_value.debug_param_index);
        resultState = STATE_STATUS;
    }
    else if(resultState == STATE_STATUS)
    {
        comm_channel_4.param_value.debug_param_index = STATE_STATUS;
        send_request_param_read(MAVLINK_COMM_2, "STATUS", comm_channel_4.param_value.debug_param_index);
        resultState = STATE_REF_READ;
    }
    else if(resultState == STATE_REF_READ)
    {
        comm_channel_4.param_value.debug_param_index = STATE_REF_READ;
        
        if(comm_channel_4.usb_speed.ref_valueRead == 0)
        send_request_param_read(MAVLINK_COMM_2, "REF_READ", comm_channel_4.param_value.debug_param_index);
        resultState = STATE_REF_WRITE;
    }
    else if(resultState == STATE_REF_WRITE)
    {
        comm_channel_4.param_value.debug_param_index = STATE_REF_WRITE;
        
        if(comm_channel_4.usb_speed.ref_valueWrite == 0)
        send_request_param_read(MAVLINK_COMM_2, "REF_WRITE", comm_channel_4.param_value.debug_param_index);
        resultState = STATE_READ_SPEED;
    }
}

/**
 * @brief  Function handle 
 */
static float GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(uint8_t param_index)
{
    float ret = 0;
    if(comm_channel_4.param_value.param_index == param_index)
    {
        ret = comm_channel_4.param_value.param_value;
        comm_channel_4.param_value.param_value = 0;
    }
    
    return ret;
}

/**
 * @brief  Function handle 
 */
bool GREMSY_AC30000_JIG_TEST_send_usb_runTest(bool enable)
{
    bool ret = false;
    
    if(enable == true)
    {
        /// send request status
        comm_channel_4.param_value.debug_param_index = 4;
        send_request_param_read(MAVLINK_COMM_2, "RUNTEST", comm_channel_4.param_value.debug_param_index);
    }
    
    return ret;
}



/**
 * @brief  Function handle 
 */
bool GREMSY_AC30000_JIG_TEST_send_endTest(bool enable)
{
    bool ret = false;
    
    if(enable == true)
    {
        /// send request status
        comm_channel_4.param_value.debug_param_index = 5;
        send_request_param_read(MAVLINK_COMM_2, "ENDTEST", comm_channel_4.param_value.debug_param_index);
    }
    
    return ret;
}

/**
 * @brief  Function handle
 */
static void GREMSY_AC30000_JIG_TEST_reciever_fullResult(gGimbal_t *comm)
{
    if(comm->usb_speed.status == DONE)
    {
        if((comm->usb_speed.valueRead != 0) && 
            (comm->usb_speed.valueWrite != 0) &&
            (comm->usb_speed.result != 0) &&
            (comm->usb_speed.ref_valueRead != 0) && 
            (comm->usb_speed.ref_valueWrite != 0) 
        )
        {
            if(comm->usb_speed.is_send_End == false)
            {
                comm->usb_speed.is_send_End = true;
                GREMSY_AC30000_JIG_TEST_send_endTest(true);
                gGimbal_Console((uint8_t *)"End test USB \n");
            }
        }
    }
}

/**
 * @brief  Function handle 
 */

void GREMSY_AC30000_JIG_TEST_usb_speedTest_process(void)
{
//    usb_speedTest_status_t status = GREMSY_AC30000_usb_speedTest_get_status();

    if(comm_channel_4.param_value.param_index == 3)
    {
        if(comm_channel_4.param_value.param_value == STANBY)
        {
            comm_channel_4.usb_speed.status = STANBY;
            // set bien enable runtest usb
            if(comm_channel_4.usb_speed.is_send_runTest == false)
            {
                comm_channel_4.usb_speed.is_send_runTest = true;
                /// send run test
                GREMSY_AC30000_JIG_TEST_send_usb_runTest(true);
            }
            
        }
        else if(comm_channel_4.param_value.param_value == RUNNING)
        {
            comm_channel_4.usb_speed.status = RUNNING;
            
            comm_channel_4.usb_speed.valueRead = 0.00000f;
            comm_channel_4.usb_speed.valueWrite = 0.00000f;
            
    //        // set bien enable runtest usb
    //        comm_channel_4.usb_speed.is_send_runTest = RUNNING;
        }
        else if(comm_channel_4.param_value.param_value == DONE)
        {
            comm_channel_4.usb_speed.status = DONE;
            comm_channel_4.usb_speed.is_sendRequest = true;
            comm_channel_4.usb_speed.is_DoneTest = true;
        }
    }
    
    /// send request result
    if(comm_channel_4.usb_speed.is_sendRequest == true)
    {
        if(gSystick_time_while(500, 102))
        {
            /// send request param
            GREMSY_AC30000_JIG_TEST_usb_speedTest_sendRequestParamResult();
        }
        
        if(comm_channel_4.param_value.param_index != 3)
        {
            /// read result and speed
            if(comm_channel_4.usb_speed.valueRead == 0)
            {
                comm_channel_4.usb_speed.valueRead      = GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(0);
            }
            
            if(comm_channel_4.usb_speed.valueWrite == 0)
            {
                comm_channel_4.usb_speed.valueWrite     = GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(1);
            }
            
            /// read result and speed
            if(comm_channel_4.usb_speed.ref_valueRead == 0)
            {
                comm_channel_4.usb_speed.ref_valueRead      = GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(6);
            }
            
            if(comm_channel_4.usb_speed.ref_valueWrite == 0)
            {
                comm_channel_4.usb_speed.ref_valueWrite     = GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(7);
            }
            
//            comm_channel_4.usb_speed.result         = GREMSY_AC30000_JIG_TEST_usb_speedTest_get_valueSpeed(2);
            if(comm_channel_4.param_value.param_index == 2)
            {
                comm_channel_4.usb_speed.result = comm_channel_4.param_value.param_value;
            }
        }
    }
    else
    {
        /// send request status
        if(gSystick_time_while(500, 103))
        {
            comm_channel_4.param_value.debug_param_index = 3;
            send_request_param_read(MAVLINK_COMM_2, "STATUS", comm_channel_4.param_value.debug_param_index);
        }
        
    }
    
    GREMSY_AC30000_JIG_TEST_reciever_fullResult(&comm_channel_4);
}


/**
 * @brief  Function handle 
 */
void GREMSY_AC30000_JIG_TEST_HEARTBEAT_timeOut(void)
{
    if(comm_channel_2.heartbeatTimeOut_count > 10)
    {
        
        /// gimbal heartbeat timeOut next step result
        comm_channel_2.heartbeatTimeOut = true;
        
//        comm_channel_2.is_send_param_Done = true;
    }
    else
    {
        if(get_timeOut(1000, JIG_TEST_HEARTBEAT_TIMEOUT))
        {
            comm_channel_2.heartbeatTimeOut_count ++;
            HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
            HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
        }
    }
}

uint8_t paramResult;
static uint8_t get_param_gimbal(uint8_t *nextParam)
{
    char buff[200];

    if(comm_channel_2.param_value.param_index == param_gimbal[*nextParam].index)
    {
        ///
        param_gimbal[*nextParam].value_param_get = comm_channel_2.param_value.param_value;
        
        /// Compare 
        if(param_gimbal[*nextParam].value_param_get == param_gimbal[*nextParam].value)
        {
            paramResult ++;
        }
        sprintf(buff, "Count : %3d\nparam read : %s ---> value : %3d\n paramResult : %3d\n", 
                *nextParam, param_gimbal[*nextParam].param_id, 
                param_gimbal[*nextParam].value_param_get, 
                paramResult);
        gGimbal_Console((uint8_t *)buff);
        *nextParam += 1;
    }

    
    return param_gimbal[*nextParam].value_param_get;
}   

static bool GREMSY_AC30000_JIG_TEST_set_param_gimbal(void)
{
    bool ret = false;
    uint8_t i = 0;
    char buff[100];
    
    for(i = 0; i < NUMBER_OF_PARAM; i++)
    {
        send_param_gimbal(param_gimbal[i].value , ONBOARD_CHANNEL, param_gimbal[i].param_id);
        HAL_Delay(5);
//        sprintf(buff, "set param gimbal : %s  |  count : %5d\n", param_gimbal[i].param_id, i);
//        gGimbal_Console((uint8_t *)buff);
    }
    ret = true;
    
    return ret;
}

static bool GREMSY_AC30000_JIG_TEST_get_param_gimbal(void)
{
    bool ret = false;
    char buff[100];

    send_paramRequest_read(ONBOARD_CHANNEL,
    param_gimbal[comm_channel_2.param_value.param_readCount].index,
    param_gimbal[comm_channel_2.param_value.param_readCount].param_id);
    
    value_param_get = get_param_gimbal(&comm_channel_2.param_value.param_readCount);


    /// kiem tra bien paramResult
    if(paramResult == 15)
    {
        ret = true;
        gGimbal_Console((uint8_t *)"COMpare Done !!!\n");
    }
    
    return ret;
}

/** 
 */
void GREMSY_AC30000_JIG_TEST_send_param_gimbal(void)
{
  uint8_t i = 0;

    if(comm_channel_2.seen_heartbeat == 0)
    {
        /// timeOut heatbeat o day
        GREMSY_AC30000_JIG_TEST_HEARTBEAT_timeOut();
    }
    else
    {
        uint8_t count;
        char buff[100];
        if(comm_channel_2.param_gimbalCount == SET_PARAM_STATE_JIG_TEST)
        {
            if(GREMSY_AC30000_JIG_TEST_set_param_gimbal())
            {
                comm_channel_2.is_send_param_Done = true;
                comm_channel_2.param_gimbalCount = SET_PARAM_STATE_CHECK_NOISE;
            }
        }
        else if(comm_channel_2.param_gimbalCount == SET_PARAM_STATE_CHECK_NOISE)
        {
//            param_gimbal[9].value = 90; // PAN
//            param_gimbal[10].value = 80; // ROLL
//            param_gimbal[11].value = 120; // TILT
            
//            if(GREMSY_AC30000_JIG_TEST_set_param_gimbal())
//            {
                comm_channel_2.param_gimbalCount = SET_PARAM_STATE_GET_PARAM_PROCESS;
            
                /// bat dau request tu param nay
                comm_channel_2.param_value.param_readCount = 9;
//            }
        }
        else if(comm_channel_2.param_gimbalCount == SET_PARAM_STATE_GET_PARAM_PROCESS)
        {
            if(GREMSY_AC30000_JIG_TEST_get_param_gimbal())
            {
                comm_channel_2.param_gimbalCount = SET_PARAM_STATE_GET_PARAM_DONE;
            }
        }
        
    }

}

/**
 * @brief  This is function send data to gimbal
 * @param   NONE
 * @param   NONE
 * @return NONE
 */
void gGremsy_send_data(void)
{
    /* Frequency maximum to send that 50 hz*/
//    if(comm_channel_2.time_update++ >= TELEMETRY_MAVLINK_DELAY)
//    {
//        comm_channel_2.time_update = 0;
//        
    if(gSystick_time_while(500, 150))
    {
        send_heartbeat(ONBOARD_CHANNEL);
    }
    
    /// send heartbeat to UART4
    if(gSystick_time_while(1000, 105))
    {
        send_heartbeat(MAVLINK_COMM_2);
    }
    
    
}

/**
 * @brief  The function will run example
 * Function will turn off/on gimbal and get some data from gimbal 
 * @param   none
 * @param   none
 * @return none
 */
void gGimbal_display(void)
{
    if(gSystick_timebase_get(&last_time_display) > 1000)
    {
        
        if(comm_channel_2.system_type == MAV_TYPE_GIMBAL)
        {
//            sprintf(buffDbg, "\n GB: id %d ", comm_channel_2.vehicle_system_id);
//            gGimbal_Console((uint8_t*)buffDbg);
            
            
            /* Display gimbal's value*/
            sprintf(buffDbg, "\n GB: id %d\nMOUNT-> time:%d pitch:%2.3f, roll:%2.3f, yaw:%2.3f, seq:%d\n", 
                                        comm_channel_2.vehicle_system_id,                            
                                        comm_channel_2.mount_val.time_boot_ms,
                                        comm_channel_2.mount_val.pitch, 
                                        comm_channel_2.mount_val.roll, 
                                        comm_channel_2.mount_val.yaw,
                                        comm_channel_2.mount_val.seq);
            gGimbal_Console((uint8_t*)buffDbg);

            
            gSystick_timebase_reset(&last_time_display);
        }
    }
    
}

void gGimbal_control(void)
{
    char buff[200];
    
    if(comm_channel_2.system_type == MAV_TYPE_GIMBAL)
    {
        if(sdk.state == STATE_IDLE)
        {
            sdk.state = STATE_GIMBAL_MOVE_RIGHT;
        }
        else if(sdk.state == STATE_GIMBAL_MOVE_RIGHT)
        {
            int16_t setpoint_tilt = 30;
            int16_t setpoint_roll = 0;
            int16_t setpoint_pan  = 180;
            
            if(comm_channel_2.ack.command != 205)
            {
    //            if(gSystick_timebase_get(&last_time_control) > 1000)
//                if(get_timeOut(100, JIG_TEST_SEND_CONTROL_GIMBAL_1))
//                {
//                    gSystick_timebase_reset(&last_time_control);
//                    sprintf(buff, "Control gimbal in angle mode : MOVE RIGHT!   B  C : %d  %d\n", buttonFeedbackIMU, feedBackIMUCount);
//                    gGimbal_Console((uint8_t*)buff);
                    /// Apply 
                    comm_channel_2.set_move(setpoint_tilt, setpoint_roll, setpoint_pan, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                   
//                }
                
//                comm_channel_2.moveRight ++;
            }
            else
            {
                if(get_timeOut(500, JIG_TEST_SEND_CONTROL_GIMBAL_1))
                {
                    gSystick_timebase_reset(&last_time_control);
//                    sprintf(buff, "Control gimbal in angle mode : MOVE RIGHT!   B  C : %d  %d\n", buttonFeedbackIMU, feedBackIMUCount);
//                    gGimbal_Console((uint8_t*)buff);
                    /// Apply 
                    comm_channel_2.set_move(setpoint_tilt, setpoint_roll, setpoint_pan, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                   
                }
            }
            

            int delta_tilt = (int)(comm_channel_2.mount_val.pitch - setpoint_tilt);
            int delta_roll = (int)(comm_channel_2.mount_val.roll - setpoint_roll);
            int delta_pan = (int)(comm_channel_2.mount_val.yaw - setpoint_pan);
            
            
            /// Check the current angle after 5s
//            if(gSystick_timebase_get(&last_time_send) > 4500)
            if(get_timeOut(1000, JIG_TEST_CONTROL_GIMBAL_1))
            {
                gSystick_timebase_reset(&last_time_send);
                
                if(buttonFeedbackIMU == true)
                {
                    if((abs(delta_tilt) < 5) && (abs(delta_roll) < 5) && (abs(delta_pan) < 5))
                    {
                        sdk.state = STATE_GIMBAL_MOVE_LEFT;
                        feedBackIMUCount++;
                        if(feedBackIMUCount > 4)
                        {
                            feedBackIMUCount = 0;
                            buttonFeedbackIMU = false;
                            gimbal_set_home();
                        }
                    }
                }
                else
                {
                    if((abs(delta_tilt) < 10) && (abs(delta_roll) < 5) && (abs(delta_pan) < 15))
                    {
                        comm_channel_2.ack.enable = false;
                        comm_channel_2.ack.command = 0;
                        
                        gimbal_control_result = true;
                        sdk.state = STATE_GIMBAL_MOVE_LEFT;
                    }
                }
            }
            
        }
        else if(sdk.state == STATE_GIMBAL_MOVE_LEFT)
        {
            int16_t setpoint_tilt = -30;
            int16_t setpoint_roll = 0;
            int16_t setpoint_pan  = -180;
            
//            if(comm_channel_2.ack.command != 205)
//            {
    //            if(gSystick_timebase_get(&last_time_control) > 1000)
                if(get_timeOut(500, JIG_TEST_SEND_CONTROL_GIMBAL_2))
                {
                    gSystick_timebase_reset(&last_time_control);
//                    sprintf(buff, "Control gimbal in angle mode : MOVE LEFT!   B  C : %d  %d\n", buttonFeedbackIMU, feedBackIMUCount);
//                    gGimbal_Console((uint8_t*)buff);
                    /// Apply 
                    comm_channel_2.set_move(setpoint_tilt, setpoint_roll, setpoint_pan, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);
                   
                }
//                comm_channel_2.moveLeft ++;
//            }
//            else
//            {
//                comm_channel_2.ack.enable = true;
//            }

            int delta_tilt = (int)(comm_channel_2.mount_val.pitch - setpoint_tilt);
            int delta_roll = (int)(comm_channel_2.mount_val.roll - setpoint_roll);
            int delta_pan = (int)(comm_channel_2.mount_val.yaw - setpoint_pan);
            
            
            /// Check the current angle after 5s
//            if(gSystick_timebase_get(&last_time_send) > 4500)
            if(get_timeOut(1000, JIG_TEST_CONTROL_GIMBAL_2))
            {
                gSystick_timebase_reset(&last_time_send);
                
                if(buttonFeedbackIMU == true)
                {
                    if((abs(delta_tilt) < 5) && (abs(delta_roll) < 5) && (abs(delta_pan) < 5))
                    {
                        sdk.state = STATE_GIMBAL_MOVE_RIGHT;
                        feedBackIMUCount++;
                        
                        if(feedBackIMUCount > 4)
                        {
                            feedBackIMUCount = 0;
                            buttonFeedbackIMU = false;
                            gimbal_set_home();
                        }
                    }
                }
                else
                {
                    if((abs(delta_tilt) < 5) && (abs(delta_roll) < 5) && (abs(delta_pan) < 5))
                    {
                        comm_channel_2.ack.enable = false;
                        comm_channel_2.ack.command = 0;
                        
                        gimbal_control_result = true;
                        comm_channel_2.set_motor(TURN_OFF);
                    }
//                    else
//                    {
//                        gimbal_control_result = false;
//                    }
                }
                
            }
        }
    }
}

/*Private function ************************************************************/

/**
 * @brief  This function send heartbeart
 * @param none
 * @return none
 */
void send_heartbeat(mavlink_channel_t channel)
{
    mavlink_message_t       msg;
    mavlink_heartbeat_t     heartbeat;
    uint16_t                len = 0;
    
    heartbeat.type          = MAV_TYPE_ONBOARD_TESTER; 
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0; 
    heartbeat.system_status = MAV_STATE_ACTIVE;
    
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_heartbeat_encode_chan(SYSID_ONBOARD,
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

/**
 * @brief  This function send motor control
 * @param 
 * @return 
 */
void control_gimbal_motor(control_gimbal_motor_t type)
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
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    ONBOARD_CHANNEL,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(ONBOARD_CHANNEL,(const char*) msgbuf, len);
    }
}

/**
 * @brief  This function send a gimbal mode
 * @param 
 * @return 
 */
void control_gimbal_mode(control_gimbal_mode_t mode)
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
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    ONBOARD_CHANNEL,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(ONBOARD_CHANNEL,(const char*) msgbuf, len);
    }
}

/**
 * @brief  This function control gimbal for each axis 
* @param Tilt mode for tilt axis
 * @param 
 * @param 
 * @return None
 */
void control_gimbal_axis_mode(control_gimbal_axis_mode_t tilt, 
                              control_gimbal_axis_mode_t roll,
                              control_gimbal_axis_mode_t pan)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    /*Default all axes that stabilize mode */
    roll.stabilize  = 1;
    tilt.stabilize  = 1;
    pan.stabilize   = 1;
    
    command_long.command            = MAV_CMD_DO_MOUNT_CONFIGURE;
    command_long.param1             = MAV_MOUNT_MODE_MAVLINK_TARGETING;
    
    command_long.param2             = roll.stabilize;
    command_long.param3             = tilt.stabilize;
    command_long.param4             = pan.stabilize;
    
    command_long.param5             = roll.input_mode;
    command_long.param6             = tilt.input_mode;
    command_long.param7             = pan.input_mode;
    
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    ONBOARD_CHANNEL,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(ONBOARD_CHANNEL,(const char*) msgbuf, len);
    }
}
/**
 * @brief  This function support control gimbal in speed and angle mode
 * @param 
 * @return 
 */
void control_gimbal_set_move(int16_t tilt, int16_t roll, int16_t pan, E_GimbalRotationMode rotationMode)
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
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    ONBOARD_CHANNEL,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(ONBOARD_CHANNEL,(const char*) msgbuf, len);
    }
}
/**
 * @brief  This function send set home position for gimbal
 * @param none
 * @return none
 */
void gimbal_set_home(void)
{
    mavlink_message_t           msg;
    mavlink_command_long_t      command_long = {0};
    uint16_t                    len = 0;
   
    command_long.command        = MAV_CMD_USER_2;
    command_long.param6         = 3;
    command_long.param7         = 0x04;
    command_long.target_component   = MAV_COMP_ID_GIMBAL;
    command_long.target_system      = MAV_COMP_ID_SYSTEM_CONTROL;
    
    /*
      save and restore sequence number for chan, as it is used by
      generated encode functions
     */
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
    uint8_t saved_seq = chan_status->current_tx_seq;
    
    mavlink_msg_command_long_encode_chan(SYSID_ONBOARD,
                                    MAV_COMP_ID_SYSTEM_CONTROL,
                                    ONBOARD_CHANNEL,
                                    &msg,
                                    &command_long);
                                
    chan_status->current_tx_seq = saved_seq;
    
    uint8_t msgbuf[len];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
        _mavlink_send_uart(ONBOARD_CHANNEL,(const char*) msgbuf, len);
    }
}


/**
 * @brief  Function handle message from gimbal
 */
void send_request_param_read(mavlink_channel_t channel, char* param_id, uint8_t param_index)
{
    mavlink_message_t msg;
    mavlink_param_request_read_t request_read;
    uint16_t len = 0;
    
    mav_array_memcpy(request_read.param_id, param_id, sizeof(char) * 16);
    request_read.param_index = param_index;
    request_read.target_component = 0;
    request_read.target_system = 0;
    
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


/**
 * @brief  This function send param gimbal
 * @param 
 * @return 
 */
void send_param_gimbal(uint16_t param_value, mavlink_channel_t channel, char* param_id)
{
    uint8_t ret = 0;
    uint16_t len;
    mavlink_message_t msg;
    
    mavlink_status_t    *chan_status = mavlink_get_channel_status(channel);
    uint8_t saved_seq = chan_status->current_tx_seq;

    mavlink_msg_param_set_pack(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &msg, SYSID_ONBOARD, MAV_COMP_ID_GIMBAL,
                               param_id, param_value, MAVLINK_TYPE_UINT16_T);
    
    chan_status->current_tx_seq = saved_seq;

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {    
        _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
    
}

/**
 * @brief This function supports for setting gimbal mode down
 * @details Function will reset yaw axis to the home position and set pitch axis to 90 degrees
 * @note This function allow control pitch and yaw with 2 dial button
 */
void control_gimbal_set_Reboot(mavlink_channel_t channel)
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

void send_paramRequest_read(mavlink_channel_t channel, uint8_t param_index, char* param_id)
{
    uint8_t ret = 0;
    uint16_t len;
    mavlink_message_t msg;
    mavlink_param_request_read_t param_request_read;
    uint8_t param_id_size = strlen(param_id);
    
    strcpy(param_request_read.param_id, param_id);
    
    
    param_request_read.param_index = param_index;
    param_request_read.target_component = MAV_COMP_ID_GIMBAL;
    param_request_read.target_system = SYSID_ONBOARD;

    mavlink_msg_param_request_read_encode(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &msg, &param_request_read);

    uint8_t msgbuf[MAVLINK_MAX_PACKET_LEN];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0)
    {
         _mavlink_send_uart(channel,(const char*) msgbuf, len);
    }
}

/**
 * @brief  Function handle message from gimbal
 */
void handle_message(mav_state_t* mav, gGimbal_t *comm_channel)
{
    switch (mav->rxmsg.msgid)
    {

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t	heartbeat;
            mavlink_msg_heartbeat_decode(&mav->rxmsg, &heartbeat);
            
            /* Check heartbeat if the gimbal have gotten*/
            if(!comm_channel->seen_heartbeat)
            {
                comm_channel->seen_heartbeat = 1;
                
                comm_channel->vehicle_component_id   = mav->rxmsg.compid;
                comm_channel->vehicle_system_id      = mav->rxmsg.sysid;
            }
            comm_channel->system_type            = heartbeat.type;
        }
        break;
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t              packet;
            mavlink_msg_sys_status_decode(&mav->rxmsg, &packet);
            
            /* Get voltage battery*/
            comm_channel->status.voltage_battery   = packet.voltage_battery;
            
            comm_channel->status.load              = packet.load;

            /* Check gimbal's motor */
            if(packet.errors_count1 & 0x10)
            {
                comm_channel->status.state = GIMBAL_STATE_ON;
                
                /* Check gimbal is follow mode*/
                if(packet.errors_count1 & 0x01)
                {
                    comm_channel->status.mode = GIMBAL_STATE_FOLLOW_MODE;
                }
                else
                {
                    comm_channel->status.mode = GIMBAL_STATE_LOCK_MODE;
                }
            }
            /* Check gimbal is initializing*/
            else if(packet.errors_count1 & 0x20)
            {
                comm_channel->status.state = GIMBAL_STATE_INIT;
            }
            else if(packet.errors_count1 & 0x04)
            {
                /* Check gimbal is error state*/
                comm_channel->status.state = GIMBAL_STATE_ERROR;
            }
            else if(!(packet.errors_count1 & 0x00))
            {
                comm_channel->status.state    = GIMBAL_STATE_OFF;
                comm_channel->status.mode     = GIMBAL_STATE_OFF;
            }
            
            /* Check gimbal's sensor status */
            if(packet.errors_count2 & 0x01)
            {
                comm_channel->status.sensor |= SENSOR_IMU_ERROR;
            }
            if(packet.errors_count2 & 0x02)
            {
                comm_channel->status.sensor |= SENSOR_EN_TILT;
            }
            if(packet.errors_count2 & 0x04)
            {
                comm_channel->status.sensor |= SENSOR_EN_ROLL;
            }
            if(packet.errors_count2 & 0x08)
            {
                comm_channel->status.sensor |= SENSOR_EN_PAN;
            }
            else 
            {
                comm_channel->status.sensor = SENSOR_OK;
            }
            
            /// kiem tra gimbal StartUp calib

            if((packet.errors_count1 & GIMBAL_STATE_SENSOR_CALIB) == GIMBAL_STATE_SENSOR_CALIB)
            {
                comm_channel->status.startUpCalib_running = false;
            }
            else
            {
                comm_channel->status.startUpCalib_running = true;
            }
            
            
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            comm_channel->status.seq = chan_status->current_rx_seq;
        }
        break;
        case MAVLINK_MSG_ID_MOUNT_STATUS:
        {
            mavlink_mount_status_t packet = {0};
            mavlink_msg_mount_status_decode(&mav->rxmsg, &packet);
            
            /* Get encoder value */
            comm_channel->encoder_val.pitch   = packet.pointing_a;
            comm_channel->encoder_val.roll    = packet.pointing_b;
            comm_channel->encoder_val.yaw     = packet.pointing_c;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            comm_channel->encoder_val.seq = chan_status->current_rx_seq;
        }
        break;
        case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
        {
            mavlink_mount_orientation_t          packet = {0};
            mavlink_msg_mount_orientation_decode(&mav->rxmsg, &packet);
            
            /* Get attitude value */
            comm_channel->mount_val.time_boot_ms   = packet.time_boot_ms;
            comm_channel->mount_val.pitch          = packet.pitch;
            comm_channel->mount_val.roll           = packet.roll;
            comm_channel->mount_val.yaw            = packet.yaw;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            comm_channel->mount_val.seq = chan_status->current_rx_seq;
        }
        break;
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_attitude_t          packet = {0};
            mavlink_msg_attitude_decode(&mav->rxmsg, &packet);
            
            /* Get attitude value */
            comm_channel->attitude.pitch          = packet.pitch * PI2ANGLE;
            comm_channel->attitude.roll           = packet.roll * PI2ANGLE;
            comm_channel->attitude.yaw            = packet.yaw * PI2ANGLE;
            comm_channel->attitude.pitchspeed     = packet.pitchspeed * PI2ANGLE;
            comm_channel->attitude.rollspeed      = packet.rollspeed * PI2ANGLE;
            comm_channel->attitude.yawspeed       = packet.yawspeed * PI2ANGLE;
            comm_channel->attitude.time_boot_ms   = packet.time_boot_ms;
            
            mavlink_status_t    *chan_status = mavlink_get_channel_status(ONBOARD_CHANNEL);
            
            comm_channel->attitude.seq = chan_status->current_rx_seq;
        }
        break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            mavlink_param_value_t          packet = {0};
            mavlink_msg_param_value_decode(&mav->rxmsg, &packet);
            
            for(uint8_t i = 0; i < strlen(comm_channel->param_value.param_id); i++)
            {
                comm_channel->param_value.param_id[i] = 0;
            }
            
            comm_channel->param_value.param_count = packet.param_count;
            memcpy(comm_channel->param_value.param_id, packet.param_id, strlen(packet.param_id));

            comm_channel->param_value.param_index = packet.param_index;
            comm_channel->param_value.param_type = packet.param_type;
            comm_channel->param_value.param_value = packet.param_value;
        }
        break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            if(comm_channel->ack.enable == false)
            {
                mavlink_command_ack_t         packet = {0};
                mavlink_msg_command_ack_decode(&mav->rxmsg, &packet);
                
                comm_channel->ack.command = packet.command;
                comm_channel->ack.progress = packet.progress;
                comm_channel->ack.result = packet.result;
            }
        }
        break;
        case MAVLINK_MSG_ID_DEBUG :
        {
//            char buff[100];
            mavlink_debug_t         packet = {0};
            mavlink_msg_debug_decode(&mav->rxmsg, &packet);
            
            comm_channel->gimbalDebug.ind = packet.ind;
            comm_channel->gimbalDebug.time_boot_ms = packet.time_boot_ms;
            comm_channel->gimbalDebug.value = packet.value;
            
//            sprintf(buff, "debugValue : %3f\n debugIndex : %3d", comm_channel->gimbalDebug.value, comm_channel->gimbalDebug.ind);
//            gGimbal_Console((uint8_t *)buff);
            
        }break;
        case MAVLINK_MSG_ID_RAW_IMU:
        {
//            char buff[100];
            mavlink_raw_imu_t         packet = {0};
            mavlink_msg_raw_imu_decode(&mav->rxmsg, &packet);
            
            comm_channel->raw_imu.time_usec = packet.time_usec;
            comm_channel->raw_imu.xacc = packet.xacc;
            comm_channel->raw_imu.xgyro = packet.xgyro;
            comm_channel->raw_imu.xmag = packet.xmag;
            comm_channel->raw_imu.yacc = packet.yacc;
            comm_channel->raw_imu.ygyro = packet.ygyro;
            comm_channel->raw_imu.ymag = packet.ymag;
            comm_channel->raw_imu.zacc = packet.zacc;
            comm_channel->raw_imu.zgyro = packet.zgyro;
            comm_channel->raw_imu.zmag = packet.zmag;
            
//            sprintf(buff, "raw_imu.xacc : %3d\n raw_imu.yacc : %3d\n", 
//            comm_channel->raw_imu.xacc, 
//            comm_channel->raw_imu.yacc);
//            gGimbal_Console((uint8_t *)buff);
        }
        break;
        default:
        {
        }
        break;
    }
}
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE**********/
