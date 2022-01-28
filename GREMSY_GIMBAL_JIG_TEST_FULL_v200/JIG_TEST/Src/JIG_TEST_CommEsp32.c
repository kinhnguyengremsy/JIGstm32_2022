/**
  ******************************************************************************
  * @file    JIG_TEST_CommEsp32.c
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2011 Gremsy.
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
#include "JIG_TEST_config.h"

#if (COMM_ESP32 == 1)

#include "JIG_TEST_CommEsp32.h"
#include "JIG_TEST_ppm_gimbal.h"
#include "JIG_TEST_can_dji_.h"
#include "JIG_TEST_sbus_gimbal.h"
#include "JIG_TEST_display_v2.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_gimbal_FSTD_v2.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_button.h"
#include "JIG_TEST_aux.h"
#include "JIG_TEST_rtc.h"

#include "timeOut.h"
#include "main.h"
#include "string.h"
#include "math.h"
/* Private typedef------------------------------------------------------------------------------*/
typedef void (*Function_t)( void * );

/**
 * @brief jigStatus
 * jigStatus
 */
typedef enum _jigStatus_t
{
    COMMAND_START = 1,
    COMMAND_STOP,
    COMMAND_RESET,
    COMMAND_WGRH,
		COMMAND_SETPARAM,

}jigStatus_t;

/**
 * @brief controlJigMode
 * controlJigMode
 */
typedef enum _controlJigMode_t
{
    CONTROL_JIG_MODE_IDLE = 0,
    CONTROL_JIG_MODE_SBUS,
    CONTROL_JIG_MODE_PPM,
    CONTROL_JIG_MODE_CAN,
    CONTROL_JIG_MODE_COM2,
    CONTROL_JIG_MODE_COM4,
    CONTROL_JIG_MODE_AUX,
    CONTROL_JIG_MODE_VIRATE,
    CONTROL_JIG_MODE_DONE,
    
}controlJigMode_t;

typedef struct
{
    bool done[8];
    uint32_t time[8];
    uint32_t result[8];
    bool checkResult;
    
}gimbalRC_management_t;
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart5;
extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM2;
extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM4;


extern mavlink_msg_heartbeat_t controlJig;
extern JIG_TEST_mavlink_gimbal_t mavlink_comm_rapberry;

gimbalRC_management_t management;

/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_COMMESP32_INIT
    @{
*/#ifndef JIG_TEST_COMMESP32_INIT
#define JIG_TEST_COMMESP32_INIT

/** @brief CommEsp32_init
    @return none
*/
void JIG_TEST_CommEsp32_init(void)
{
    char buff[200];

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

/** @group JIG_TEST_COMMESP32_GIMBAL_RC
    @{
*/#ifndef JIG_TEST_COMMESP32_GIMBAL_RC
#define JIG_TEST_COMMESP32_GIMBAL_RC

/** @brief CommEsp32_checkResultControl
    @return bool
*/
static bool JIG_TEST_CommEsp32_checkResultControl(int16_t panAngle, int16_t rollAngle, int16_t tiltAngle, uint8_t mode, uint32_t time)
{
    bool ret = false;
    static uint32_t timeDebug = 0;
    
    uint16_t deltaPan = abs((int16_t)(mavlink_gimbal_COM4.mount_val.yaw      - panAngle));
    uint16_t deltaRoll = abs((int16_t)(mavlink_gimbal_COM4.mount_val.roll    - rollAngle));
    uint16_t deltaTilt = abs((int16_t)(mavlink_gimbal_COM4.mount_val.pitch   - tiltAngle));
    
    if(HAL_GetTick() - timeDebug > 1000 || timeDebug == 0)
    {
        timeDebug = HAL_GetTick();
        
        char debugBuff[200];
        sprintf(debugBuff, "[checkResultControl] pan : %5d[%f] | roll : %5d[%f] | tilt : %5d[%f] | mode : %d | time : %10d\n"
        , deltaPan, mavlink_gimbal_COM4.mount_val.yaw
        , deltaRoll, mavlink_gimbal_COM4.mount_val.roll
        , deltaTilt, mavlink_gimbal_COM4.mount_val.pitch
        , mode, time);
        JIG_TEST_console_write(debugBuff);
    }
    
    if(deltaPan < 5 && deltaRoll == 0 && deltaTilt < 5)
    {
        char buff[200];
        uint32_t temp = 0;
        
        management.result[mode] = 1;
        
        temp = management.result[mode] << (mode - 1);
        
        controlJig.custom_mode |= temp;
        
        if(time <= 7)
        {
            sprintf(buff, "[checkResultControl] control mode %d Done late | result : %2d | temp : %5d | custom_mode : %10d\n", mode, management.result[mode],temp, controlJig.custom_mode);
            JIG_TEST_console_write(buff);
            
            // done mode
            controlJig.autopilot = 2;
        }
        else if(time > 7)
        {
            sprintf(buff, "[checkResultControl] control mode %d Done soon| result : %2d | temp : %5d | custom_mode : %10d\n", mode, management.result[mode],temp, controlJig.custom_mode);
            JIG_TEST_console_write(buff);
            
            /// done mode
            controlJig.autopilot = 2;
        }
        
        ret = true;
    }
    
    return ret;
}

/** @brief CommEsp32_GimbalControlSbus
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlSbus(void *arg)
{
    const int16_t panAngle  = -180;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = -90;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    management.time[position] = 10;
    
    if(management.done[position] == false)
    {
        JIG_TEST_sbus_gimbal_process();
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
    else
    {
        JIG_TEST_sbus_gimbal_enable(false);
    }
}

/** @brief CommEsp32_GimbalControlPpm
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlPpm(void *arg)
{
    static bool enablePpm = false;
    const int16_t panAngle  = 180;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = 90;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    if(enablePpm == false)
    {
        enablePpm = true;
        JIG_TEST_ppm_gimbal_enable(true);
    }
    
    management.time[position] = 10;
    
    if(management.done[position] == false)
    {
        JIG_TEST_ppm_gimbal_process();
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
    else
    {
        JIG_TEST_ppm_gimbal_enable(false);
        enablePpm = false;
    }
}

/** @brief CommEsp32_GimbalControlCan
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlCan(void *arg)
{
    static bool enableCan = false;
    const int16_t panAngle  = -180;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = -90;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    if(enableCan == false)
    {
        enableCan = true;
        JIG_TEST_can_dji_set_move(504, 504, true);
    }
    
    management.time[position] = 10;
    
    if(management.done[position] == false)
    {
        JIG_TEST_can_dji_process();
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
    else
    {
        JIG_TEST_ppm_gimbal_enable(false);
        enableCan = false;
    }
}


/** @brief CommEsp32_GimbalControlCom2
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlCom2(void *arg)
{
    const int16_t panAngle  = 170;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = 40;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    management.time[position] = 10;
    
    if(management.done[position] == false)
    {
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
}

/** @brief CommEsp32_GimbalControlCom4
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlCom4(void *arg)
{
    const int16_t panAngle  = -170;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = -45;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    management.time[position] = 10;
    
    if(management.done[position] == false)
    {
        JIG_TEST_gimbal_FSTD_v2_controlWithCom4();
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
}

/** @brief CommEsp32_GimbalControlAux
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlAux(void *arg)
{

}

/** @brief CommEsp32_GimbalControlVirate
    @return none
*/
static void JIG_TEST_CommEsp32_GimbalControlVirate(void *arg)
{
    const int16_t panAngle  = 0;
    const int16_t rollAngle = 0;
    const int16_t tiltAngle = 0;
    
    uint8_t position = mavlink_comm_rapberry.heartbeat.base_mode;
    
    management.time[position] = 12;
    
    if(management.done[position] == false)
    {
        if(JIG_TEST_gimbal_FSTD_v2_vibrate_v2() == true)
        management.done[position] = JIG_TEST_CommEsp32_checkResultControl(    panAngle
                                                                            , rollAngle
                                                                            , tiltAngle
                                                                            , position
                                                                            , management.time[position]);
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_COMMESP32_PROCESS
    @{
*/#ifndef JIG_TEST_COMMESP32_PROCESS
#define JIG_TEST_COMMESP32_PROCESS

/** @brief CommEsp32_getJigControl
    @return jigStatus_t
*/
static jigStatus_t JIG_TEST_CommEsp32_getJigControl(uint8_t heartBeat_type)
{
    jigStatus_t jigControl;
    
    switch(heartBeat_type)
    {
        case 1:
        {
            jigControl = COMMAND_START;
        }break;
        case 2:
        {
            jigControl = COMMAND_STOP;
        }break;
        case 3:
        {
            jigControl = COMMAND_RESET;
        }break;
        case 4:
        {
            jigControl = COMMAND_WGRH;
        }break;
    }
    
    return jigControl;
}

/** @brief CommEsp32_getModeControl
    @return controlJigMode_t
*/
static controlJigMode_t JIG_TEST_CommEsp32_getModeControl(uint8_t heartBeat_baseMode)
{
    controlJigMode_t modeControl;
    
    switch(heartBeat_baseMode)
    {
        case 1:
        {
            modeControl = CONTROL_JIG_MODE_SBUS;
        }break;
        case 2:
        {
            modeControl = CONTROL_JIG_MODE_PPM;
        }break;
        case 3:
        {
            modeControl = CONTROL_JIG_MODE_CAN;
        }break;
        case 4:
        {
            modeControl = CONTROL_JIG_MODE_COM2;
        }break;
        case 5:
        {
            modeControl = CONTROL_JIG_MODE_COM4;
        }break;
        case 6:
        {
            modeControl = CONTROL_JIG_MODE_AUX;
        }break;
        case 7:
        {
            modeControl = CONTROL_JIG_MODE_VIRATE;
        }break;
    }
    
    return modeControl;
}

void test(void *arg)
{
    static uint32_t time = 0;
    static uint16_t count = 0;
    char buff[100];
    
    if(HAL_GetTick() - time > 1000)
    {
        time = HAL_GetTick();
        sprintf(buff, "[func] test : %5d\n", count++);
        JIG_TEST_console_write(buff);
    }
}

/** @brief CommEsp32_mainTestProcess
    @return none
*/
static bool JIG_TEST_CommEsp32_applyControl(bool *start, Function_t func)
{
    bool ret = false;
    
    static uint32_t timeControl = 0; 
    static uint8_t countControl = 0;
    static bool waittingNextMode = false;

    if(*start == false)
    {
        *start = true;
    
        waittingNextMode = false;
        timeControl = 0;
        countControl = 0;
    
        /// running mode
        controlJig.autopilot = 1;
        
        JIG_TEST_console_write("[controlJig] New mode Control\n");
    }
                        
    if(HAL_GetTick() - timeControl > 1000 || timeControl == 0)
    {
        timeControl = HAL_GetTick();
    
        if(waittingNextMode == false)
        {
            char buff[100];
            sprintf(buff, "[controlJig] mode : %d | count : %d\n", mavlink_comm_rapberry.heartbeat.base_mode, countControl);
            JIG_TEST_console_write(buff);
            
            countControl ++;
        }
        else
        {
            JIG_TEST_console_write("[controlJig] test done waitting next mode\n");
        }
        
        if(countControl < 10)
        {
            //// printf console
            JIG_TEST_console_write("[controlJig] apply control\n");
        }
    }
    
    if(countControl > 15)
    {
        waittingNextMode = true;
        
        /// control mode Error
        controlJig.autopilot = 3;
    }
    else
    {
        /// apply control <--> gimbal is HOME
        func(NULL);
    }
    
    if(mavlink_comm_rapberry.heartbeat.autopilot == 2) ret = true;
    
    return ret;
}

/** @brief CommEsp32_mainTestProcess
    @return none
*/
static void JIG_TEST_CommEsp32_mainTestProcess(void)
{
    static bool startMode[8];
		static uint32_t timeToggleLed = 0;
    
    jigStatus_t         jigStatus;
    controlJigMode_t    modeControl; 
    
    jigStatus   = JIG_TEST_CommEsp32_getJigControl(mavlink_comm_rapberry.heartbeat.type);
    modeControl = JIG_TEST_CommEsp32_getModeControl(mavlink_comm_rapberry.heartbeat.base_mode);
    
    /// command start stop
    if(jigStatus == COMMAND_STOP)
    {
        static uint32_t timeStandby = 0;
    
        if(HAL_GetTick() - timeStandby > 1000 || timeStandby == 0)
        {
            timeStandby = HAL_GetTick();
        
            JIG_TEST_console_write("[controlJig] status Standby\n");
        }
    
        /// status standby
        controlJig.system_status = 1;
				
        /// turn on led green
        HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
    }
    else if(jigStatus == COMMAND_START)
    {
        if(HAL_GetTick() - timeToggleLed > 500)
        {
            timeToggleLed = HAL_GetTick();
            
            HAL_GPIO_TogglePin(green_GPIO_Port, green_Pin);
            HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
        }
			
        /// status running
        controlJig.system_status = 2;
        
        /// reciever command mode control
        if(modeControl == CONTROL_JIG_MODE_SBUS)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[1], JIG_TEST_CommEsp32_GimbalControlSbus) == true)
            {
                //// 
                
            }
        }
        else if(modeControl == CONTROL_JIG_MODE_PPM)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[2], JIG_TEST_CommEsp32_GimbalControlPpm) == true)
            {

            }
        }
        else if(modeControl == CONTROL_JIG_MODE_CAN)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[3], JIG_TEST_CommEsp32_GimbalControlCan) == true)
            {

            }
        }
        else if(modeControl == CONTROL_JIG_MODE_COM2)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[4], JIG_TEST_CommEsp32_GimbalControlCom2) == true)
            {

            }
        }
        else if(modeControl == CONTROL_JIG_MODE_COM4)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[5], JIG_TEST_CommEsp32_GimbalControlCom4) == true)
            {

            }
        }
        else if(modeControl == CONTROL_JIG_MODE_AUX)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[6], test) == true)
            {

            }
        }
        else if(modeControl == CONTROL_JIG_MODE_VIRATE)
        {
            if(JIG_TEST_CommEsp32_applyControl(&startMode[8], JIG_TEST_CommEsp32_GimbalControlVirate) == true)
            {

            }
        }
    }
    else if(jigStatus == COMMAND_RESET)
    {
        static uint32_t timeWattingReset = 0;
				static uint8_t countReset = 0;
				
        
        if(HAL_GetTick() - timeWattingReset > 200)
        {
            JIG_TEST_console_write("[controlJig] status reset\n");
					
						/// toggle led green
						HAL_GPIO_TogglePin(red_GPIO_Port, red_Pin);
						HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(blue_GPIO_Port, blue_Pin, GPIO_PIN_SET);
            
            timeWattingReset = HAL_GetTick();
            
            if(++countReset >= 11) NVIC_SystemReset();
        }
        
    }
    else if(jigStatus == COMMAND_WGRH)
    {
        static uint32_t timeWattingGimbalReturnHome = 0;
        
        /// status running
        controlJig.system_status = 2;
        
        if(HAL_GetTick() - timeWattingGimbalReturnHome > 1000 || timeWattingGimbalReturnHome == 0)
        {
            JIG_TEST_console_write("[controlJig] Watting Gimbal Return Home\n");
            
            timeWattingGimbalReturnHome = HAL_GetTick();
					
			HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
        }
    }
		else if(jigStatus == COMMAND_SETPARAM)
		{
            if(HAL_GetTick() - timeToggleLed > 100)
            {            
                timeToggleLed = HAL_GetTick();
                        
                HAL_GPIO_TogglePin(blue_GPIO_Port, blue_Pin);
                HAL_GPIO_WritePin(green_GPIO_Port, green_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(red_GPIO_Port, red_Pin, GPIO_PIN_SET);
            }				
		}
}

/** @brief CommEsp32_process
    @return none
*/
void JIG_TEST_CommEsp32_process(void)
{
    static uint32_t time_process = 0;
    static uint32_t time_processTemp =0;

    /// reset time process
    calculator_reset_time(&time_processTemp);
    
    /// send & reciever data from COM2, COM4, RASPBERRY
    JIG_TEST_mavlink_gimbal_process();

    JIG_TEST_CommEsp32_mainTestProcess();
    
    /// calculation time process
    time_process = calculator_get_time_us(&time_processTemp);
}

#endif
/**
    @}
*/

#endif
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


