/** 
  ******************************************************************************
  * @file    .c
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    __DATE__
  * @brief   
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy.  
  * All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "JIG_TEST_ppm_gimbal.h"
#include "JIG_TEST_console.h"
#include "ppm.h"
#include "timeOut.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    JIG_TEST_CONTROL_LOOP,
    JIG_TEST_CONTROL_TEST,
    
}JIG_TEST_ppm_gimbal_typeControl_t;

typedef struct
{
    bool is_Control_typeLoop_first;
    bool is_Control_typeLoop_CW;
    bool is_Control_typeLoop_CCW;
    
    bool is_Control_typeTest_first;
    bool is_Control_typeTest_process;
    bool is_Control_typeTest_result;
    
}JIG_TEST_ppm_gimbal_Control_t;

typedef struct
{
    bool            is_setValueChannel;
    ppm_channel_t   channel;
    uint16_t        value;
    
    JIG_TEST_ppm_gimbal_Control_t  control;
}JIG_TEST_ppm_gimbal_t;
/* Private define ------------------------------------------------------------*/

#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_MODE            PPM_CHANNEL_7
#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT            PPM_CHANNEL_2
#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_ROLL            PPM_CHANNEL_8
#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_PAN             PPM_CHANNEL_1
#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED      PPM_CHANNEL_3
#define JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_PAN_SPEED       PPM_CHANNEL_3

#define JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_OFF          1307
#define JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_LOCK         1507
#define JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_FOLLOW       1907

#define JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_VALUE_CW           1307
#define JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_VALUE_CCW          1707

#define JIG_TEST_PPM_GIMBAL_CHANNEL_ROLL_VALUE_CW           1507 
#define JIG_TEST_PPM_GIMBAL_CHANNEL_ROLL_VALUE_CCW          1507

#define JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_VALUE_CW            1307
#define JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_VALUE_CCW           1707

#define JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_SPEED_VALUE        1907
#define JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_SPEED_VALUE         JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_SPEED_VALUE

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
JIG_TEST_ppm_gimbal_t ppm_gimbal;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @group JIG_TEST_PPM_GIMBAL_CONFIGURATION
    @{
*/#ifndef JIG_TEST_PPM_GIMBAL_CONFIGURATION
#define JIG_TEST_PPM_GIMBAL_CONFIGURATION

/** @brief jig_test_ppm_gimbal_config
    @return none
*/
void JIG_TEST_ppm_gimbal_configuration(void)
{
    ppm_configuration();
}
#endif

/** @group JIG_TEST_PPM_GIMBAL_PROCESS
    @{
*/#ifndef JIG_TEST_PPM_GIMBAL_PROCESS
#define JIG_TEST_PPM_GIMBAL_PROCESS

/** @brief ppm_gimbal_setttingFirstControl
    @return none
*/
static void JIG_TEST_ppm_gimbal_setttingFirstControl(void)
{
    ppm_channel_t channel;
    int16_t value;
    
    /// setting channel mode
    channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_MODE;
    value = JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_LOCK;
    ppm_set_channel(channel, value);
    
    /// setting channel tilt
    channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT;
    value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_VALUE_CW;
    ppm_set_channel(channel, value);
    
    /// setting channel roll
    channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_ROLL;
    value = JIG_TEST_PPM_GIMBAL_CHANNEL_ROLL_VALUE_CW;
    ppm_set_channel(channel, value);
    
    /// setting channel pan
    channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_PAN;
    value = JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_VALUE_CW;
    ppm_set_channel(channel, value);
    
    /// setting channel tilt speed
    channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
    value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
    ppm_set_channel(channel, value);
}

/** @brief ppm_gimbal_Control_CW
    @return none
*/
static void JIG_TEST_ppm_gimbal_Control_CW(void)
{
    /// kiem tra thoi gian sau 6s chuyen sang control ccw
    if(get_timeOut(5000, PPM_CONTROL_TYPE_LOOP_CW))
    {
        /// reset flag control cw
        ppm_gimbal.control.is_Control_typeLoop_CW = false;
            
        /// set flag control ccw
        ppm_gimbal.control.is_Control_typeLoop_CCW = true;
        
        /// setting value sbus channel for ccw
        ppm_channel_t channel;
        int16_t value;
        
        /// setting channel mode
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_MODE;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_FOLLOW;
        ppm_set_channel(channel, value);
        
        /// setting channel tilt
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_VALUE_CCW;
        ppm_set_channel(channel, value);
        
        /// setting channel roll
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_ROLL;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_ROLL_VALUE_CCW;
        ppm_set_channel(channel, value);
        
        /// setting channel pan
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_PAN;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_VALUE_CCW;
        ppm_set_channel(channel, value);
        
        /// setting channel tilt speed
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
        ppm_set_channel(channel, value);
    }
}

/** @brief ppm_gimbal_Control_CCW
    @return none
*/
static void JIG_TEST_ppm_gimbal_Control_CCW(void)
{
    /// kiem tra thoi gian sau 6s chuyen sang control ccw
    if(get_timeOut(5000, PPM_CONTROL_TYPE_LOOP_CW))
    {
        /// reset flag control ccw
        ppm_gimbal.control.is_Control_typeLoop_CCW = false;
            
        /// set flag control cw
        ppm_gimbal.control.is_Control_typeLoop_CW = true;
        
        /// setting value sbus channel for cw
        ppm_channel_t channel;
        int16_t value;
        
        /// setting channel mode
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_MODE;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_MODE_VALUE_LOCK;
        ppm_set_channel(channel, value);
        
        /// setting channel tilt
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_VALUE_CW;
        ppm_set_channel(channel, value);
        
        /// setting channel roll
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_ROLL;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_ROLL_VALUE_CW;
        ppm_set_channel(channel, value);
        
        /// setting channel pan
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_PAN;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_PAN_VALUE_CW;
        ppm_set_channel(channel, value);
        
        /// setting channel tilt speed
        channel = JIG_TEST_PPM_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
        value = JIG_TEST_PPM_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
        ppm_set_channel(channel, value);
    }
}

/** @brief ppm_gimbal_process
    @return none
*/
static void JIG_TEST_ppm_gimbal_Control(JIG_TEST_ppm_gimbal_typeControl_t type)
{
    static uint32_t count;
    
    if(type == JIG_TEST_CONTROL_LOOP)
    {
        if(ppm_gimbal.control.is_Control_typeLoop_first == false)
        {
            /// set flag control first
            ppm_gimbal.control.is_Control_typeLoop_first = true;
            
            /// setting first Control
            JIG_TEST_ppm_gimbal_setttingFirstControl();
            
            /// set flag control cw
            ppm_gimbal.control.is_Control_typeLoop_CW = true;
        }
        else
        {
            if(ppm_gimbal.control.is_Control_typeLoop_CW == true)
            {
                JIG_TEST_ppm_gimbal_Control_CW();
                
                if(++ count > 150000)
                {
                    count = 0;
                    JIG_TEST_console_write("PPM  ---> control gimbal cw\n");
                }
            }
            
            if(ppm_gimbal.control.is_Control_typeLoop_CCW == true)
            {
                JIG_TEST_ppm_gimbal_Control_CCW();
                
                if(++ count > 150000)
                {
                    count = 0;
                    JIG_TEST_console_write("PPM  ---> control gimbal ccw\n");
                }
            }
        }
    }
    else if(type == JIG_TEST_CONTROL_TEST)
    {
        if(ppm_gimbal.control.is_Control_typeTest_first == false)
        {
            ppm_gimbal.control.is_Control_typeTest_first = true;
            
             /// setting first Control
            JIG_TEST_ppm_gimbal_setttingFirstControl();
        }
        else
        {
            JIG_TEST_ppm_gimbal_Control_CW();
            
            /// check result here
            
        }
    }
}

/** @brief ppm_gimbal_enable
    @return none
*/
void JIG_TEST_ppm_gimbal_enable(bool enable)
{
    if(enable == true)
    {
        ppm_enable();
    }
    else
    {
        ppm_disable();
    }
}

/** @brief jig_test_ppm_gimbal_process
    @return none
*/
void JIG_TEST_ppm_gimbal_process(void)
{
    JIG_TEST_ppm_gimbal_Control(JIG_TEST_CONTROL_TEST);
    
    if(ppm_gimbal.is_setValueChannel == true)
    {
        if(ppm_set_channel(ppm_gimbal.channel, ppm_gimbal.value) == true)
        {
            ppm_gimbal.is_setValueChannel = false;
        }
    }
}

#endif
/**
    @}
*/


/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

