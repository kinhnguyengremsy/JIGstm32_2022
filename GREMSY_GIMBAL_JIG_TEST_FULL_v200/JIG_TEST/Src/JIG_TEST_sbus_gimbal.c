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
#include "stm32f4xx_hal.h"
#include "JIG_TEST_sbus_gimbal.h"
#include "JIG_TEST_console.h"
#include "timeOut.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    SBUS_CHANNEL_1,
    SBUS_CHANNEL_2,
    SBUS_CHANNEL_3,
    SBUS_CHANNEL_4,
    SBUS_CHANNEL_5,
    SBUS_CHANNEL_6,
    SBUS_CHANNEL_7,
    SBUS_CHANNEL_8,
    SBUS_CHANNEL_9,
    SBUS_CHANNEL_10,
    SBUS_CHANNEL_11,
    SBUS_CHANNEL_12,
    SBUS_CHANNEL_13,
    SBUS_CHANNEL_14,
    SBUS_CHANNEL_15,
    SBUS_CHANNEL_16,
    
}JIG_TEST_sbus_gimbal_channel_t;

typedef enum
{
    JIG_TEST_CONTROL_LOOP,
    JIG_TEST_CONTROL_TEST,
    
}JIG_TEST_sbus_gimbal_typeControl_t;

typedef struct
{
    bool is_Control_typeLoop_first;
    bool is_Control_typeLoop_CW;
    bool is_Control_typeLoop_CCW;
    
    bool is_Control_typeTest_first;
    bool is_Control_typeTest_process;
    bool is_Control_typeTest_result;
    
}JIG_TEST_sbus_gimbal_Control_t;

typedef struct
{
    bool                            is_setValueChannel;
    JIG_TEST_sbus_gimbal_channel_t  channel;
    int16_t                         value;
    
    JIG_TEST_sbus_gimbal_Control_t  control;
    
}JIG_TEST_sbus_gimbal_t;
/* Private define ------------------------------------------------------------*/
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MAX  1000
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MID  -22
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MIN  -1000

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_MODE           SBUS_CHANNEL_7
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT           SBUS_CHANNEL_2
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_ROLL           SBUS_CHANNEL_8
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_PAN            SBUS_CHANNEL_1
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED     SBUS_CHANNEL_3
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_PAN_SPEED      SBUS_CHANNEL_3

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_OFF         -500
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_LOCK        JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MID
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_FOLLOW      500

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_VALUE_CW          -500
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_VALUE_CCW         500

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_ROLL_VALUE_CW          JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MID 
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_ROLL_VALUE_CCW         JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MID

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_VALUE_CW          -500
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_VALUE_CCW         500

#define JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_SPEED_VALUE      500
#define JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_SPEED_VALUE       JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_SPEED_VALUE
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const uint8_t _sbusHeader = 0x0F;
static const uint8_t _sbusFooter = 0x00;
int16_t channels_w[16];

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_tx;

JIG_TEST_sbus_gimbal_t sbus_gimbal;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/** @group JIG_TEST_SBUS_WRITE
    @{
*/#ifndef JIG_TEST_SBUS_WRITE
#define JIG_TEST_SBUS_WRITE

/** @brief sbus_write_packet
    @return none
*/
static void JIG_TEST_sbus_write_packet(void)
{
    static uint8_t packet[25];
    /* assemble the SBUS packet */
    // SBUS header
    packet[0] = _sbusHeader;
    // 16 channels of 11 bit data

    packet[1] = (uint8_t) ((-(channels_w[0] - 1024) & 0x07FF));
    packet[2] = (uint8_t) ((-(channels_w[0] - 1024) & 0x07FF)>>8 | (channels_w[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((-(channels_w[1] - 1024) & 0x07FF)>>5 | (channels_w[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((-(channels_w[2] - 1024) & 0x07FF)>>2);
    packet[5] = (uint8_t) ((-(channels_w[2] - 1024) & 0x07FF)>>10 | (channels_w[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((-(channels_w[3] - 1024) & 0x07FF)>>7 | (channels_w[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((-(channels_w[4] - 1024) & 0x07FF)>>4 | (channels_w[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((-(channels_w[5] - 1024) & 0x07FF)>>1);
    packet[9] = (uint8_t) ((-(channels_w[5] - 1024) & 0x07FF)>>9 | (channels_w[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((-(channels_w[6] - 1024) & 0x07FF)>>6 | (channels_w[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((-(channels_w[7] - 1024) & 0x07FF)>>3);
    packet[12] = (uint8_t) ((-(channels_w[8] - 1024) & 0x07FF));
    packet[13] = (uint8_t) ((-(channels_w[8] - 1024) & 0x07FF)>>8 | (channels_w[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((-(channels_w[9] - 1024) & 0x07FF)>>5 | (channels_w[10] & 0x07FF)<<6);
    packet[15] = (uint8_t) ((-(channels_w[10] - 1024) & 0x07FF)>>2);
    packet[16] = (uint8_t) ((-(channels_w[10] - 1024) & 0x07FF)>>10 | (channels_w[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((-(channels_w[11] - 1024) & 0x07FF)>>7 | (channels_w[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((-(channels_w[12] - 1024) & 0x07FF)>>4 | (channels_w[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((-(channels_w[13] - 1024) & 0x07FF)>>1);
    packet[20] = (uint8_t) ((-(channels_w[13] - 1024) & 0x07FF)>>9 | (channels_w[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((-(channels_w[14] - 1024) & 0x07FF)>>6 | (channels_w[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((-(channels_w[15] - 1024) & 0x07FF)>>3);

    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = _sbusFooter;

    // write packet
    if(get_timeOut(20, SBUS_WRITE_PACKET))
    {
        HAL_UART_Transmit_DMA(&huart6, packet, 25);
    }
}

/** @brief sbus_gimbal_process
    @return none
*/
static void JIG_TEST_sbus_gimbal_set_channel_value(JIG_TEST_sbus_gimbal_channel_t channel, int16_t value)
{
    if(channel == SBUS_CHANNEL_1)
    {
        channels_w[SBUS_CHANNEL_1] = value;
    }
    else if(channel == SBUS_CHANNEL_2)
    {
        channels_w[SBUS_CHANNEL_2] = value;
    }
    else if(channel == SBUS_CHANNEL_3)
    {
        channels_w[SBUS_CHANNEL_3] = value;
    }
    else if(channel == SBUS_CHANNEL_4)
    {
        channels_w[SBUS_CHANNEL_4] = value;
    }
    else if(channel == SBUS_CHANNEL_5)
    {
        channels_w[SBUS_CHANNEL_5] = value;
    }
    else if(channel == SBUS_CHANNEL_6)
    {
        channels_w[SBUS_CHANNEL_6] = value;
    }
    else if(channel == SBUS_CHANNEL_7)
    {
        channels_w[SBUS_CHANNEL_7] = value;
    }
    else if(channel == SBUS_CHANNEL_8)
    {
        channels_w[SBUS_CHANNEL_8] = value;
    }
    else if(channel == SBUS_CHANNEL_9)
    {
        channels_w[SBUS_CHANNEL_9] = value;
    }
    else if(channel == SBUS_CHANNEL_10)
    {
        channels_w[SBUS_CHANNEL_10] = value;
    }
    else if(channel == SBUS_CHANNEL_11)
    {
        channels_w[SBUS_CHANNEL_11] = value;
    }
    else if(channel == SBUS_CHANNEL_12)
    {
        channels_w[SBUS_CHANNEL_12] = value;
    }
    else if(channel == SBUS_CHANNEL_13)
    {
        channels_w[SBUS_CHANNEL_13] = value;
    }
    else if(channel == SBUS_CHANNEL_14)
    {
        channels_w[SBUS_CHANNEL_14] = value;
    }
    else if(channel == SBUS_CHANNEL_15)
    {
        channels_w[SBUS_CHANNEL_15] = value;
    }
    else if(channel == SBUS_CHANNEL_16)
    {
        channels_w[SBUS_CHANNEL_16] = value;
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_SBUS_GIMBAL_PROCESS
    @{
*/#ifndef JIG_TEST_SBUS_GIMBAL_PROCESS
#define JIG_TEST_SBUS_GIMBAL_PROCESS

/** @brief sbus_gimbal_setttingFirstControl
    @return none
*/
static void JIG_TEST_sbus_gimbal_setttingFirstControl(void)
{
    JIG_TEST_sbus_gimbal_channel_t channel;
    int16_t value;
    
    /// setting channel mode
    channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_MODE;
    value = JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_LOCK;
    JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    
    /// setting channel tilt
    channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT;
    value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_VALUE_CW;
    JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    
    /// setting channel roll
    channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_ROLL;
    value = JIG_TEST_SBUS_GIMBAL_CHANNEL_ROLL_VALUE_CW;
    JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    
    /// setting channel pan
    channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_PAN;
    value = JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_VALUE_CW;
    JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    
    /// setting channel tilt speed
    channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
    value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
    JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
}

/** @brief sbus_gimbal_Control_CW
    @return none
*/
static void JIG_TEST_sbus_gimbal_Control_CW(void)
{
    /// kiem tra thoi gian sau 6s chuyen sang control ccw
    if(get_timeOut(5000, SBUS_CONTROL_TYPE_LOOP_CW))
    {
        /// reset flag control cw
        sbus_gimbal.control.is_Control_typeLoop_CW = false;
            
        /// set flag control ccw
        sbus_gimbal.control.is_Control_typeLoop_CCW = true;
        
        /// setting value sbus channel for ccw
        JIG_TEST_sbus_gimbal_channel_t channel;
        int16_t value;
        
        /// setting channel mode
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_MODE;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_FOLLOW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel tilt
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_VALUE_CCW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel roll
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_ROLL;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_ROLL_VALUE_CCW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel pan
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_PAN;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_VALUE_CCW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel tilt speed
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    }
}

/** @brief sbus_gimbal_Control_CCW
    @return none
*/
static void JIG_TEST_sbus_gimbal_Control_CCW(void)
{
    /// kiem tra thoi gian sau 6s chuyen sang control ccw
    if(get_timeOut(5000, SBUS_CONTROL_TYPE_LOOP_CW))
    {
        /// reset flag control ccw
        sbus_gimbal.control.is_Control_typeLoop_CCW = false;
            
        /// set flag control cw
        sbus_gimbal.control.is_Control_typeLoop_CW = true;
        
        /// setting value sbus channel for cw
        JIG_TEST_sbus_gimbal_channel_t channel;
        int16_t value;
        
        /// setting channel mode
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_MODE;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_MODE_VALUE_LOCK;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel tilt
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_VALUE_CW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel roll
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_ROLL;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_ROLL_VALUE_CW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel pan
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_PAN;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_PAN_VALUE_CW;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
        
        /// setting channel tilt speed
        channel = JIG_TEST_SBUS_GIMBAL_CHANNEL_DEFAULT_TILT_SPEED;
        value = JIG_TEST_SBUS_GIMBAL_CHANNEL_TILT_SPEED_VALUE;
        JIG_TEST_sbus_gimbal_set_channel_value(channel, value);
    }
}

/** @brief sbus_gimbal_process
    @return none
*/
static void JIG_TEST_sbus_gimbal_Control(JIG_TEST_sbus_gimbal_typeControl_t type)
{
    static uint32_t count;
    
    if(type == JIG_TEST_CONTROL_LOOP)
    {
        if(sbus_gimbal.control.is_Control_typeLoop_first == false)
        {
            /// set flag control first
            sbus_gimbal.control.is_Control_typeLoop_first = true;
            
            /// setting first Control
            JIG_TEST_sbus_gimbal_setttingFirstControl();
            
            /// set flag control cw
            sbus_gimbal.control.is_Control_typeLoop_CW = true;
        }
        else
        {
            if(sbus_gimbal.control.is_Control_typeLoop_CW == true)
            {
                JIG_TEST_sbus_gimbal_Control_CCW();
                
                if(++ count > 150000)
                {
                    count = 0;
                    JIG_TEST_console_write("SBUS  ---> control gimbal cCw\n");
                }
            }
            
            if(sbus_gimbal.control.is_Control_typeLoop_CCW == true)
            {
                JIG_TEST_sbus_gimbal_Control_CW();
                
                if(++ count > 150000)
                {
                    count = 0;
                    JIG_TEST_console_write("SBUS  ---> control gimbal cw\n");
                }
            }
        }
    }
    else if(type == JIG_TEST_CONTROL_TEST)
    {
        if(sbus_gimbal.control.is_Control_typeTest_first == false)
        {
            sbus_gimbal.control.is_Control_typeTest_first = true;
            
             /// setting first Control
            JIG_TEST_sbus_gimbal_setttingFirstControl();
        }
        else
        {
            JIG_TEST_sbus_gimbal_Control_CW();
            
            /// check result here
            
        }
    }
}

/** @brief sbus_gimbal_process
    @return none
*/
void JIG_TEST_sbus_gimbal_process(void)
{
    JIG_TEST_sbus_write_packet();
    
    JIG_TEST_sbus_gimbal_Control(JIG_TEST_CONTROL_LOOP);
    
    if(sbus_gimbal.is_setValueChannel == true)
    {
        sbus_gimbal.is_setValueChannel = false;
        JIG_TEST_sbus_gimbal_set_channel_value(sbus_gimbal.channel, sbus_gimbal.value);
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_SBUS_GIMBAL_CONFIGURATION
    @{
*/#ifndef JIG_TEST_SBUS_GIMBAL_CONFIGURATION
#define JIG_TEST_SBUS_GIMBAL_CONFIGURATION

/** @brief sbus_gimbal_enable
    @return none
*/
void JIG_TEST_sbus_gimbal_enable(bool enable)
{
    if(enable == true)
    {
        
    }
    else
    {
        HAL_UART_DeInit(&huart6);
        
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
        
        HAL_DMA_DeInit(&hdma_usart6_tx);
    }
}

/** @brief sbus_gimbal_set_default_channel
    @return none
*/
static void JIG_TEST_sbus_gimbal_set_default_channel(void)
{
    uint8_t i = 0;
    
    for(i = 0; i < 16; i++)
    {
        channels_w[i] = JIG_TEST_SBUS_GIMBAL_CHANNEL_VALUE_MID;
    }
}

/** @brief sbus_gimbal_configuration
    @return none
*/
void JIG_TEST_sbus_gimbal_configuration(void)
{
    JIG_TEST_sbus_gimbal_set_default_channel();
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

