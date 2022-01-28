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
#include "timeOut.h"
#include "JIG_TEST_can_dji_.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_console.h"
/* can dji library */
#include "can_dji.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CAN_DJI_BUTTON_C1       = 16, 
    CAN_DJI_BUTTON_C2       = 32,
    CAN_DJI_BUTTON_RECORD   = 4,
    CAN_DJI_BUTTON_SHUTTER  = 2,
    CAN_DJI_BUTTON_PLAYBACK = 8, 
    CAN_DJI_BUTTON_CAM_DIAL = 1,
    
}JIG_TEST_can_dji_remote_button_name_t;

typedef struct
{
    uint8_t count;
    
}JIG_TEST_can_dji_private_debug_variables_t;

typedef struct
{
    uint16_t TxLen;
    
    uint8_t Dial_button;
    
    uint8_t gsCanTxBuff[256];
    uint8_t TxSize;
    uint8_t TxPtr;
    
}JIG_TEST_can_dji_protocol_t;

typedef struct
{
    bool    Tx;
    uint8_t TxCount;
    
    bool    Rx;
    uint8_t RxCount;
    
    bool sFilterConfig;
    bool IT_RxActive;
    bool IT_TxActive;
    bool Start;
    
}JIG_TEST_can_dji_Error_t;

typedef struct
{
    bool is_firstSend;
    bool is_set_rcValue;
    bool is_sendC1Button;
    bool sw_mode;
    
    int16_t rc_tilt;
    int16_t rc_roll;
    int16_t rc_pan;
    
    JIG_TEST_can_dji_remote_button_name_t remoteButton;
    
}JIG_TEST_can_dji_gimbal_message_t;

typedef struct
{
    CAN_TxHeaderTypeDef   TxHeader;
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t               TxData[8];
    uint8_t               RxData[8];
    uint32_t              TxMailbox;
    
    bool canTxFinish;
    
    JIG_TEST_can_dji_private_debug_variables_t debug_variables;
    
    JIG_TEST_can_dji_protocol_t protocol;
    
    JIG_TEST_can_dji_Error_t Error;
    
    JIG_TEST_can_dji_gimbal_message_t gimbal_message;
    
}JIG_TEST_can_dji_private_t;
/* Private define ------------------------------------------------------------*/
#define GIMBAL_CAN_DJI_RC_MAX   2048
#define GIMBAL_CAN_DJI_RC_MID   1024
#define GIMBAL_CAN_DJI_RC_MIN   0

#define GIMBAL_TILT_MOVE_CW     (GIMBAL_CAN_DJI_RC_MAX * 0.25)
#define GIMBAL_TILT_MOVE_CCW    (GIMBAL_CAN_DJI_RC_MAX * 0.75)

#define GIMBAL_ROLL_MOVE_CW     (GIMBAL_CAN_DJI_RC_MAX * 0.25)
#define GIMBAL_ROLL_MOVE_CCW    (GIMBAL_CAN_DJI_RC_MAX * 0.75)

#define GIMBAL_PAN_MOVE_CW      (GIMBAL_CAN_DJI_RC_MAX * 0.25)
#define GIMBAL_PAN_MOVE_CCW     (GIMBAL_CAN_DJI_RC_MAX * 0.75)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan2;
extern JIG_TEST_mavlink_gimbal_t mavlink_gimbal_COM2;

JIG_TEST_can_dji_private_t can_dji_private;

gs_can_message_t    gsCanRxMsg;
gs_can_status_t     gsCanRxState;
gs_can_message_t    gsCanTxMsg;
gs_can_status_t     gsCanTxState;

/** Data can dji */
can_dji_t           dji_system;

static char *str = "\nCAN --->";
/* Private function prototypes -----------------------------------------------*/
/** @brief can_dji_set_rc_channel
    @return 
*/
static void JIG_TEST_can_dji_set_rc_channel(int16_t tilt, int16_t roll, int16_t pan);

/* Private functions ---------------------------------------------------------*/
/** @group CAN_PERIPHERAL_CONFIGURATION
    @{
*/#ifndef CAN_PERIPHERAL_CONFIGURATION
#define CAN_PERIPHERAL_CONFIGURATION

/** @brief can_dji_Error_Handler
    @return none
*/
static void JIG_TEST_can_dji_Error_Handler(void)
{
    /// Error handle
    
    if(can_dji_private.Error.sFilterConfig == true)
    {
        while(1)
        {
            
        } 
    }
    
    if(can_dji_private.Error.Start == true)
    {
        while(1)
        {
            
        } 
    }
    
    if(can_dji_private.Error.IT_TxActive == true)
    {
        while(1)
        {
            
        } 
    }
    
    if(can_dji_private.Error.IT_RxActive == true)
    {
        while(1)
        {
            
        } 
    }
    
    if(can_dji_private.Error.Tx == true)
    {
        /// reset flag Tx error
        can_dji_private.Error.Tx = false;
        
        /// dem so lan truyen loi
        if(get_timeOut(1000, CAN_DJI_TX_ERROR))
        {
            can_dji_private.Error.TxCount ++;
            
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("CAN Write ERROR\n");
        }
        
        if(can_dji_private.Error.TxCount > 50)
        {
            while(1)
            {
                
            } 
        }
    }
    
    if(can_dji_private.Error.Rx == true)
    {
        /// reset flag Rx error
        can_dji_private.Error.Rx = false;
        
        if(get_timeOut(1000, CAN_DJI_RX_ERROR))
        {
            /// dem so lan truyen loi
            can_dji_private.Error.RxCount ++;
        }
        
        if(can_dji_private.Error.RxCount > 100)
        while(1)
        {
            
        } 
    }
}

/** @brief can_filterConfig
    @return none
*/
static void JIG_TEST_can_dji_sFilterConfig(void)
{
    CAN_FilterTypeDef  sFilterConfig;
    
    /*##-- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank            = 0;
    sFilterConfig.FilterMode            = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale           = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh          = 0x0000;
    sFilterConfig.FilterIdLow           = 0x0000;
    sFilterConfig.FilterMaskIdHigh      = 0x0000;
    sFilterConfig.FilterMaskIdLow       = 0x0000;
    sFilterConfig.FilterFIFOAssignment  = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation      = ENABLE;
    sFilterConfig.SlaveStartFilterBank  = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.sFilterConfig = true;
        
        /* Filter configuration Error */
        JIG_TEST_can_dji_Error_Handler();
    }
}


/** @brief can_dji_set_default_param
    @return none
*/
static void JIG_TEST_can_dji_set_default_param(void)
{
    can_dji_private.gimbal_message.rc_tilt = GIMBAL_CAN_DJI_RC_MID;
    can_dji_private.gimbal_message.rc_roll = GIMBAL_CAN_DJI_RC_MID;
    can_dji_private.gimbal_message.rc_pan = GIMBAL_CAN_DJI_RC_MID;
    
    can_dji_private.gimbal_message.remoteButton = CAN_DJI_BUTTON_C1;
    
}

/** @brief can_dji_start_peripheral
    @return none
*/
static void JIG_TEST_can_dji_start_peripheral(void)
{
    /*##-3- Start the CAN peripheral ###########################################*/
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.Start = true;
        
        /* Start Error */
        JIG_TEST_can_dji_Error_Handler();
    }

    /*##-- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.IT_RxActive = true;
        
        /* Notification Error */
        JIG_TEST_can_dji_Error_Handler();
    }
    
    /*##-- Activate CAN TX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.IT_TxActive = true;
        
        /* Notification Error */
        JIG_TEST_can_dji_Error_Handler();
    }
    
    /// set_default_param gimbal message
    JIG_TEST_can_dji_set_default_param();
}

/** @brief can_dji_configuration
    @return none
*/
void JIG_TEST_can_dji_configuration(void)
{
    /// can sFilterConfig
    JIG_TEST_can_dji_sFilterConfig();
    
    /// start peripheral
    JIG_TEST_can_dji_start_peripheral();
    
    //protocol can config
    can_dji_private.canTxFinish  = true;
    gsCanRxState.value.StdId = 0x412;
    gsCanRxState.value.Data = can_dji_private.RxData;
}
#endif
/**
    @}
*/


/** @group CAN_DJI_TRANCIEVER
    @{
*/#ifndef CAN_DJI_TRANCIEVER
#define CAN_DJI_TRANCIEVER

/** @brief  can_dji_send
  * @retval none
  */
static void JIG_TEST_can_dji_send(void *buff, uint8_t index, uint8_t count)
{
    uint8_t i = 0;
    uint8_t *ptr = buff;
    
    /// limit count
    if(count > 8)
    {
        count = 8;
    }
    
    /// reset flag tranmits finish
    can_dji_private.canTxFinish = false;
    
    /// setting can param TxHeader
    can_dji_private.TxHeader.StdId  = 0x412;
    can_dji_private.TxHeader.ExtId  = 0x01;
    can_dji_private.TxHeader.RTR    = CAN_RTR_DATA;
    can_dji_private.TxHeader.IDE    = CAN_ID_STD;
    can_dji_private.TxHeader.DLC    = count;
    
    /// copy data to TxData buffer
    for(i = 0; i < count; i++)
    {
        can_dji_private.TxData[i] = ptr[index + i];
    }
    
    /* Start the Transmission process */
    if (HAL_CAN_AddTxMessage(&hcan2, &can_dji_private.TxHeader
        , can_dji_private.TxData, &can_dji_private.TxMailbox) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.Tx = true;
        
        /* Transmission request Error */
        JIG_TEST_can_dji_Error_Handler();
    }
    else
    {
        /// reset flag Error
        can_dji_private.Error.Tx = false;
        
        /// reset tx Count
        can_dji_private.Error.TxCount = 0;
    }
}

/** @brief can_dji_test_tranmits
    @return none
*/
static void JIG_TEST_can_dji_write(void* buff, uint8_t len)
{
    uint8_t i = 0;
    uint8_t *ptr = (uint8_t*)buff;
    uint8_t count = len;

    if(count > 8) count = 8;

    for( i = 0; i < len; i++)
    {
        can_dji_private.protocol.gsCanTxBuff[i] = ptr[i];
    }

    can_dji_private.canTxFinish        = false;
    can_dji_private.protocol.TxSize    = len;
    can_dji_private.protocol.TxPtr     = count;
    
    /// send Data
    JIG_TEST_can_dji_send(can_dji_private.protocol.gsCanTxBuff, 0, count);
}

#endif
/**
    @}
*/

/** @group JIG_TEST_CAN_DJI_GIMBAL_MESSAGE
    @{
*/#ifndef JIG_TEST_CAN_DJI_GIMBAL_MESSAGE
#define JIG_TEST_CAN_DJI_GIMBAL_MESSAGE

/** @brief can_dji_set_rc_channel
    @return 
*/
static void JIG_TEST_can_dji_set_rc_channel(int16_t tilt, int16_t roll, int16_t pan)
{
    uint16_t len = 0;
    int16_t rc_channel[3];
    
    rc_channel[0] = tilt;
    rc_channel[1] = roll;
    rc_channel[2] = pan;
    
    len = can_dji_cmd_gimbal_control_msg_pack(SRC_DEST_HD_LINK, 
                                              SRC_DEST_GIMBAL, 
                                              &gsCanTxMsg, 
                                              &gsCanTxState,
                                              rc_channel
    );
    
    /// send can_dji_message
    JIG_TEST_can_dji_write(&gsCanTxMsg.delimiter, len);
}

/** @brief can_dji_set_remote_button
    @return 
*/
static void JIG_TEST_can_dji_set_remote_button(JIG_TEST_can_dji_remote_button_name_t button)
{
    uint16_t len = 0;
    uint8_t remoteButton[8];
    
    if(button == CAN_DJI_BUTTON_C1)
    {
        remoteButton[0] = CAN_DJI_BUTTON_C1;
    }
    else if(button == CAN_DJI_BUTTON_C2)
    {
        remoteButton[0] = CAN_DJI_BUTTON_C2;
    }
    else if(button == CAN_DJI_BUTTON_PLAYBACK)
    {
        remoteButton[0] = CAN_DJI_BUTTON_PLAYBACK;
    }
    else if(button == CAN_DJI_BUTTON_RECORD)
    {
        remoteButton[0] = CAN_DJI_BUTTON_RECORD;
    }
    else if(button == CAN_DJI_BUTTON_SHUTTER)
    {
        remoteButton[0] = CAN_DJI_BUTTON_SHUTTER;
    }
    else
    {
        remoteButton[0] = 0;
    }
    
    if(button == CAN_DJI_BUTTON_CAM_DIAL)
    {
        remoteButton[4] = CAN_DJI_BUTTON_CAM_DIAL;
    }
    
    len = can_dji_cmd_remote_control_gimbal_msg_pack(SRC_DEST_HD_LINK, 
                                                      SRC_DEST_GIMBAL, 
                                                      &gsCanTxMsg, 
                                                      &gsCanTxState,
                                                      remoteButton
    );
    
    /// send can_dji_message
    JIG_TEST_can_dji_write(&gsCanTxMsg.delimiter, len);
}

#endif
/**
    @}
*/

/** @group JIG_TEST_CAN_DJI_PROCESS
    @{
*/#ifndef JIG_TEST_CAN_DJI_PROCESS
#define JIG_TEST_CAN_DJI_PROCESS

/** @brief can_dji_first_send
    @return none
*/
static void JIG_TEST_can_dji_first_send(void)
{
    if(can_dji_private.gimbal_message.is_firstSend == false)
    {
        can_dji_private.gimbal_message.is_firstSend = true;
        JIG_TEST_can_dji_set_rc_channel(can_dji_private.gimbal_message.rc_tilt, can_dji_private.gimbal_message.rc_roll, can_dji_private.gimbal_message.rc_pan);
    }
}

/** @brief can_dji_set_move
    @return none
*/
void JIG_TEST_can_dji_set_move(uint16_t tilt_speed, uint16_t pan_speed, bool enable)
{
//    can_dji_private.gimbal_message.is_set_rcValue = enable;
    can_dji_private.gimbal_message.rc_tilt = tilt_speed;
    can_dji_private.gimbal_message.rc_roll = 1024;
    can_dji_private.gimbal_message.rc_pan = pan_speed;
}

/** @brief can_dji_test_protocol
    @return none
*/
static void JIG_TEST_can_dji_test_protocol(void)
{

    static uint16_t count_state;
    static uint32_t count_consolel;
    char *str = "CAN_DJI --->";

    if(++count_state > 5000)
    {
        count_state = 0;
        
//        JIG_TEST_can_dji_set_remote_button(CAN_DJI_BUTTON_CAM_DIAL);
        
        JIG_TEST_can_dji_set_rc_channel(can_dji_private.gimbal_message.rc_tilt, can_dji_private.gimbal_message.rc_roll, can_dji_private.gimbal_message.rc_pan);
        
        if(++ count_consolel > 5000)
        {
            JIG_TEST_console_write(str);
            JIG_TEST_console_write("Can write rc value\n");
        }
    }

//    //////////////////////////////////////////////////////// send Control rc ///////////////////////////////////////////////////////////////
    if(can_dji_private.gimbal_message.is_set_rcValue == true)
    {
        can_dji_private.gimbal_message.is_set_rcValue = false;
        
        
        JIG_TEST_can_dji_set_remote_button(can_dji_private.gimbal_message.remoteButton);
//        JIG_TEST_can_dji_set_rc_channel(can_dji_private.gimbal_message.rc_tilt, can_dji_private.gimbal_message.rc_roll, can_dji_private.gimbal_message.rc_pan);
//        
//        JIG_TEST_can_dji_set_default_param();
    }

}

/** @brief can_dji_process
    @return none
*/
void JIG_TEST_can_dji_process(void)
{
    
    JIG_TEST_can_dji_test_protocol();
    
    JIG_TEST_can_dji_first_send();
}

#endif
/**
    @}
*/

/** @group CAN_INTERRUPT_CALLBACK_HANDLE
    @{
*/#ifndef CAN_INTERRUPT_CALLBACK_HANDLE
#define CAN_INTERRUPT_CALLBACK_HANDLE

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0
        , &can_dji_private.RxHeader
        , can_dji_private.RxData) != HAL_OK)
    {
        /// set flag Error
        can_dji_private.Error.Rx = true;
        
        /* Reception Error */
        JIG_TEST_can_dji_Error_Handler();
    }
    else
    {
        /// reset flag Error
        can_dji_private.Error.Rx = false;
        
        /// reset Rx Count
        can_dji_private.Error.RxCount = 0;
    }
    
    gsCanRxState.value.DLC = can_dji_private.RxHeader.DLC;
    
    /* Check and parse msg */
    if(gs_can_parse(can_dji_private.RxHeader.StdId, &gsCanRxMsg, &gsCanRxState))
    {
        can_dji_handle(&gsCanRxMsg, &dji_system);
    }
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(can_dji_private.protocol.TxPtr < can_dji_private.protocol.TxSize)
    {
        uint8_t dlc = can_dji_private.protocol.TxSize - can_dji_private.protocol.TxPtr;
        
        if(dlc > 8) dlc = 8;
        can_dji_private.protocol.TxPtr += dlc;
        JIG_TEST_can_dji_send(can_dji_private.protocol.gsCanTxBuff, can_dji_private.protocol.TxPtr - dlc, dlc);
    }
    else
    {
        can_dji_private.protocol.TxPtr = 0;
        can_dji_private.canTxFinish = 1;
    }
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

