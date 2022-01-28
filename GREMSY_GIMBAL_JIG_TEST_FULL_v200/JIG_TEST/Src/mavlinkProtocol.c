/*******************************************************************************
 * Copyright (c) 2021, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gGimbal.c
 * @author  The GremsyCo
 * @version V2.0.0
 * @date    2021
 * @brief   This file contains expand of hal_dma
 *
 ******************************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "mavlinkProtocol.h"
#include "serialPort.h"
#include "main.h"
#include "../mavlink_v2/mavlink_helpers.h"
/* Private define-------------------------------------------------------------*/

/* Private typedef------------------------------------------------------------*/

/* Private variable- ---------------------------------------------------------*/
/*variables interface with gTune*/
mavlink_system_t    mavlink_system = {SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL};

serialPort_t serial_port3;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

serialPort_t serial_port4;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

serialPort_t serial_port5;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;

extern serialPort_t Console;
/* Private function- ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @group MAVLINK_PROTOCOL_CONFIGURATION
    @{
*/#ifndef MAVLINK_PROTOCOL_CONFIGURATION
#define MAVLINK_PROTOCOL_CONFIGURATION

/** @brief serialPort3_init
    @return none
*/
void mavlinkProtocol_serialPort3_init(void)
{
    /// init serialPort library
    serial_port3.zPrivate.uartHandle.hdmarx = &hdma_usart3_rx;
    serial_port3.zPrivate.uartHandle.Instance = USART3;
    serial_port3.isWriteFinish = true;
    
    /// init uart3 reciver hardware
    HAL_UART_Receive_DMA(&huart3, serial_port3.zPrivate.readBuffer, 256);
}

/** @brief serialPort3_Deinit
    @return none
*/
void mavlinkProtocol_serialPort3_Deinit(void)
{
    /// Deinit serialPort library
    memset(&serial_port3, 0, sizeof(serialPort_t));
}

/** @brief serialPort4_init
    @return none
*/
void mavlinkProtocol_serialPort4_init(void)
{
    /// init serialPort library
    serial_port4.zPrivate.uartHandle.hdmarx = &hdma_uart4_rx;
    serial_port4.zPrivate.uartHandle.Instance = UART4;
    serial_port4.isWriteFinish = true;
    
    /// init uart4 reciver hardware
    HAL_UART_Receive_DMA(&huart4, serial_port4.zPrivate.readBuffer, 256);
}

/** @brief serialPort4_Deinit
    @return none
*/
void mavlinkProtocol_serialPort4_Deinit(void)
{
    /// Deinit serialPort library
    memset(&serial_port4, 0, sizeof(serialPort_t));
}

/** @brief serialPort5_init
    @return none
*/
void mavlinkProtocol_serialPort5_init(void)
{
    /// init serialPort library
    serial_port5.zPrivate.uartHandle.hdmarx = &hdma_uart5_rx;
    serial_port5.zPrivate.uartHandle.Instance = UART5;
    serial_port5.isWriteFinish = true;
    
    /// init uart3 reciver hardware
    HAL_UART_Receive_DMA(&huart5, serial_port5.zPrivate.readBuffer, 256);
}

/** @brief serialPort5_Deinit
    @return none
*/
void mavlinkProtocol_serialPort5_Deinit(void)
{
    /// Deinit serialPort library
    memset(&serial_port5, 0, sizeof(serialPort_t));
}

/**
 * @brief gs_init
 * The function shall initialize independence com channel
 * @param NONE
 * @return NONE
 */
void mavlinkProtocol_init(void)
{
    mavlinkProtocol_serialPort3_init();

    mavlinkProtocol_serialPort4_init();
    
    mavlinkProtocol_serialPort5_init();
}


#endif
/**
    @}
*/

/** @group MAVLINK_PROTOCOL_READ_DATA
    @{
*/#ifndef MAVLINK_PROTOCOL_READ_DATA
#define MAVLINK_PROTOCOL_READ_DATA

/**
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t mavlinkProtocol_serialPort3_readData(mav_state_t *mav)
{
    uint8_t ret = 0;
    
    if(serialPort_read(&serial_port3, 256))
    {
        int i = 0;

        for(i = 0; i < serial_port3.bytesToRead; i++)
        {
            uint8_t msg_received;
            /* Paser data */
            msg_received = mavlink_parse_char(  ONBOARD_CHANNEL, 
                                                serial_port3.readBuffer[i],
                                                &mav->rxmsg,
                                                &mav->status);
            /** 0 if no message could be decoded or bad CRC, 1 on good message and CRC*/
            if(msg_received)
            {
                ret = 1;
            }
            else 
            {
                ret = 0;
            }
        }
    }
    else
    {
        
    }
    return ret;
}


/**
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t mavlinkProtocol_serialPort4_readData(mav_state_t *mav)
{
    uint8_t ret = 0;
    
    if(serialPort_read(&serial_port4, 256))
    {
        int i = 0;

        for(i = 0; i < serial_port4.bytesToRead; i++)
        {
            uint8_t msg_received;
            /* Paser data */
            msg_received = mavlink_parse_char(  MAVLINK_COMM_2, 
                                                serial_port4.readBuffer[i],
                                                &mav->rxmsg,
                                                &mav->status);
            /** 0 if no message could be decoded or bad CRC, 1 on good message and CRC*/
            if(msg_received)
            {
                ret = 1;
            }
            else 
            {
                ret = 0;
            }
        }
    }
    return ret;
}

/**
 * @brief  This is function read data from raspberry
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t mavlinkProtocol_serialPort5_readData(mav_state_t *mav)
{
    uint8_t ret = 0;
    
    if(serialPort_read(&serial_port5, 256))
    {
        int i = 0;

        for(i = 0; i < serial_port5.bytesToRead; i++)
        {
            uint8_t msg_received;
            /* Paser data */
            msg_received = mavlink_parse_char(  MAVLINK_COMM_3, 
                                                serial_port5.readBuffer[i],
                                                &mav->rxmsg,
                                                &mav->status);
            /** 0 if no message could be decoded or bad CRC, 1 on good message and CRC*/
            if(msg_received)
            {
                ret = 1;
            }
            else 
            {
                ret = 0;
            }
        }
    }
    return ret;
}

#endif
/**
    @}
*/

/** @group MAVLINK_PROTOCOL_SEND_DATA
    @{
*/#ifndef MAVLINK_PROTOCOL_SEND_DATA
#define MAVLINK_PROTOCOL_SEND_DATA

/** @brief send a buffer out a MAVLink channel
    @param[in] chan
    @param[in] buf
    @param[in] len
    @return none
*/
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (!valid_channel(chan))
    {
        return;
    }
    if(chan == ONBOARD_CHANNEL)
    {
        /* Send data to serial port 2*/
        serialPort_write_list(&serial_port3, (uint8_t *)buf);
    }
    else if(chan == MAVLINK_COMM_2)
    {
        /* Send data to serial port 4 */
        serialPort_write_list(&serial_port4, (uint8_t *)buf);
    }
    else if(chan == MAVLINK_COMM_3)
    {
        /* Send data to serial port 4 */
        serialPort_write_list(&serial_port5, (uint8_t *)buf);
    }
}


#endif
/**
    @}
*/

/** @group MAVLINK_PROTOCOL_SERIAL_TRANMITS_INTERRUPT_CALLBACK
    @{
*/#ifndef MAVLINK_PROTOCOL_SERIAL_TRANMITS_INTERRUPT_CALLBACK
#define MAVLINK_PROTOCOL_SERIAL_TRANMITS_INTERRUPT_CALLBACK

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(Console.zPrivate.uartHandle.Instance == USART1)
    {
        serialPort_tx_finish(&Console);
    }
    if(serial_port3.zPrivate.uartHandle.Instance == USART3)
    {
        serialPort_tx_finish(&serial_port3);
    }
    if(serial_port4.zPrivate.uartHandle.Instance == UART4)
    {
        serialPort_tx_finish(&serial_port4);
    }
    if(serial_port5.zPrivate.uartHandle.Instance == UART5)
    {
        serialPort_tx_finish(&serial_port5);
    }
}


#endif
/**
    @}
*/

/*********** Portions COPYRIGHT 2021 Gremsy.Co., Ltd.*****END OF FILE**********/
