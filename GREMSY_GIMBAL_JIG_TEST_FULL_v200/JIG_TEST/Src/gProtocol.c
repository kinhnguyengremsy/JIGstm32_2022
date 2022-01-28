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
#include "gProtocol.h"
#include "gSerialPort.h"
#include "main.h"
#include "../mavlink_v2/mavlink_helpers.h"
/* Private define-------------------------------------------------------------*/
typedef enum _gimbal_control_t
{
    GIMBAL_CONTROL_MOTOR    = MAV_CMD_USER_1,
    GIMBAL_CONTROL_MODE     = MAV_CMD_USER_2,
} gimbal_control_t;

/* Private Typedef------------------------------------------------------------*/

/* Private variable- ---------------------------------------------------------*/
/*variables interface with gTune*/
mavlink_system_t    mavlink_system = {SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL};
 
/** Gimbal send rate in hz*/
static const uint8_t  sendRate[] = 
{
    [SEND_HEARTBEAT]           = 1,  // send 1hz
    [SEND_STATUS]              = 10,
    [SEND_ATTITUDE]            = 100,
    [SEND_ENCODER_VAL]         = 100,
    [SEND_REQUEST_ATTITUDE]    = 1,
    [SEND_PARAM]               = 100,
    [SEND_RC_CHAN]             = 50,
    [SEND_UUID]                = 1,
    [SEND_DEBUG]               = 10,
    [SEND_RAW_IMU]             = 50,
    [SEND_REQ_DATA_STREAM_ATTI] = 50,
};

#define MAXSENDS (sizeof(sendRate) / sizeof(sendRate[0]))

static uint8_t mavTicks[MAXSENDS];

gSerialPort_t serial_port;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

gSerialPort_t serialPort_COM4;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

gSerialPort_t Console;
/* Private function- ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void gGimbal_Console(uint8_t *str)
{
    gSerialport_write(&Console, str, strlen((char *)str));
}

/**
 * @brief gs_init
 * The function shall initialize independence com channel
 * @param NONE
 * @return NONE
 */
void gProtocol_init(protocol_t *proto)
{
    
    serial_port.zPrivate.uartHandle.hdmarx = &hdma_usart3_rx;
    serial_port.zPrivate.uartHandle.Instance = USART3;
    
    serial_port.isWriteFinish = true;
    
    HAL_UART_Receive_DMA(&huart3, serial_port.zPrivate.readBuffer, 256);
    
    serialPort_COM4.zPrivate.uartHandle.hdmarx = &hdma_uart4_rx;
    serialPort_COM4.zPrivate.uartHandle.Instance = UART4;
    serialPort_COM4.isWriteFinish = true;
    
    Console.zPrivate.uartHandle.Instance = USART1;
    Console.isWriteFinish = true;
    
    HAL_UART_Receive_DMA(&huart4, serialPort_COM4.zPrivate.readBuffer, 256);
}

/*Private function ************************************************************/

/**
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t gProtocol_read_data(mav_state_t *mav)
{
    uint8_t ret = 0;
    
    if(gSerialport_read(&serial_port, 255))
    {
        int i = 0;

        for(i = 0; i < serial_port.bytesToRead; i++)
        {
            uint8_t msg_received;
            /* Paser data */
            msg_received = mavlink_parse_char(ONBOARD_CHANNEL, 
                                serial_port.readBuffer[i],
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
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t gProtocol_serialPort_COM4_read_data(mav_state_t *mav)
{
    uint8_t ret = 0;
    
    if(gSerialport_read(&serialPort_COM4, 255))
    {
        int i = 0;

        for(i = 0; i < serialPort_COM4.bytesToRead; i++)
        {
            uint8_t msg_received;
            /* Paser data */
            msg_received = mavlink_parse_char(  MAVLINK_COMM_2, 
                                                serialPort_COM4.readBuffer[i],
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


/*
  send a buffer out a MAVLink channel
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
        gSerialport_write_list(&serial_port, (uint8_t *)buf);
    }
    else if(chan == MAVLINK_COMM_2)
    {
        /* Send data to serial port 4 */
        gSerialport_write_list(&serialPort_COM4, (uint8_t *)buf);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(Console.zPrivate.uartHandle.Instance == USART1)
    {
        gSerialport_tx_finish(&Console);
    }
    if(serial_port.zPrivate.uartHandle.Instance == USART3)
    {
        gSerialport_tx_finish(&serial_port);
    }
    if(serialPort_COM4.zPrivate.uartHandle.Instance == UART4)
    {
        gSerialport_tx_finish(&serialPort_COM4);
    }
}

/**
 * @brief  this function will calculate the frequent to send data periodically 
 * @param   streamNum: Specific the message 
 * @param   chan:      Specific the channel want to send mav/gtune channel
 * @return return true/false
 */
int send_rate(type_send_t streamNum)
{
    uint8_t	rate = (uint8_t)sendRate[streamNum];
    
    if(rate == 0)
    {
        return 0;
    }
    
    if(mavTicks[streamNum] == 0)
    {
        // we're triggering now, setup the next trigger point
        if(rate  > TELEMETRY_MAVLINK_MAXRATE)
        {
            rate  = TELEMETRY_MAVLINK_MAXRATE;
        }
        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        
        return 1;
    }
    
    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE**********/
