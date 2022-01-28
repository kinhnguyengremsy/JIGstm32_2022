/**
******************************************************************************
* @file gSerialPort1.h
* @author 
* @version 
* @date 
* @brief This file contains all the functions prototypes
for the Gremsy
* common firmware library.
*
************************************************************
******************
* @par
* COPYRIGHT NOTICE: (c) 2016 Gremsy. All rights reserved.
*
* The information contained herein is confidential
* property of Company. The use, copying, transfer or
* disclosure of such information is prohibited except
* by express written agreement with Company.
*
************************************************************
******************
*/
/* Define to prevent recursive inclusion
------------------------------------------------------------------------------*/
#ifndef __SERIAL_PORT_H_
#define __SERIAL_PORT_H_
#ifdef __cplusplus
extern "C" {
#endif
/* Includes------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdarg.h"
/* Exported types------------------------------------------------------------------------------*/
typedef struct
{
    UART_HandleTypeDef  uartHandle;
    DMA_HandleTypeDef   dmaTx;
    DMA_HandleTypeDef   dmaRx;
    uint16_t    uartDataCount;
    uint8_t     readBuffer[256];
    uint8_t     writeBuffer[256];
    uint8_t     writeBufferTemp[256];
    uint8_t     writeBufferPtr;
}gSerialPort_private_t;

typedef struct
{
    uint32_t time;
    
    uint8_t readBuffer[256];
    uint8_t bytesToRead;
    uint8_t readBufferSize;
    
    bool isWriteFinish;
    
    gSerialPort_private_t   zPrivate;
}gSerialPort_t;
/* Exported constants------------------------------------------------------------------------------*/
/* Exported macro------------------------------------------------------------------------------*/

/* Exported functions------------------------------------------------------------------------------*/
bool gSerialport_read(gSerialPort_t* serial, uint16_t buffsize);
void gSerialport_tx_finish(gSerialPort_t* serial);
void gSerialport_write(gSerialPort_t* serial, uint8_t* buff, uint8_t len);
void gSerialport_write_list(gSerialPort_t* serial, void* buff,...);
#ifdef __cplusplus
}
#endif
#endif /* __SERIAL_PORT_H_ */
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


