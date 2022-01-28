/**
******************************************************************************
* @file serialPort1.h
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
    uint8_t     writeBuffer[1000];
    uint8_t     writeBufferTemp[1000];
    uint8_t     writeBufferPtr;
}serialPort_private_t;

typedef struct
{
    uint32_t time;
    
    uint8_t readBuffer[256];
    uint8_t bytesToRead;
    uint8_t readBufferSize;
    
    bool isWriteFinish;
    
    serialPort_private_t   zPrivate;
}serialPort_t;
/* Exported constants------------------------------------------------------------------------------*/
/* Exported macro------------------------------------------------------------------------------*/

/* Exported functions------------------------------------------------------------------------------*/
bool serialPort_read(serialPort_t* serial, uint16_t buffsize);
void serialPort_tx_finish(serialPort_t* serial);
void serialPort_write(serialPort_t* serial, uint8_t* buff, uint8_t len);
void serialPort_write_list(serialPort_t* serial, void* buff,...);
#ifdef __cplusplus
}
#endif
#endif /* __SERIAL_PORT_H_ */
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


