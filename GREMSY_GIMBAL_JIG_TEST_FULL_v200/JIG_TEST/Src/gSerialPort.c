/**
  ******************************************************************************
  * @file .c
  * @author 
  * @version 
  * @date 
  * @brief
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2016 Gremsy.
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
#include "gSerialPort.h"
#include "string.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
uint16_t gSerialPort_read_byte_count(gSerialPort_t* serial)
{
    if(serial->readBufferSize == 0)
    {
        uint16_t countCur = __HAL_DMA_GET_COUNTER(serial->zPrivate.uartHandle.hdmarx);
        return 256 - countCur;
    }
    else
    {
        return serial->readBufferSize - __HAL_DMA_GET_COUNTER(serial->zPrivate.uartHandle.hdmarx);
    }
}

bool gSerialport_read(gSerialPort_t* serial, uint16_t buffsize)
{
    int16_t dataCnt =   gSerialPort_read_byte_count(serial);
    int16_t n  =     dataCnt - serial->zPrivate.uartDataCount;
    uint16_t i =    0;
    uint16_t j =    0;

    //reset byte to read
    serial->bytesToRead = 0;

    if(n<0)
    {
        n += buffsize;
    }

    if(n != 0){
        for(i = 0; i < n; i++){
            j = i + serial->zPrivate.uartDataCount;
            
            if(j >= buffsize) j -= buffsize;
            
            serial->readBuffer[i] = serial->zPrivate.readBuffer[j];
        }
        
        serial->zPrivate.uartDataCount = dataCnt;
        serial->bytesToRead = n;
        
        return true;
    }

    return false;
}


void gSerialport_send(gSerialPort_t* serial)
{
    if(serial->zPrivate.uartHandle.Instance == USART1)
    {
        HAL_UART_Transmit_DMA(&huart1, serial->zPrivate.writeBuffer, serial->zPrivate.writeBufferPtr);
    }
    else if(serial->zPrivate.uartHandle.Instance == USART3)
    {
        HAL_UART_Transmit_DMA(&huart3, serial->zPrivate.writeBuffer, serial->zPrivate.writeBufferPtr);
    }
    else if(serial->zPrivate.uartHandle.Instance == UART4)
    {
        HAL_UART_Transmit_DMA(&huart4, serial->zPrivate.writeBuffer, serial->zPrivate.writeBufferPtr);
    }
}

void gSerialport_write(gSerialPort_t* serial, uint8_t* buff, uint8_t len)
{
    uint16_t i = 0;

    if(serial->isWriteFinish == true)
     {
        for( i = 0; i < serial->zPrivate.writeBufferPtr; i++)
        {
            serial->zPrivate.writeBuffer[i] = serial->zPrivate.writeBufferTemp[i];
        }
        
        for( i = 0; i < len; i++)
        {
            serial->zPrivate.writeBuffer[serial->zPrivate.writeBufferPtr++] = buff[i];
        }
        
        gSerialport_send(serial);
        
        serial->zPrivate.writeBufferPtr = 0;
        serial->isWriteFinish = false;
    }
    else
    {
        for( i = 0; i < len; i++)
        {
            serial->zPrivate.writeBufferTemp[serial->zPrivate.writeBufferPtr++] = buff[i];
        }
    }
}

void gSerialport_write_list(gSerialPort_t* serial, void* buff,...)
{
    int len = 0;
    uint8_t* ptr = buff;

    va_list list;
    va_start(list, buff);
    len = va_arg(list, int);    

    if(len < 256 && len > 0)    gSerialport_write(serial, ptr, len);
    else                        gSerialport_write(serial, ptr, strlen((char*)ptr));
}

// write finish
void gSerialport_tx_finish(gSerialPort_t* serial)
{
    if(serial->zPrivate.writeBufferPtr != 0)
    {
        uint8_t i = 0;
        
        for( i = 0; i < serial->zPrivate.writeBufferPtr; i++){
            serial->zPrivate.writeBuffer[i] = serial->zPrivate.writeBufferTemp[i];
        }
        
        gSerialport_send(serial);
        serial->zPrivate.writeBufferPtr = 0;
    }
    else{
        serial->isWriteFinish = true;
    }
}

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


