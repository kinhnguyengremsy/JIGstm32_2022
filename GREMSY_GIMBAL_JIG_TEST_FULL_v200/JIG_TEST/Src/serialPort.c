/**
  ******************************************************************************
  * @file serialPort.c
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
#include "serialPort.h"
#include "string.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
uint16_t serialPort_read_byte_count(serialPort_t* serial)
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

bool serialPort_read(serialPort_t* serial, uint16_t buffsize)
{
    int16_t dataCnt =   serialPort_read_byte_count(serial);
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


void serialPort_send(serialPort_t* serial)
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
    else if(serial->zPrivate.uartHandle.Instance == UART5)
    {
        HAL_UART_Transmit_DMA(&huart5, serial->zPrivate.writeBuffer, serial->zPrivate.writeBufferPtr);
    }
}

void serialPort_write(serialPort_t* serial, uint8_t* buff, uint8_t len)
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
        
        serialPort_send(serial);
        
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

void serialPort_write_list(serialPort_t* serial, void* buff,...)
{
    int len = 0;
    uint8_t* ptr = buff;

    va_list list;
    va_start(list, buff);
    len = va_arg(list, int);    

    if(len < 256 && len > 0)    serialPort_write(serial, ptr, len);
    else                        serialPort_write(serial, ptr, strlen((char*)ptr));
}

// write finish
void serialPort_tx_finish(serialPort_t* serial)
{
    if(serial->zPrivate.writeBufferPtr != 0)
    {
        uint8_t i = 0;
        
        for( i = 0; i < serial->zPrivate.writeBufferPtr; i++){
            serial->zPrivate.writeBuffer[i] = serial->zPrivate.writeBufferTemp[i];
        }
        
        serialPort_send(serial);
        serial->zPrivate.writeBufferPtr = 0;
    }
    else{
        serial->isWriteFinish = true;
    }
}

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


