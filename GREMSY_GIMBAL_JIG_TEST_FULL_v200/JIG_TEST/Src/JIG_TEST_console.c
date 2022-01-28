/**
  ******************************************************************************
  * @file .c
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
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
#include "JIG_TEST_console.h"
#include "serialPort.h"

/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

serialPort_t Console;
extern DMA_HandleTypeDef hdma_usart1_rx;

/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
/** @group JIG_TEST_CONSOLE_CONFIGURATION
    @{
*/#ifndef JIG_TEST_CONSOLE_CONFIGURATION
#define JIG_TEST_CONSOLE_CONFIGURATION

/** @brief console_configuration
    @return none
*/
void JIG_TEST_console_configuration(void)
{
    /// init serialPort library
    Console.zPrivate.uartHandle.hdmarx = &hdma_usart1_rx;
    Console.zPrivate.uartHandle.Instance = USART1;
    Console.isWriteFinish = true;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_CONSOLE_PROCESS
    @{
*/#ifndef JIG_TEST_CONSOLE_PROCESS
#define JIG_TEST_CONSOLE_PROCESS

/** @brief console_write
    @return none
*/
void JIG_TEST_console_write(const char *buff, ...)
{
//    serialPort_write(&Console, (uint8_t *)buff, strlen((char *)buff));  
    printf(buff);
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


