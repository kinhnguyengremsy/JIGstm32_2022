/**
  ******************************************************************************
  * @file timeOut.c
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
#include "timeOut.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
static uint32_t time_out[256];

extern TIM_HandleTypeDef htim5;
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group TIME_OUT_PROCESS
    @{
*/#ifndef TIME_OUT_PROCESS
#define TIME_OUT_PROCESS

/** @brief timeOut_get_ms
    @return 1ms
*/
static uint32_t timeOut_get_ms(void)
{
    return (__HAL_TIM_GET_COUNTER(&htim5) / 1000);
}

/** @brief get_timeOut
    @return true : het thoi gian
            false : dang tinh thoi gian
*/
bool get_timeOut(uint32_t time, timeOut_task_t task)
{
    bool ret = false;
    uint32_t temp = timeOut_get_ms() - time_out[(uint32_t)task];

    if(temp >= time)
    {
        time_out[(uint32_t)task] = timeOut_get_ms();
        
        ret = true;
    }

    return ret;
}

/** @brief calculator_get_time_us
    @return time_us
*/
uint32_t calculator_get_time_us(uint32_t *time)
{
    return (__HAL_TIM_GET_COUNTER(&htim5) - *time);
}

/** @brief calculator_get_time_ms
    @return time_ms
*/
uint32_t calculator_get_time_ms(uint32_t *time)
{
    return (__HAL_TIM_GET_COUNTER(&htim5) - *time) / 1000;
}

/** @brief calculator_reset_time
    @return time_ms
*/
void calculator_reset_time(uint32_t *time)
{
    *time = __HAL_TIM_GET_COUNTER(&htim5);
}

/** @brief reset_timeOut
    @return none
*/
void reset_timeOut(timeOut_task_t task)
{
    time_out[(uint32_t)task] = __HAL_TIM_GET_COUNTER(&htim5);
}


#endif
/**
    @}
*/

/** @group TIME_OUT_CONFIGURATION
    @{
*/#ifndef TIME_OUT_CONFIGURATION
#define TIME_OUT_CONFIGURATION

/** @brief timeOut_configuration
    @return none
*/
void timeOut_configuration(void)
{
    HAL_TIM_Base_Start(&htim5);
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


