/**
  ******************************************************************************
  * @file JIG_TEST_rtc.c
  * @author  Gremsy Team
  * @version v2.0.1
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
#include "JIG_TEST_rtc.h"
#include "main.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_RTC_CONFIGURATION
    @{
*/#ifndef JIG_TEST_RTC_CONFIGURATION
#define JIG_TEST_RTC_CONFIGURATION

/** @brief rtc_configuration
    @return rtc_configuration
*/
void JIG_TEST_rtc_configuration(void)
{

}


#endif
/**
    @}
*/

/** @group JIG_TEST_RTC_CALENDAR
    @{
*/#ifndef JIG_TEST_RTC_CALENDAR
#define JIG_TEST_RTC_CALENDAR

/** @brief rtc_storage_register_value
    @return true : storage complete
            false : running storage
*/
bool JIG_TEST_rtc_storage_register_value(uint32_t value, uint32_t rtc_backup_register_addr)
{
    bool ret = false;
    
    /// set value to backup register
    LL_RTC_BAK_SetRegister(RTC, rtc_backup_register_addr, value);
    
    /// get & compare value from backup register
    if(LL_RTC_BAK_GetRegister(RTC, rtc_backup_register_addr) == value)
    {
        ret = true;
    }
    
    return ret;
}

/** @brief rtc_get_value_from_backup_register
    @return uint32_t
*/
uint32_t JIG_TEST_rtc_get_value_from_backup_register(uint32_t rtc_backup_register_addr)
{
    uint32_t value = 0;
    
    /// get & compare value from backup register
    value = LL_RTC_BAK_GetRegister(RTC, rtc_backup_register_addr);
    
    return value;
}


#endif
/**
    @}
*/

/** @group JIG_TEST_RTC_PROCESS
    @{
*/#ifndef JIG_TEST_RTC_PROCESS
#define JIG_TEST_RTC_PROCESS

/** @brief rtc_process
    @return rtc_process
*/
void JIG_TEST_rtc_process(void)
{

}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


