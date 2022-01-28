/** 
  ******************************************************************************
  * @file    JIG_TEST_rtc.h
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __JIG_TEST_RTC_H
#define __JIG_TEST_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief rtc_configuration
    @return rtc_configuration
*/
void JIG_TEST_rtc_configuration(void);

/** @brief rtc_storage_register_value
    @return true : storage complete
            false : running storage
*/
bool JIG_TEST_rtc_storage_register_value(uint32_t value, uint32_t rtc_backup_register_addr);

/** @brief rtc_get_value_from_backup_register
    @return uint32_t
*/
uint32_t JIG_TEST_rtc_get_value_from_backup_register(uint32_t rtc_backup_register_addr);

/** @brief rtc_process
    @return rtc_process
*/
void JIG_TEST_rtc_process(void);
#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_RTC_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

