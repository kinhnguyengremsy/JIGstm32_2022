/** 
  ******************************************************************************
  * @file    JIG_TEST_button.h
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

#ifndef __JIG_TEST_BUTTON_H
#define __JIG_TEST_BUTTON_H

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

/** @brief button_configuration
    @return none
*/
void JIG_TEST_button_configuration(void);

/** @brief button_process
    @return none
*/
void JIG_TEST_button_process(void);

/** @brief button_state_feed_back_imu
    @return true : feed back process
            false : no call feed back
*/
bool JIG_TEST_button_state_feed_back_imu(void);

/** @brief button_state_back_to_scan_new_barCode
    @return true : back to scan new barCode
*/
bool JIG_TEST_button_state_back_to_scan_new_barCode(void);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_BUTTON_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

