/** 
  ******************************************************************************
  * @file    JIG_TEST_display_v2.h
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

#ifndef __JIG_TEST_DISPLAY_V2_H
#define __JIG_TEST_DISPLAY_V2_H

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
/** @brief display_v2_configuration
    @return none
*/
void JIG_TEST_display_v2_configuration(void);

/** @brief display_v2_process
    @return none
*/
void JIG_TEST_display_v2_process(void);
#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_DISPLAY_V2_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

