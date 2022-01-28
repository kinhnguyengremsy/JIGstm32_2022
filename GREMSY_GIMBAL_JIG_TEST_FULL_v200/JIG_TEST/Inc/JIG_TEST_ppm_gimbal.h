/** 
  ******************************************************************************
  * @file    JIG_TEST_ppm_gimbal.h
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

#ifndef __JIG_TEST_PPM_GIMBAL_H
#define __JIG_TEST_PPM_GIMBAL_H

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
/** @brief jig_test_ppm_gimbal_config
    @return 
*/
void JIG_TEST_ppm_gimbal_configuration(void);

/** @brief jig_test_ppm_gimbal_process
    @return none
*/
void JIG_TEST_ppm_gimbal_process(void);

/** @brief ppm_gimbal_enable
    @return none
*/
void JIG_TEST_ppm_gimbal_enable(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_PPM_GIMBAL_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

