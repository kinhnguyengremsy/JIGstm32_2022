/** 
  ******************************************************************************
  * @file    ppm.h
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

#ifndef __PPM_H
#define __PPM_H

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
typedef enum
{
    PPM_CHANNEL_1,
    PPM_CHANNEL_2,
    PPM_CHANNEL_3,
    PPM_CHANNEL_4,
    PPM_CHANNEL_5,
    PPM_CHANNEL_6,
    PPM_CHANNEL_7,
    PPM_CHANNEL_8,
    
}ppm_channel_t;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief ppm configuration
    @return none
*/
void ppm_configuration(void);

/** @brief set value channel ppm
    @return none
*/
bool ppm_set_channel(ppm_channel_t channel, uint16_t value);

/** @brief ppm_enable
    @return none
*/
void ppm_enable(void);

/** @brief ppm_enable
    @return none
*/
void ppm_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* __PPM_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

