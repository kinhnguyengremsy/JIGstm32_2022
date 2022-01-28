/** 
  ******************************************************************************
  * @file    .c
  * @author  Gremsy Team
  * @version v2.0.0
  * @date    __DATE__
  * @brief   
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy.  
  * All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "ppm.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define NUMBER_OF_DATA              9
#define PPM_ZERO_VALUE_8_CHANNEL    1507
#define PPM_VALUE_CHANNEL_REST      10400

#define PPM_VALUE_MIN               1000
#define PPM_VALUE_MID               1507
#define PPM_VALUE_MAX               2000
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t aSRC_Buffer[NUMBER_OF_DATA] = {
  PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_ZERO_VALUE_8_CHANNEL
, PPM_VALUE_CHANNEL_REST
};

extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim4_up;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @group PPM_CONFIGURATION
    @{
*/#ifndef PPM_CONFIGURATION
#define PPM_CONFIGURATION

/** @brief ppm configuration
    @return none
*/
void ppm_configuration(void)
{
  /// Start PWM channel 4
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    
  /* TIM1 DMA Update enable */
  TIM4->DIER |= TIM_DMA_UPDATE;

  /* Select the DMA base register and DMA burst length */
  TIM4->DCR = TIM_DMABase_ARR | TIM_DMABurstLength_1Transfer;
  
  /// clear all dma interrupt
    __HAL_DMA_DISABLE_IT(&hdma_tim4_up, DMA_IT_TC | DMA_IT_DME | DMA_IT_FE | DMA_IT_HT | DMA_IT_TE);

  /* Disable DMA1 Channel5 */
    __HAL_DMA_DISABLE(&hdma_tim4_up);
    
  /* Write to DMA1 Channel5 number of data’s register */
    __HAL_DMA_SET_COUNTER(&hdma_tim4_up, NUMBER_OF_DATA);
    
  /* Write to DMA1 Channel5 peripheral address register */
    hdma_tim4_up.Instance->PAR = (uint32_t)&TIM4->DMAR;
    
  /* Write to DMA1 Channel5 Memory address register */
  /* Set the address to the memory buffer “aSRC_Buffer?? */
    hdma_tim4_up.Instance->M0AR = (uint32_t)aSRC_Buffer;
    
  /* Enable DMA1 Channel5 */
//    __HAL_DMA_ENABLE(&hdma_tim4_up);
}
#endif
/**
    @}
*/

/** @group PPM_CHANNEL_COMMUNICATION
    @{
*/#ifndef PPM_CHANNEL_COMMUNICATION
#define PPM_CHANNEL_COMMUNICATION

/** @brief set value channel ppm
    @return none
*/
bool ppm_set_channel(ppm_channel_t channel, uint16_t value)
{
    /// Limit ppm channel value
    if(value < PPM_VALUE_MIN)
    {
        value = PPM_VALUE_MIN;
    }
    else if(value > PPM_VALUE_MAX)
    {
        value = PPM_VALUE_MAX;
    }
    
    /// set ppm channel value
    if(channel == PPM_CHANNEL_1)
    {
        aSRC_Buffer[PPM_CHANNEL_1] = value;
    }
    else if(channel == PPM_CHANNEL_2)
    {
        aSRC_Buffer[PPM_CHANNEL_2] = value;
    }
    else if(channel == PPM_CHANNEL_3)
    {
        aSRC_Buffer[PPM_CHANNEL_3] = value;
    }
    else if(channel == PPM_CHANNEL_4)
    {
        aSRC_Buffer[PPM_CHANNEL_4] = value;
    }
    else if(channel == PPM_CHANNEL_5)
    {
        aSRC_Buffer[PPM_CHANNEL_5] = value;
    }
    else if(channel == PPM_CHANNEL_6)
    {
        aSRC_Buffer[PPM_CHANNEL_6] = value;
    }
    else if(channel == PPM_CHANNEL_7)
    {
        aSRC_Buffer[PPM_CHANNEL_7] = value;
    }
    else if(channel == PPM_CHANNEL_8)
    {
        aSRC_Buffer[PPM_CHANNEL_8] = value;
    }
    
    return true;
}

/** @brief ppm_enable
    @return none
*/
void ppm_enable(void)
{
    __HAL_DMA_ENABLE(&hdma_tim4_up);
}

/** @brief ppm_enable
    @return none
*/
void ppm_disable(void)
{
    __HAL_DMA_DISABLE(&hdma_tim4_up);
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

