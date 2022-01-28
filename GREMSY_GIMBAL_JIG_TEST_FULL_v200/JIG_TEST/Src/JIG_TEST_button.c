/** 
  ******************************************************************************
  * @file    JIG_TEST_button.c
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
#include "JIG_TEST_button.h"
#include "main.h"
#include "timeOut.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_config.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint8_t count;
    bool run_feed_back_imu;
    uint8_t back_to_scan_new_barCode;
    
}JIG_TEST_button_private_t;
/* Private define ------------------------------------------------------------*/

#if (BUTTON_NONE_STLINK == 1)
    #define BUTTON_PIN  GPIO_PIN_14
    #define BUTTON_PORT GPIOA
#endif
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

JIG_TEST_button_private_t button_private;

char *str = "\nBUTTON --->";
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @group JIG_TEST_BUTON_CONFIGURATION
    @{
*/#ifndef JIG_TEST_BUTON_CONFIGURATION
#define JIG_TEST_BUTON_CONFIGURATION

/** @brief button_configuration
    @return none
*/
void JIG_TEST_button_configuration(void)
{

    #if (USE_BUTTON_NONE_STLINK == 1)
        GPIO_InitTypeDef GPIO_InitStruct = {0};
    
        /*Configure GPIO pin : button_Pin */
        GPIO_InitStruct.Pin = BUTTON_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
    #else
    
    #endif
}

#endif
/**
    @}
*/

/** @group JIG_TEST_BUTON_EVENT
    @{
*/#ifndef JIG_TEST_BUTON_EVENT
#define JIG_TEST_BUTON_EVENT

/** @brief button_get_event
    @return true : button press
            false : unppress
*/
static bool JIG_TEST_button_get_event(void)
{
    bool ret = false;
    
    #if (USE_BUTTON_NONE_STLINK == 1)
        if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET)
    #else
        if(HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin) == GPIO_PIN_RESET)
    #endif
    {
        ret = true;
    }
    
    return ret;
}

/** @brief button_press
    @return true : press
            false : unpress
*/
static bool JIG_TEST_button_press(void)
{
    bool ret = false;
    uint32_t i = 0;
    static uint32_t count_console;
    
    if(JIG_TEST_button_get_event() == true)
    { 
        for(i = 0; i < BUTTON_NOISE; i++)
        {
            /// check button event
            if(JIG_TEST_button_get_event() == false)
            {
                return false;
            }
        }
        
        if(i == BUTTON_NOISE)
        {
            /// wait unpress button
            while(JIG_TEST_button_get_event() == true)
            {
                if(HAL_GetTick() - count_console > 1000)
                {
                    count_console = HAL_GetTick();
                    
                    JIG_TEST_console_write(str);
                    HAL_Delay(5);
                    JIG_TEST_console_write("button is pressing \n");
                }
            }
        }
        
        JIG_TEST_console_write(str);
        HAL_Delay(5);
        JIG_TEST_console_write("button is unpress \n");
        
        ret = true;
    }
    
    return ret;
}

/** @brief button_state_feed_back_imu
    @return true : feed back process
            false : no call feed back
*/
bool JIG_TEST_button_state_feed_back_imu(void)
{
    bool ret = false;
    
    if(button_private.run_feed_back_imu == true)
    {
        /// reset flag tun feed back imu
        button_private.run_feed_back_imu = false;
        
        /// reset flag back to scan new barCode
        button_private.back_to_scan_new_barCode = 0;
        
        ret = true;
    }
    
    return ret;
}

/** @brief button_state_back_to_scan_new_barCode
    @return true : back to scan new barCode
*/
bool JIG_TEST_button_state_back_to_scan_new_barCode(void)
{
    bool ret = false;
    
    if(button_private.back_to_scan_new_barCode == 2)
    {
        /// reset flag back to scan new barCode
        button_private.back_to_scan_new_barCode = 0;
        
        /// reset flag tun feed back imu
        button_private.run_feed_back_imu = false;
        
        ret = true;
    }
    
    return ret;
}

/** @brief button_select_2_mode
    @return none
*/
static void JIG_TEST_button_select_2_mode(void)
{
    if(get_timeOut(2500, JIG_TEST_BUTTON_SELECT_2_MODE))
    {
        if(button_private.back_to_scan_new_barCode == 1)
        {
            button_private.back_to_scan_new_barCode = 0;
            
            /// run feed back imu
            button_private.run_feed_back_imu = true;
        }
        else if(button_private.back_to_scan_new_barCode == 2)
        {
            /// not run feed back imu
            button_private.run_feed_back_imu = false;
        }
    }
    else
    {
        if(JIG_TEST_button_press() == true)
        {
            /// back to scan new barCode
            button_private.back_to_scan_new_barCode ++;
            
            
            /// only 1 mode
            if(button_private.back_to_scan_new_barCode > 1)
            {
                button_private.back_to_scan_new_barCode = 0;
            }
        }
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_BUTON_PROCESS
    @{
*/#ifndef JIG_TEST_BUTON_PROCESS
#define JIG_TEST_BUTON_PROCESS

/** @brief button_process
    @return none
*/
void JIG_TEST_button_process(void)
{
    JIG_TEST_button_select_2_mode();
    
//    if(JIG_TEST_button_press() == true)
//    {
//        char buff[100];
//        
//        sprintf(buff, "[BUTTON_TEST] count : %3d", button_private.count ++);
//        JIG_TEST_console_write(buff);
//    }
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

