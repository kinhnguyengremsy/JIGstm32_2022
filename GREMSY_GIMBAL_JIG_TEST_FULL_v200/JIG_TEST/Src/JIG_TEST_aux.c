/**
  ******************************************************************************
  * @file JIG_TEST_aux.c
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
#include "JIG_TEST_aux.h"
#include "JIG_TEST_gimbal_FSTD_v2.h"
#include "JIG_TEST_console.h"
#include "main.h"
#include "timeOut.h"
/* Private typedef------------------------------------------------------------------------------*/

typedef enum
{
    PIN_NONE,
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
    B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15,
    C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15,
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15,
    E0, E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15,
    F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15,
    G0, G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11, G12, G13, G14, G15,
    H0, H1, H2, H3, H4, H5, H6, H7, H8, H9, H10, H11, H12, H13, H14, H15,
    I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15,
    PIN_COUNT,
}pin_t;

/* Private define------------------------------------------------------------------------------*/

#define GPIO_PORT(pin)  (((pin -1) >> 4) & 0x0F)
#define GPIO_PIN(pin)   ((pin - 1) & 0x0F)
#define TIME_PIN_ALL    320
#define TIME_RATIO      2
#define TIME_GAP        (TIME_PIN_ALL*TIME_RATIO)
#define TIME_ALL        (TIME_PIN_ALL*TIME_RATIO + TIME_GAP)

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/

uint16_t HARDWARE_GPIO_PIN[16] = 
{
    GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_3,
    GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
    GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
    GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15,
};

GPIO_TypeDef *HARDWARE_GPIO_PORT[4] = 
{
    GPIOA, GPIOB, GPIOC, GPIOD
};

static pin_t PIN_OUT[] = 
{
    // AUX S9
    B8,
    
    /// AUX S8
    B13,
    
    ///AUX S2 | AUX S7
    B0//B12
    
};

static pin_t PIN_IN[] = 
{
    ///AUC S7 | AUX S6
    B12,//B5,
    
    ///AUX S6 AUX S3
    B5,//B1,
    
    /// AUX S3 | AUX S2
    B1
};

static pin_t pin_aux_ok[] = 
{
    B0, B1, B3, B4, 
    B5, B12, B13, B8
};

extern JIG_TEST_gimbal_FSTD_v2_global_t  gimbal_FSTD_global;

/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
/** @group JIG_TEST_AUX_CONFIGURATION
    @{
*/#ifndef JIG_TEST_AUX_CONFIGURATION
#define JIG_TEST_AUX_CONFIGURATION

/** @brief aux_configuration
    @return none
*/
void JIG_TEST_aux_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, AUX_S8_Pin | AUX_S9_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : AUX_S6_Pin AUX_S7_Pin */
    GPIO_InitStruct.Pin = AUX_S6_Pin | AUX_S7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : AUX_S8_Pin AUX_S9_Pin */
    GPIO_InitStruct.Pin = AUX_S8_Pin | AUX_S9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}



#endif
/**
    @}
*/

/** @group JIG_TEST_AUX_READ_WRITE_GPIO
    @{
*/#ifndef JIG_TEST_AUX_READ_WRITE_GPIO
#define JIG_TEST_AUX_READ_WRITE_GPIO

/** @brief aux_gpio_write
    @return none
*/
static void JIG_TEST_aux_gpio_write(pin_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(HARDWARE_GPIO_PORT[GPIO_PORT(pin)], HARDWARE_GPIO_PIN[GPIO_PIN(pin)], state);
}

/** @brief aux_gpio_toggle
    @return none
*/
static void JIG_TEST_aux_gpio_toggle(pin_t pin)
{
    HAL_GPIO_TogglePin(HARDWARE_GPIO_PORT[GPIO_PORT(pin)], HARDWARE_GPIO_PIN[GPIO_PIN(pin)]);
}

/** @brief aux_gpio_readPin
    @return none
*/
static uint8_t JIG_TEST_aux_gpio_readPin(pin_t pin)
{
    return HAL_GPIO_ReadPin(HARDWARE_GPIO_PORT[GPIO_PORT(pin)], HARDWARE_GPIO_PIN[GPIO_PIN(pin)]);
}

/** @brief aux_gpio_transmit
    @return none
*/
static void JIG_TEST_aux_gpio_transmit(pin_t pin, uint32_t time)
{
    if(time > TIME_GAP)
    {
        uint32_t temp = time - TIME_GAP;
        uint32_t mask = pin * 2;
        
        if(temp >= mask)
        {
            JIG_TEST_aux_gpio_write(pin, GPIO_PIN_RESET);
        }
        else
        {
            JIG_TEST_aux_gpio_write(pin, GPIO_PIN_SET);
        }
    }
    else
    {
        uint32_t time_temp = TIME_GAP - TIME_RATIO;
        
        if(time >= time_temp)
        {
            JIG_TEST_aux_gpio_write(pin, GPIO_PIN_RESET);
        }
        else
        {
            JIG_TEST_aux_gpio_write(pin, GPIO_PIN_SET);
        }
    }
}

/** @brief aux_gpio_run_aux_ok
    @return none
*/
static void JIG_TEST_aux_gpio_result_ok(void)
{
    static int8_t count = 0;
    static bool state = false;
//    char *aux_result_ok = "\nAUX_RESULT_OK --->";
//    char buff[100];
    
    if(get_timeOut(100, JIG_TEST_AUX_GPIO_RESULT_OK))
    {
        if(state == false)
        {
            JIG_TEST_aux_gpio_write(pin_aux_ok[count], GPIO_PIN_SET);
            
            count ++;
            
            if(count > 7)
            {
                state = !state;
            }
        }
        else
        {
            JIG_TEST_aux_gpio_write(pin_aux_ok[count], GPIO_PIN_RESET);
            
            count --;
            
            if(count < 0)
            {
                state = !state;
            }
        }

//        
//        JIG_TEST_console_write(aux_result_ok);
//        sprintf(buff, "count :%d | state : %d", count, state);
//        JIG_TEST_console_write(buff);
        
    }
}

#endif
/**
    @}
*/

/** @group JIG_TEST_AUX_PROCESS
    @{
*/#ifndef JIG_TEST_AUX_PROCESS
#define JIG_TEST_AUX_PROCESS

/** @brief aux_process
    @return none
*/
static void JIG_TEST_aux_test_process(bool enable)
{

}

/** @brief aux_process
    @return none
*/
void JIG_TEST_aux_process(void)
{
    if(gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_AUX] == true)
    {
        JIG_TEST_aux_gpio_result_ok();
    }
}


#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


