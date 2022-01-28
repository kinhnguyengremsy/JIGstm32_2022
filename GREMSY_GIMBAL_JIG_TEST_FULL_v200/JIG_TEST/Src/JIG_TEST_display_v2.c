/**
  ******************************************************************************
  * @file JIG_TEST_display_v2.c
  * @author  Gremsy Team
  * @version v2.0.0
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
#include "JIG_TEST_display_v2.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_gimbal_FSTD_v2.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_comm_raspberry_v2.h"
#include "main.h"
#include "timeOut.h"
#include "ssd1306.h"
#include "string.h"
/* Private typedef------------------------------------------------------------------------------*/
typedef struct
{
    bool is_Ready;
    
}JIG_TEST_displayError_t;

typedef struct
{
    bool is_deviceReady;
    bool is_clearScreen;
    
    char *gimbalDevice;
    
    bool total_result;
    
    JIG_TEST_displayError_t error;
}JIG_TEST_display_v2_private_t;
/* Private define------------------------------------------------------------------------------*/

#define FONT_16x26   16
#define FONT_11x18   11
#define FONT_6x8     6
#define FONT_7x10    7

#define GREMSY_DEVICE_NUMBER 13

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
JIG_TEST_display_v2_private_t               display_private;
uint8_t JGI_TEST_DISPLAY_LINE_ONE_Y     = 0;
uint8_t JGI_TEST_DISPLAY_LINE_TWO_Y     = 25;
uint8_t JGI_TEST_DISPLAY_LINE_THREE_Y   = 35;
uint8_t JGI_TEST_DISPLAY_LINE_FOUR_Y    = 45;
uint8_t JGI_TEST_DISPLAY_LINE_FIVE_Y    = 55;
extern JIG_TEST_gimbal_FSTD_v2_global_t     gimbal_FSTD_global;
extern JIG_TEST_mavlink_gimbal_t            mavlink_gimbal_COM2;

struct _gremsy_device
{
    uint8_t id;
    char *name;
}gremsy_device[GREMSY_DEVICE_NUMBER] = 
{
    {.id = 0x3A, .name = "PixyU"},
    {.id = 0x0A, .name = "PixyF"},
    {.id = 0x1A, .name = "PixyFI"},
    {.id = 0x4A, .name = "PixyW"},
    {.id = 0x6A, .name = "PixyD1v2"},
    {.id = 0x7A, .name = "ACSL"},
    {.id = 0x44, .name = "T3v3"},
    {.id = 0x14, .name = "T3v2"},
    {.id = 0x54, .name = "skyFish"},
    {.id = 0x12, .name = "S1v2"},
    {.id = 0x22, .name = "S1v3"},
    {.id = 0x08, .name = "T7"},
    {.id = 0x0B, .name = "Mio"},
    
};
/* Private function prototypes------------------------------------------------------------------------------*/
/** @brief display_v2_after_reset
    @return none
*/
static void JIG_TEST_display_v2_after_reset(void);
/* Private functions------------------------------------------------------------------------------*/

/** @group JIG_TEST_DISPLAY_V2_CONFIGURATION
    @{
*/#ifndef JIG_TEST_DISPLAY_V2_CONFIGURATION
#define JIG_TEST_DISPLAY_V2_CONFIGURATION

/** @brief display_v2_configuration
    @return none
*/
void JIG_TEST_display_v2_configuration(void)
{
    
    #if (GIMBAL_FSTD_JIG_TEST == 1)
        /// set default gimbal Device
        display_private.gimbalDevice = "GREMSY";
    #else
        /// set default gimbal Device
        display_private.gimbalDevice = "AC30000";
    #endif
    
    if(ssd1306_Init() == 1)
    {
        display_private.is_deviceReady = true;
        
        JIG_TEST_display_v2_after_reset();
        
        /// write to console i2c device status
        JIG_TEST_console_write("------------- DISPLAY INIT DONE -------------------\n");
    }
    else
    {
        /// set flag displat error device ready
        display_private.error.is_Ready = true;
        
        /// write to console i2c device status
        JIG_TEST_console_write("------------- DISPLAY INIT ERROR -------------------\n");
    }
    
}


#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_V2_WRITE_DATA
    @{
*/#ifndef JIG_TEST_DISPLAY_V2_WRITE_DATA
#define JIG_TEST_DISPLAY_V2_WRITE_DATA

/** @brief 
    @return 
*/
/** @brief display_ErrorHandle
    @return none
*/
static void JIG_TEST_display_ErrorHandle(void)
{
    static uint8_t error;
    
    /// error device ready
    if(display_private.error.is_Ready == true)
    {
        if(get_timeOut(1000, DISPLAY_ERROR_DEVICE_READY))
        {
            /// write error Console
            error ++;
        }
    }
}

/** @brief display_get_center_screen
    @return none
*/
static uint16_t JIG_TEST_display_get_center_screen(char *str, uint8_t font_x)
{
    uint16_t pos_start = 0;
    uint16_t len = strlen(str);

    /// get number of str
    uint16_t temp = len * font_x;
    
    /// get pos start display
    pos_start = (SSD1306_WIDTH - temp ) / 2;
    
    return pos_start;
}

/** @brief display_writeData
    @return none
*/
static void JIG_TEST_display_writeData(uint16_t font_x, uint8_t y, char *str, SSD1306_COLOR color, FontDef font)
{
    uint16_t posx_start = 0;
    
    posx_start = JIG_TEST_display_get_center_screen(str, font_x);
    ssd1306_SetCursor(posx_start, y);
    ssd1306_WriteString(str, font, color);
}

/** @brief display_UpdateScreen
    @return none
*/
static void JGI_TEST_display_UpdateScreen(void)
{
    ssd1306_UpdateScreen();
}

/** @brief display_UpdateScreen
    @return none
*/
static void JGI_TEST_display_v2_clear(uint8_t y, FontDef font)
{
    ssd1306_SetCursor(0, 18);
    ssd1306_WriteString("              ", Font_11x18, White);
    
    ssd1306_SetCursor(0, 36);
    ssd1306_WriteString("              ", Font_11x18, White);
    
    ssd1306_SetCursor(2, 54);
    ssd1306_WriteString("                   ", Font_7x10, White);

}

/** @brief display_get_device_name
    @return none
*/
static char *JIG_TEST_display_get_device_name(uint8_t id)
{

//    char *gremsy = "GREMSY";
    
    #if (JIG_TEST_ID == 0x20 || JIG_TEST_ID == 0x21)
    
        char *gremsy = "AC30000";
        return gremsy;
    #endif
    
    #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x34)

        for(uint8_t i = 0; i < GREMSY_DEVICE_NUMBER ; i++)
        {
            if(id == gremsy_device[i].id)
            {
                return gremsy_device[i].name;
            }
        }
    
    #endif

//    return gremsy;
    
}
#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_V2_GET_STATE
    @{
*/#ifndef JIG_TEST_DISPLAY_V2_GET_STATE
#define JIG_TEST_DISPLAY_V2_GET_STATE

/** @brief display_v2_get_scanBarcode_done
    @return none
*/
static bool JIG_TEST_display_v2_get_scanBarcode_done(void)
{
    return gimbal_FSTD_global.get_reset_system;
}

/** @brief display_v2_get_gimbal_startup_calib_motor
    @return none
*/
static void JIG_TEST_display_v2_gimbal_startup_calib_motor(void)
{
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);

    if(gimbal_FSTD_global.gimbal_startup_calib == 1)
    {
        JIG_TEST_display_writeData(   FONT_6x8
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , "GIMBAL STATE CALIB ", White, Font_6x8);

        JIG_TEST_display_writeData(   FONT_7x10
                                    , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                    , "MOTOR", Black, Font_7x10);
    }
    else
    {
        JIG_TEST_display_writeData(   FONT_6x8
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , "GIMBAL STARTUP CALIB ", White, Font_6x8);
    }
}

/** @brief display_v2_get_gimbal_startup_calib_imu
    @return none
*/
static void JIG_TEST_display_v2_gimbal_startup_calib_imu(void)
{
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);

    if(gimbal_FSTD_global.gimbal_startup_calib == 2)
    {
        JIG_TEST_display_writeData(   FONT_6x8
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , "GIMBAL STATE CALIB ", White, Font_6x8);

        JIG_TEST_display_writeData(   FONT_7x10
                                    , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                    , "IMU", Black, Font_7x10);
    }
}

/** @brief display_v2_setting_param_gimbal
    @return none
*/
static void JIG_TEST_display_v2_setting_param_gimbal(void)
{
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);

    JIG_TEST_display_writeData(   FONT_6x8
                                , JGI_TEST_DISPLAY_LINE_THREE_Y
                                , "SETTING PARAM GIMBAL", White, Font_6x8);
}

/** @brief display_v2_waitting_scan_barCode
    @return none
*/
static void JIG_TEST_display_v2_waitting_scan_barCode(void)
{
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);

    JIG_TEST_display_writeData(   FONT_6x8
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , "WAITTING FOR SCAN", White, Font_6x8);
    JIG_TEST_display_writeData(   FONT_7x10
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , "BARCODE", Black, Font_7x10);
}

/** @brief display_v2_waitting_for_login
    @return none
*/
static void JIG_TEST_display_v2_waitting_for_login(void)
{
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);

    JIG_TEST_display_writeData(   FONT_6x8
                                , JGI_TEST_DISPLAY_LINE_THREE_Y + 2
                                , "WAITTING FOR LOGIN", White, Font_6x8);
}

/** @brief display_v2_time_test
    @return none
*/
static void JIG_TEST_display_v2_time_test(void)
{
    char buff[100];

    sprintf(buff, "Time test : %.3d", gimbal_FSTD_global.time_test_total);
    JIG_TEST_display_writeData(   FONT_6x8
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , buff, White, Font_6x8);
    
}

/** @brief display_v2_mode_test
    @return none
*/
static void JIG_TEST_display_v2_mode_test(char *mode_test)
{
    char buff[100];
    
    JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
    
    sprintf(buff, "%s", mode_test);
    JIG_TEST_display_writeData(   FONT_7x10
                                , JGI_TEST_DISPLAY_LINE_TWO_Y + 2
                                , buff, White, Font_7x10);
    
}

/** @brief display_get_string_result
    @return none
*/
static void JIG_TEST_display_v2_get_string_result(uint8_t mode_test, char *str1, char *str2, char *str3)
{
    if(mode_test == true)
    {
        strcpy(str1, str2);
    }
    else
    {
        strcpy(str1, str3);
    }
}

#if (JIG_TEST_ID == 0x10)
/** @brief display_v2_mode_done
    @return none
*/
static void JIG_TEST_display_v2_state_done(void)
{
    static bool copy_data = false;
    static uint8_t total_result_count = 0;
    static uint8_t count;
    static uint8_t count_clear;
    static uint8_t count_display;

    const uint8_t number_of_result = 11;
    
    struct _jig_test_result
    {
        char result_ok[15];
        char result_error[15];
        
    }jig_test_result[number_of_result] =
    {
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
        {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
        {.result_ok = "CAN:OK",      .result_error = "CAN:ER"},
        {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
        {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
        {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
        {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
    };
        
        static char str_result[number_of_result][20];
        
        if(copy_data == false)
        {
            copy_data = true;

            for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_TOTAL_MODE - 2; i ++)
            {
                if(gimbal_FSTD_global.result_mode_test[i] == true)
                {
                    total_result_count ++;
                }
            }

            /// kiem tra tong cac ket qua test
            if(total_result_count == 7)
            {
                display_private.total_result = true;
            }
            
            /// copy data result jig test full gimbal signal
            for(uint8_t i = 0; i <= number_of_result; i++)
            {
                if(i <= 3)
                {
                    JIG_TEST_display_v2_get_string_result(   display_private.total_result
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
                else if(i > 3)
                {
                    JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 3]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
            }
        }
            
        if(++count_display >= 2)
        {
            count_display = 0;
            
            if(count_clear % 2 == 0)
            {

                if(count > 3)
                JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
                
                /// clear screen lan dau
                if(count == 0)
                {
                   JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
                }
            }
            else
            {
                if(count < 4)
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_16x26
                                                , 25
                                                , str_result[count], Black, Font_16x26);
                }
                else if(count == 10)
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_7x10
                                                , 35
                                                , str_result[count], White, Font_7x10);
                    
                  #if (S1v3_VIRATE_TEST == 1)
                    if(gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_VIBRATE] == false)
                    {
                        if(gimbal_FSTD_global.vibrate_state_s1v3 == 0)
                        {
                            JIG_TEST_display_writeData(   FONT_7x10
                                                        , 45
                                                        , "TILT", White, Font_7x10);
                        }
                        else if(gimbal_FSTD_global.vibrate_state_s1v3 == 1)
                        {
                            JIG_TEST_display_writeData(   FONT_7x10
                                                        , 45
                                                        , "ROLL", White, Font_7x10);
                        }
                        else if(gimbal_FSTD_global.vibrate_state_s1v3 == 2)
                        {
                            JIG_TEST_display_writeData(   FONT_7x10
                                                        , 45
                                                        , "PAN", White, Font_7x10);
                        }
                    }
                  #endif
                }
                else
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_7x10
                                                , 35
                                                , str_result[count], White, Font_7x10);
                }
                
                /// dem so result in ra man hinh
                if(++count > (number_of_result - 1 ))
                {
                    count = 0;
                }
            }
            
            if(++count_clear > (number_of_result * 2))
            {
                count_clear = 0;
            }
        }
}
#endif

#if (JIG_TEST_ID == 0x11)
/** @brief display_v2_mode_done
    @return none
*/
static void JIG_TEST_display_v2_state_done(void)
{
    static bool copy_data = false;
    static uint8_t total_result_count = 0;
    static uint8_t count;
    static uint8_t count_clear;
    
    #if (GIMBAL_FSTD_JIG_TEST == 0)
        const uint8_t number_of_result = 15;
        
        struct _jig_test_result
        {
            char result_ok[15];
            char result_error[15];
            
        }jig_test_result[number_of_result - 3] =
        {
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
            {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
            {.result_ok = "CAN:OK",      .result_error = "CAN:ER"},
            {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
            {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
            {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
            {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
            {.result_ok = "CLOUD:OK",    .result_error = "CLOUD:ER"},
        };
        

    
        char usb_speed_result_no_usb[10]    = "USB:ER#01";
        char usb_speed_result_low_speed[10] = "USB:ER#02";
        char usb_speed_result_none_test[10] = "USB:ER#03";
        char usb_speed_result_passed[10]    = "USB:OK";
    #else
        const uint8_t number_of_result = 12;
        
        struct _jig_test_result
        {
            char result_ok[15];
            char result_error[15];
            
        }jig_test_result[number_of_result] =
        {
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "PASS",        .result_error = "FAIL"},
            {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
            {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
            {.result_ok = "CAN:OK",      .result_error = "CAN:ER"},
            {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
            {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
            {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
            {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
            {.result_ok = "CLOUD:OK",    .result_error = "CLOUD:ER"},
        };
    #endif
        
        static char str_result[number_of_result][20];
        
        if(copy_data == false)
        {
            copy_data = true;

            for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_TOTAL_MODE - 2; i ++)
            {
                if(gimbal_FSTD_global.result_mode_test[i] == true)
                {
                    total_result_count ++;
                }
            }
            
            #if (GIMBAL_FSTD_JIG_TEST == 1)
                /// kiem tra tong cac ket qua test
                if(total_result_count == 8)
                {
                    display_private.total_result = true;
                }
            #endif
                
            #if (GIMBAL_FSTD_JIG_TEST == 1)
                /// copy data result jig test full gimbal signal
                for(uint8_t i = 0; i <= number_of_result; i++)
                {
                    if(i <= 3)
                    {
                        JIG_TEST_display_v2_get_string_result(   display_private.total_result
                                                                , str_result[i]
                                                                , jig_test_result[i].result_ok
                                                                , jig_test_result[i].result_error);
                    }
                    else if(i > 3)
                    {
                        JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 3]
                                                                , str_result[i]
                                                                , jig_test_result[i].result_ok
                                                                , jig_test_result[i].result_error);
                    }
                }
            #endif
        }
            
        if(count_clear % 2 == 0)
        {
            #if (GIMBAL_FSTD_JIG_TEST == 1)
                if(count > 3)
                JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
            #else
                if(count > 3 && count <= 11)
                JIG_TEST_display_clear_three_line();
            #endif
            
            /// clear screen lan dau
            if(count == 0)
            {
               JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
            }
        }
        else
        {
            if(count < 4)
            {
                /// write data to screen
                JIG_TEST_display_writeData(   FONT_16x26
                                            , 25
                                            , str_result[count], Black, Font_16x26);
            }
            else if(count == 10)
            {
                /// write data to screen
                JIG_TEST_display_writeData(   FONT_7x10
                                            , 35
                                            , str_result[count], White, Font_7x10);
            }
            else
            {
                /// write data to screen
                JIG_TEST_display_writeData(   FONT_7x10
                                            , 35
                                            , str_result[count], White, Font_7x10);
            }
            
            /// dem so result in ra man hinh
            if(++count > (number_of_result - 1 ))
            {
                count = 0;
            }
        }
        
        if(++count_clear > (number_of_result * 2))
        {
            count_clear = 0;
        }
}

#endif

#if (JIG_TEST_ID == 0x20)
/** @brief display_v2_mode_done
    @return none
*/
static void JIG_TEST_display_v2_state_done(void)
{
    static bool copy_data = false;
    static uint8_t total_result_count = 0;
    static uint8_t count;
    static uint8_t count_clear;
    static bool usb_speed_data = 0;
    const uint8_t number_of_result = 13;
    
    struct _jig_test_result
    {
        char result_ok[15];
        char result_error[15];
        
    }jig_test_result[number_of_result - 3] =
    {
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
        {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
        {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
        {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
        {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
        {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
    };
    


    char usb_speed_result_no_usb[10]    = "USB:ER#01";
    char usb_speed_result_low_speed[10] = "USB:ER#02";
    char usb_speed_result_none_test[10] = "USB:ER#03";
    char usb_speed_result_passed[10]    = "USB:OK";
    
    static char str_result[number_of_result][20];
    
    if(copy_data == false)
    {
        copy_data = true;

        for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_TOTAL_MODE - 2; i ++)
        {
            if(gimbal_FSTD_global.result_mode_test[i] == true)
            {
                /// remove mode CAN
                if(i != 3)
                total_result_count ++;
            }
        }
        
        /// kiem tra ket qua usb speed test
        if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_PASS)
        {
            total_result_count ++;
        }
        
        #if (GIMBAL_FSTD_JIG_TEST == 1)
            /// kiem tra tong cac ket qua test
            if(total_result_count == 7)
            {
                display_private.total_result = true;
            }
        #endif
            
        #if (GIMBAL_FSTD_JIG_TEST == 1)
            /// copy data result jig test full gimbal signal
            for(uint8_t i = 0; i <= (number_of_result - 4); i++)
            {
                if(i <= 3)
                {
                    JIG_TEST_display_v2_get_string_result(   display_private.total_result
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
                else if(i >= 4 && i <= 5)
                {
                    JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 3]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
                else if(i >= 6)
                {
                    /// remove mode CAN
                    JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 2]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
                
            }
                
            /// copy data usb speed result jig test full gimbal signal
            for(uint8_t i = (number_of_result - 3); i <= number_of_result; i ++)
            {
                if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_NONE)
                {
                    strcpy(str_result[i], usb_speed_result_none_test);
                }
                else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_NO_USB)
                {
                    strcpy(str_result[i], usb_speed_result_no_usb);
                }
                else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_LOW_SPEED)
                {
                    strcpy(str_result[i], usb_speed_result_low_speed);
                }
                else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_PASS)
                {
                    strcpy(str_result[i], usb_speed_result_passed);
                }
            }
        #endif
    }
        
    if(count_clear % 2 == 0)
    {
        if(count > 3 && count <= 10)
        JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
        
        /// clear screen lan dau
        if(count == 0)
        {
           JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
        }
    }
    else
    {
        if(count < 4)
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_16x26
                                        , 25
                                        , str_result[count], Black, Font_16x26);
        }
        else if(count == 9)
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 35
                                        , str_result[count], White, Font_7x10);
        }
        else if(count >= (number_of_result - 3) && count <= number_of_result)
        {
            /// write data to screen USB result
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 25
                                        , str_result[count], White, Font_7x10);
            
            static char str_read_speed[100];
            char str_read_speed_temp[50];
            char str_ref_read_speed[50];
            
            static char str_write_speed[100];
            char str_write_speed_temp[50];
            char str_ref_write_speed[50];
            
            /// copy data usb speed result
            if(usb_speed_data == false)
            {
                usb_speed_data = true;

                sprintf(str_read_speed_temp, "USB_r:%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]);
                sprintf(str_ref_read_speed, "/%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]);
                
                sprintf(str_write_speed_temp, "USB_w:%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]);
                sprintf(str_ref_write_speed, "/%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]);
                
                strcpy(str_read_speed, str_read_speed_temp);
                strcat(str_read_speed, str_ref_read_speed);
                
                strcpy(str_write_speed, str_write_speed_temp);
                strcat(str_write_speed, str_ref_write_speed);
                
            }
            
            /// write data to screen
            JIG_TEST_display_writeData(FONT_6x8, 35, str_read_speed, White, Font_6x8);
            JIG_TEST_display_writeData(FONT_6x8, 45, str_write_speed, White, Font_6x8);
        }
        else
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 35
                                        , str_result[count], White, Font_7x10);
        }
        /// dem so result in ra man hinh
        if(++count > (number_of_result - 1 ))
        {
            count = 0;
        }
    }
    
    if(++count_clear > (number_of_result * 2))
    {
        count_clear = 0;
    }
}

#endif
#if (JIG_TEST_ID == 0x21)
/** @brief display_v2_mode_done
    @return none
*/
static void JIG_TEST_display_v2_state_done(void)
{
    static bool copy_data = false;
    static uint8_t count;
    static uint8_t count_clear;
    static bool usb_speed_data = 0;
    const uint8_t number_of_result = 14;
    
    struct _jig_test_result
    {
        char result_ok[15];
        char result_error[15];
        
    }jig_test_result[number_of_result - 3] =
    {
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
        {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
        {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
        {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
        {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
        {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
        {.result_ok = "CLOUD:OK",    .result_error = "CLOUD:ER"},
    };
    
    char usb_speed_result_no_usb[10]    = "USB:ER#01";
    char usb_speed_result_low_speed[10] = "USB:ER#02";
    char usb_speed_result_none_test[10] = "USB:ER#03";
    char usb_speed_result_passed[10]    = "USB:OK";
    
    static char str_result[number_of_result][20];
    
    if(copy_data == false)
    {
        copy_data = true;

        if(gimbal_FSTD_global.mode_test == JIG_TEST_GIMBAL_V2_MODE_DONE 
            && gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_PASS
        )
        {
            display_private.total_result = true;
        }

        /// copy data result jig test full gimbal signal
        for(uint8_t i = 0; i <= (number_of_result - 4); i++)
        {
            if(i <= 3)
            {
                JIG_TEST_display_v2_get_string_result(   display_private.total_result
                                                        , str_result[i]
                                                        , jig_test_result[i].result_ok
                                                        , jig_test_result[i].result_error);
            }
            else if(i >= 4 && i <= 5)
            {
                JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 3]
                                                        , str_result[i]
                                                        , jig_test_result[i].result_ok
                                                        , jig_test_result[i].result_error);
            }
            else if(i >= 6)
            {
                /// remove mode CAN
                JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 2]
                                                        , str_result[i]
                                                        , jig_test_result[i].result_ok
                                                        , jig_test_result[i].result_error);
            }
        }
            
        /// copy data usb speed result jig test full gimbal signal
        for(uint8_t i = (number_of_result - 3); i <= number_of_result; i ++)
        {
            if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_NONE)
            {
                strcpy(str_result[i], usb_speed_result_none_test);
            }
            else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_NO_USB)
            {
                strcpy(str_result[i], usb_speed_result_no_usb);
            }
            else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_LOW_SPEED)
            {
                strcpy(str_result[i], usb_speed_result_low_speed);
            }
            else if(gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_RESULT] == JIG_TEST_USB_SPEED_RESULT_PASS)
            {
                strcpy(str_result[i], usb_speed_result_passed);
            }
        }
        
    }
        
    if(count_clear % 2 == 0)
    {
        if(count > 3 && count <= 10)
        JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
        
        /// clear screen lan dau
        if(count == 0)
        {
           JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
        }
    }
    else
    {
        if(count < 4)
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_16x26
                                        , 25
                                        , str_result[count], Black, Font_16x26);
        }
        else if(count == 9)
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 35
                                        , str_result[count], White, Font_7x10);
        }
        else if(count >= (number_of_result - 3) && count <= number_of_result)
        {
            /// write data to screen USB result
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 25
                                        , str_result[count], White, Font_7x10);
            
            static char str_read_speed[100];
            char str_read_speed_temp[50];
            char str_ref_read_speed[50];
            
            static char str_write_speed[100];
            char str_write_speed_temp[50];
            char str_ref_write_speed[50];
            
            /// copy data usb speed result
            if(usb_speed_data == false)
            {
                usb_speed_data = true;

                sprintf(str_read_speed_temp, "USB_r:%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_READ_SPEED]);
                sprintf(str_ref_read_speed, "/%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_READ_SPEED]);
                
                sprintf(str_write_speed_temp, "USB_w:%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_WRITE_SPEED]);
                sprintf(str_ref_write_speed, "/%.2f", gimbal_FSTD_global.usb_speed.value_speed_test[JIG_TEST_USB_SPEED_READ_STATE_REF_WRITE_SPEED]);
                
                strcpy(str_read_speed, str_read_speed_temp);
                strcat(str_read_speed, str_ref_read_speed);
                
                strcpy(str_write_speed, str_write_speed_temp);
                strcat(str_write_speed, str_ref_write_speed);
                
            }
            
            /// write data to screen
            JIG_TEST_display_writeData(FONT_6x8, 35, str_read_speed, White, Font_6x8);
            JIG_TEST_display_writeData(FONT_6x8, 45, str_write_speed, White, Font_6x8);
        }
        else
        {
            /// write data to screen
            JIG_TEST_display_writeData(   FONT_7x10
                                        , 35
                                        , str_result[count], White, Font_7x10);
        }
        /// dem so result in ra man hinh
        if(++count > (number_of_result - 1 ))
        {
            count = 0;
        }
    }
    
    if(++count_clear > (number_of_result * 2))
    {
        count_clear = 0;
    }
}

#endif
#if (JIG_TEST_ID == 0x34)
/** @brief display_v2_mode_done
    @return none
*/
static void JIG_TEST_display_v2_state_done(void)
{
    static bool copy_data = false;
    static uint8_t total_result_count = 0;
    static uint8_t count;
    static uint8_t count_clear;
    static uint8_t speedDisplay = 0;

    const uint8_t number_of_result = 11;
    
    struct _jig_test_result
    {
        char result_ok[15];
        char result_error[15];
        
    }jig_test_result[number_of_result] =
    {
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "PASS",        .result_error = "FAIL"},
        {.result_ok = "SBUS:OK",     .result_error = "SBUS:ER"},
        {.result_ok = "PPM:OK",      .result_error = "PPM:ER"},
        {.result_ok = "CAN:OK",      .result_error = "CAN:ER"},
        {.result_ok = "COM2:OK",     .result_error = "COM2:ER"},
        {.result_ok = "COM4:OK",     .result_error = "COM4:ER"},
        {.result_ok = "AUX:OK",      .result_error = "AUX:ER"},
        {.result_ok = "VIBRATE:OK",  .result_error = "VIBRATE:ER"},
    };
        
        static char str_result[number_of_result][20];
        
        if(copy_data == false)
        {
            copy_data = true;

            for(uint8_t i = 1; i < JIG_TEST_GIMBAL_V2_TOTAL_MODE - 2; i ++)
            {
                if(gimbal_FSTD_global.result_mode_test[i] == true)
                {
                    total_result_count ++;
                }
            }

            /// kiem tra tong cac ket qua test
            if(total_result_count == 7)
            {
                display_private.total_result = true;
            }
            
            /// copy data result jig test full gimbal signal
            for(uint8_t i = 0; i <= number_of_result; i++)
            {
                if(i <= 3)
                {
                    JIG_TEST_display_v2_get_string_result(   display_private.total_result
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
                else if(i > 3)
                {
                    JIG_TEST_display_v2_get_string_result(   gimbal_FSTD_global.result_mode_test[i - 3]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                }
            }
        }
            
        if(++speedDisplay > 1)
        {
            speedDisplay = 0;
            
            if(count_clear % 2 == 0)
            {

                if(count > 3)
                JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
                
                /// clear screen lan dau
                if(count == 0)
                {
                   JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
                }
            }
            else
            {
                if(count < 4)
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_16x26
                                                , 25
                                                , str_result[count], Black, Font_16x26);
                }
                else if(count == 10)
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_7x10
                                                , 35
                                                , str_result[count], White, Font_7x10);
                }
                else
                {
                    /// write data to screen
                    JIG_TEST_display_writeData(   FONT_7x10
                                                , 35
                                                , str_result[count], White, Font_7x10);
                }
                
                /// dem so result in ra man hinh
                if(++count > (number_of_result - 1 ))
                {
                    count = 0;
                }
            }
            
            if(++count_clear > (number_of_result * 2))
            {
                count_clear = 0;
            }
        }
}
#endif

/** @brief display_v2_gimbal_name
    @return none
*/
static void JIG_TEST_display_v2_gimbal_name(char *name)
{
    char buff[100];
    
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("              ", Font_11x18, White);
    
    sprintf(buff, "%s", name);
    JIG_TEST_display_writeData(   FONT_11x18
                                , JGI_TEST_DISPLAY_LINE_ONE_Y
                                , buff, Black, Font_11x18);
    
}
#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_V2_MAIN_PROCESS
    @{
*/#ifndef JIG_TEST_DISPLAY_V2_MAIN_PROCESS
#define JIG_TEST_DISPLAY_V2_MAIN_PROCESS

/** @brief display_v2_main_control_process
    @return none
*/
static void JIG_TEST_display_v2_main_control_process(void)
{
    static uint32_t count_error = 0;
    
    /// kiem tra gimbal device
    if(mavlink_gimbal_COM2.vehicle_system_id != 0)
    {
        display_private.gimbalDevice = JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id);
        JIG_TEST_display_v2_gimbal_name(display_private.gimbalDevice);
    }
    
    if(display_private.is_deviceReady == true)
    {
        #if (JIG_TEST_ID == 0x11 || JIG_TEST_ID == 0x21)
            if(gimbal_FSTD_global.cloudData_command.get_heartbeatReady == true)
        #else
            if(true)
        #endif
            {
                switch((uint8_t)gimbal_FSTD_global.state_test)
                {
                    case JIG_TEST_GIMBAL_V2_STATE_WAIT_LOGIN:
                    {
                        JIG_TEST_display_v2_waitting_for_login();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_WAIT_BARCODE:
                    {
                        JIG_TEST_display_v2_waitting_scan_barCode();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_CHECK_RESET_SYSTEM:
                    {
                        JIG_TEST_display_v2_mode_test("SysRST");
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_WAIT_START:
                    {
                        JIG_TEST_display_v2_mode_test("START");
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_CHECK_COM2:
                    {
                        JIG_TEST_display_v2_mode_test("CHECK GIMBAL");
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_MOTOR:
                    {
                        JIG_TEST_display_v2_gimbal_startup_calib_motor();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_WAIT_GIMBAL_STARTUP_CALIB_IMU:
                    {
                        JIG_TEST_display_v2_gimbal_startup_calib_imu();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_CHECK_IMU_TYPE:
                    {
                        JIG_TEST_display_v2_mode_test("CHECK IMU TYPE");
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_SETTING_PARAM:
                    {
                        JIG_TEST_display_v2_setting_param_gimbal();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_CONTROL_JIG_TEST:
                    {
                        switch((uint8_t)gimbal_FSTD_global.mode_test)
                        {
                            case JIG_TEST_GIMBAL_V2_MODE_IDLE:
                            {
                                
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_SBUS:
                            {
                                JIG_TEST_display_v2_mode_test("SBUS");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_PPM:
                            {
                                JIG_TEST_display_v2_mode_test("PPM");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_CAN:
                            {
                                JIG_TEST_display_v2_mode_test("CAN");;
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_COM:
                            {
                                JIG_TEST_display_v2_mode_test("COM2");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_COM4:
                            {
                                JIG_TEST_display_v2_mode_test("COM4");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_AUX:
                            {
                                JIG_TEST_display_v2_mode_test("AUX");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_VIBRATE:
                            {
                                JIG_TEST_display_v2_mode_test("VIBRATE");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD:
                            {
                                JIG_TEST_display_v2_mode_test("LOAD Profile");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_USB_SPEED:
                            {
                                JIG_TEST_display_v2_mode_test("USB_SPEED");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_DONE:
                            {
                                JIG_TEST_display_v2_mode_test("PUSH CLOUD");
                            }break;
                            case JIG_TEST_GIMBAL_V2_MODE_ERROR:
                            {
                                JIG_TEST_display_v2_mode_test("push cloud");
                            }break;
                        }
                        
                        /// DISPLAY time test
                        JIG_TEST_display_v2_time_test();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_DONE:
                    {
                        JIG_TEST_display_v2_state_done();
                    }break;
                    case JIG_TEST_GIMBAL_V2_STATE_ERROR:
                    {
                        if(JIG_TEST_gimbal_FSTD_v2_get_error_heartbeat_com2() == true)
                        JIG_TEST_display_v2_mode_test("HB Gimbal Error");
                        
                        if(gimbal_FSTD_global.mode_test == JIG_TEST_GIMBAL_V2_MODE_DONE)
                        {
                            if(gimbal_FSTD_global.result_mode_test[JIG_TEST_GIMBAL_V2_MODE_PUSH_CLOUD] == false)
                            JIG_TEST_display_v2_mode_test("Push Data Error");
                        }
                        
                        
                        if(gimbal_FSTD_global.profile_ship_error == true)
                        {
                            JIG_TEST_display_v2_mode_test("profileShip Error");
                        }
                        
                        if(gimbal_FSTD_global.cloudData_command.jig_timeOut == true)
                        {
                            JIG_TEST_display_v2_mode_test("jig timeOut");
                        }
                        
                        if(gimbal_FSTD_global.cloudData_command.cant_read_uuid == true)
                        {
                            JIG_TEST_display_v2_mode_test("Can't Read UUID");
                        }
                    }break;
                    default:
                    {

                    }break;
                }
                
                /// reset count error
                count_error = 0;
            }
            else
            {
                if(JIG_TEST_gimbal_FSTD_v2_get_first_run() == true)
                {
                    if(++ count_error > 10)
                    {
                        count_error = 0;
                        JIG_TEST_display_v2_mode_test("HB Pi Error");
                    }
                }
            }
    }
}



#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_V2_PROCESS
    @{
*/#ifndef JIG_TEST_DISPLAY_V2_PROCESS
#define JIG_TEST_DISPLAY_V2_PROCESS

/** @brief display_v2_after_reset
    @return none
*/
static void JIG_TEST_display_v2_after_reset(void)
{
    char buff[100];
    
    if(JIG_TEST_display_v2_get_scanBarcode_done() == true)
    {
//        if(JIG_TEST_display_v2_get_login() == true)
//        {
            #if (GIMBAL_FSTD_JIG_TEST == 1)
                display_private.gimbalDevice = JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id);//(gimbal_FSTD_comm.display_gimbal_id_storage);
            #endif

            JGI_TEST_display_v2_clear(JGI_TEST_DISPLAY_LINE_TWO_Y, Font_7x10);
        
            sprintf(buff, "%s", JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id));//(gimbal_FSTD_comm.display_gimbal_id_storage));
            JIG_TEST_display_writeData(   FONT_11x18
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , buff, Black, Font_11x18);

            JIG_TEST_display_writeData(   FONT_6x8
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , "BARCODE SCAN", White, Font_6x8);

            JIG_TEST_display_writeData(   FONT_7x10
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "DONE", Black, Font_7x10);
//        }
    }
    else
    {
        sprintf(buff, "%s", display_private.gimbalDevice);

        JIG_TEST_display_writeData(   FONT_11x18
                                    , JGI_TEST_DISPLAY_LINE_ONE_Y
                                    , buff, Black, Font_11x18);

        JIG_TEST_display_writeData(   FONT_6x8
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , "GIMBAL JIG TEST ", White, Font_6x8);
        
        #if (JIG_TEST_ID == 0x10 || JIG_TEST_ID == 0x34)
            JIG_TEST_display_writeData(   FONT_7x10
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "FSTD", Black, Font_7x10);
        #endif
        #if (JIG_TEST_ID == 0x11)
            JIG_TEST_display_writeData(   FONT_7x10
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "C.FSTD", Black, Font_7x10);
        #endif
        #if (JIG_TEST_ID == 0x20)
            JIG_TEST_display_writeData(   FONT_7x10
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "F.AC30K", Black, Font_7x10);
        #endif
        #if (JIG_TEST_ID == 0x21)
            JIG_TEST_display_writeData(   FONT_7x10
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "C.F.AC30K", Black, Font_7x10);
        #endif
    }
    
    JGI_TEST_display_UpdateScreen();
}

/** @brief display_v2_process
    @return none
*/
void JIG_TEST_display_v2_process(void)
{
    JIG_TEST_display_v2_main_control_process();
    
    JGI_TEST_display_UpdateScreen();
}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


