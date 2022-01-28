/** 
  ******************************************************************************
  * @file    JIG_TEST_display.c
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
#include "JIG_TEST_display.h"
#include "JIG_TEST_console.h"
#include "JIG_TEST_gimbal_FSTD.h"
#include "JIG_TEST_mavlink_gimbal.h"
#include "JIG_TEST_comm_raspberry.h"
#include "JIG_TEST_rtc.h"
#include "main.h"
#include "timeOut.h"
#include "ssd1306.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/

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
}JIG_TEST_display_t;
/* Private define ------------------------------------------------------------*/

uint8_t JGI_TEST_DISPLAY_LINE_ONE_X = 40;
uint8_t JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

uint8_t JGI_TEST_DISPLAY_LINE_TWO_X = 50;
uint8_t JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

uint8_t JGI_TEST_DISPLAY_LINE_THREE_X = 5;
uint8_t JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

uint8_t JGI_TEST_DISPLAY_LINE_FOUR_X = 5;
uint8_t JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;

#define JIG_TEST_TIME_ALL                               (JIG_TEST_TIME_GIMBAL_MODE_SBUS \
                                                        + JIG_TEST_TIME_GIMBAL_MODE_PPM \
                                                        + JIG_TEST_TIME_GIMBAL_MODE_CAN \
                                                        + JIG_TEST_TIME_GIMBAL_MODE_CONTROL_COM2 \
                                                        + JIG_TEST_TIME_GIMBAL_MODE_CONTROL_COM4 \
                                                        + JIG_TEST_TIME_GIMBAL_MODE_AUX)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
JIG_TEST_display_t display;

extern JIG_TEST_gimbal_FSTD_t                       gimbal_FSTD_comm;
extern JIG_TEST_mavlink_gimbal_t                    mavlink_gimbal_COM2;
extern JIG_TEST_comm_raspberry_global_t             raspberry_global;

struct _gremsy_device
{
    uint8_t id;
    char *name;
}gremsy_device[11] = 
{
    {.id = 0x3A, .name = "PixyU"},
    {.id = 0x0A, .name = "PixyF"},
    {.id = 0x1A, .name = "PixyFI"},
    {.id = 0x4A, .name = "PixyW"},
    {.id = 0x6A, .name = "PixyD1v2"},
    {.id = 0x44, .name = "T3v3"},
    {.id = 0x14, .name = "T3v2"},
    {.id = 0x12, .name = "S1v2"},
    {.id = 0x22, .name = "S1v3"},
    {.id = 0x08, .name = "T7"},
    {.id = 0x0B, .name = "Mio"},
    
};



/* Private function prototypes -----------------------------------------------*/
/** @brief init_after_scan_barCode_done
    @return none
*/
static void JIG_TEST_display_init_after_scan_barCode_done(bool barCode_is_Done);
/* Private functions ---------------------------------------------------------*/

/** @group JIG_TEST_DISPLAY_TEST_SCREEN
    @{
*/#ifndef JIG_TEST_DISPLAY_TEST_SCREEN
#define JIG_TEST_DISPLAY_TEST_SCREEN

void ssd1306_TestBorder() {
    ssd1306_Fill(Black);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    uint8_t x = 0;
    uint8_t y = 0;
    do {
        ssd1306_DrawPixel(x, y, Black);

        if((y == 0) && (x < 127))
            x++;
        else if((x == 127) && (y < (SSD1306_HEIGHT-1)))
            y++;
        else if((y == (SSD1306_HEIGHT-1)) && (x > 0)) 
            x--;
        else
            y--;

        ssd1306_DrawPixel(x, y, White);
        ssd1306_UpdateScreen();
    
        HAL_Delay(5);
        end = HAL_GetTick();
    } while((end - start) < 8000);
   
    HAL_Delay(1000);
}

void ssd1306_TestFonts() {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(2, 0);
    ssd1306_WriteString("Font 16x26", Font_16x26, White);
    ssd1306_SetCursor(2, 26);
    ssd1306_WriteString("Font 11x18", Font_11x18, White);
    ssd1306_SetCursor(2, 26+18);
    ssd1306_WriteString("Font 7x10", Font_7x10, White);
    ssd1306_SetCursor(2, 26+18+10);
    ssd1306_WriteString("Font 6x8", Font_6x8, White);
    ssd1306_UpdateScreen();
}

void ssd1306_TestFPS() {
    ssd1306_Fill(White);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    int fps = 0;
    char message[] = "ABCDEFGHIJK";
   
    ssd1306_SetCursor(2,0);
    ssd1306_WriteString("Testing...", Font_11x18, Black);
   
    do {
        ssd1306_SetCursor(2, 18);
        ssd1306_WriteString(message, Font_11x18, Black);
        ssd1306_UpdateScreen();
       
        char ch = message[0];
        memmove(message, message+1, sizeof(message)-2);
        message[sizeof(message)-2] = ch;

        fps++;
        end = HAL_GetTick();
    } while((end - start) < 5000);
   
    HAL_Delay(1000);

    char buff[64];
    fps = (float)fps / ((end - start) / 1000.0);
    snprintf(buff, sizeof(buff), "~%d FPS", fps);
   
    ssd1306_Fill(White);
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString(buff, Font_11x18, Black);
    ssd1306_UpdateScreen();
}

void ssd1306_TestLine() {

  ssd1306_Line(1,1,SSD1306_WIDTH - 1,SSD1306_HEIGHT - 1,White);
  ssd1306_Line(SSD1306_WIDTH - 1,1,1,SSD1306_HEIGHT - 1,White);
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestRectangle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawRectangle(1 + (5*delta),1 + (5*delta) ,SSD1306_WIDTH-1 - (5*delta),SSD1306_HEIGHT-1 - (5*delta),White);
  }
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestCircle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    ssd1306_DrawCircle(20* delta+30, 15, 10, White);
  }
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestArc() {
  ssd1306_DrawArc(30, 30, 30, 20, 270, White);
  ssd1306_UpdateScreen();
  return;
}

void ssd1306_TestPolyline() {
  SSD1306_VERTEX loc_vertex[] =
  {
      {35,40},
      {40,20},
      {45,28},
      {50,10},
      {45,16},
      {50,10},
      {53,16}
  };

  ssd1306_Polyline(loc_vertex,sizeof(loc_vertex)/sizeof(loc_vertex[0]),White);
  ssd1306_UpdateScreen();
  return;
}

#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_WRITE_DATA
    @{
*/#ifndef JIG_TEST_DISPLAY_WRITE_DATA
#define JIG_TEST_DISPLAY_WRITE_DATA

/** @brief display_ErrorHandle
    @return none
*/
static void JIG_TEST_display_ErrorHandle(void)
{
    static uint8_t error;
    
    /// error device ready
    if(display.error.is_Ready == true)
    {
        if(get_timeOut(1000, DISPLAY_ERROR_DEVICE_READY))
        {
            /// write error Console
            error ++;
        }
    }
}

/** @brief display_writeData
    @return none
*/
static void JIG_TEST_display_writeData(uint8_t x, uint8_t y, char *str, SSD1306_COLOR color, FontDef font)
{
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(str, font, color);
}

/** @brief display_UpdateScreen
    @return none
*/
static void JGI_TEST_display_UpdateScreen(void)
{
    ssd1306_UpdateScreen();
}

/** @brief display_clearScreenALL
    @return none
*/
static void JIG_TEST_display_clearScreenALL()
{
//    ssd1306_ClearSreen();
    
    /// Setting line position
    JGI_TEST_DISPLAY_LINE_ONE_X = 0;
    JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

    JGI_TEST_DISPLAY_LINE_TWO_X = 0;
    JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

    JGI_TEST_DISPLAY_LINE_THREE_X = 0;
    JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

    JGI_TEST_DISPLAY_LINE_FOUR_X = 0;
    JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
    
//    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_ONE_X
//                                , JGI_TEST_DISPLAY_LINE_ONE_Y
//                                , "                      ", White, Font_11x18);
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_TWO_X
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , "                       ", White, Font_6x8);
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_THREE_X
                                , JGI_TEST_DISPLAY_LINE_THREE_Y
                                , "                       ", White, Font_6x8);
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_FOUR_X
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , "                      ", White, Font_6x8);
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_FOUR_X
                                , 50
                                , "                       ", White, Font_6x8);

}

/** @brief display_clear_three_line
    @return none
*/
static void JIG_TEST_display_clear_three_line(void)
{
//    ssd1306_ClearSreen();
    
    /// Setting line position
    JGI_TEST_DISPLAY_LINE_ONE_X = 0;
    JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

    JGI_TEST_DISPLAY_LINE_TWO_X = 0;
    JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

    JGI_TEST_DISPLAY_LINE_THREE_X = 0;
    JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

    JGI_TEST_DISPLAY_LINE_FOUR_X = 0;
    JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_TWO_X
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , "                    ", White, Font_7x10);
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_THREE_X
                                , JGI_TEST_DISPLAY_LINE_THREE_Y
                                , "                    ", White, Font_7x10);
                                
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_FOUR_X
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , "                    ", White, Font_7x10);
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_FOUR_X
                                , 55
                                , "                    ", White, Font_7x10);
}

/** @brief display_clearScreen
    @return none
*/
static void JIG_TEST_display_clearScreen_mode()
{
//    ssd1306_ClearSreen();
    
    /// Setting line position
    JGI_TEST_DISPLAY_LINE_ONE_X = 0;
    JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

    JGI_TEST_DISPLAY_LINE_TWO_X = 0;
    JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

    JGI_TEST_DISPLAY_LINE_THREE_X = 0;
    JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

    JGI_TEST_DISPLAY_LINE_FOUR_X = 85;
    JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_TWO_X
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , "                        ", White, Font_6x8);
    
    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_THREE_X
                                , JGI_TEST_DISPLAY_LINE_THREE_Y
                                , "                        ", White, Font_6x8);
//    JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_FOUR_X
//                                , 50
//                                , "                   ", White, Font_6x8);

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

#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_PROCESS
    @{
*/#ifndef JIG_TEST_DISPLAY_PROCESS
#define JIG_TEST_DISPLAY_PROCESS

/** @brief display_mode_IDLE
    @return none
*/
static void JIG_TEST_display_mode_IDLE(uint8_t state)
{
    char buff[SSD1306_WIDTH];
//    static uint8_t count_state;
    static bool check_calib;
    uint16_t pos_start = 0;
    
    /// Setting line position
    JGI_TEST_DISPLAY_LINE_ONE_X = 40;
    JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

    JGI_TEST_DISPLAY_LINE_TWO_X = 20;
    JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

    JGI_TEST_DISPLAY_LINE_THREE_X = 5;
    JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

    
    JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
    
    if(state == 0)
    {
            /// hien thi man hinh cho triger tu raspberry
            JGI_TEST_DISPLAY_LINE_FOUR_X = 40;
            JGI_TEST_DISPLAY_LINE_TWO_X = 27;
            
            JIG_TEST_display_writeData(   JGI_TEST_DISPLAY_LINE_ONE_X
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , "                         ", White, Font_11x18);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , "                          ", White, Font_7x10);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_THREE_Y
                                        , "                          ", White, Font_7x10);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "                          ", White, Font_7x10);
            
            sprintf(buff, "%s", display.gimbalDevice);
            /// get pos start display line one
            pos_start = JIG_TEST_display_get_center_screen(buff, 11);
            
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , buff, Black, Font_11x18);
            
            sprintf(buff, "GREMSY JIG TEST");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 6);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , buff, White, Font_6x8);
            
            sprintf(buff, "RASPBERRY ERROR");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 7);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , buff, Black, Font_7x10);
                                        
    }
    else if(state == 1)
    {
        char str_total[10];
        uint16_t pos_start = 0;
        
        //// wait for scan barcode
        if(gimbal_FSTD_comm.barcode_ready == true)
        {
            /// hien thi man hinh cho triger tu raspberry
            JGI_TEST_DISPLAY_LINE_FOUR_X = 40;
            JGI_TEST_DISPLAY_LINE_TWO_X = 27;
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , "                         ", White, Font_11x18);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , "                          ", White, Font_7x10);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_THREE_Y
                                        , "                          ", White, Font_7x10);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "                          ", White, Font_7x10);
            

            sprintf(buff, "%s", "GREMSY");
            /// get pos start display line one
            pos_start = JIG_TEST_display_get_center_screen(buff, 11);
            
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , buff, Black, Font_11x18);
            
            sprintf(buff, "WAIT FOR SCAN");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 6);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , buff, White, Font_6x8);
            
            sprintf(str_total, "BARCODE");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(str_total, 7);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , str_total, Black, Font_7x10);
                                        
        }

        
        
        /// run test
        if(gimbal_FSTD_comm.start_stopSystem == 1)
        {
            uint8_t state_gimbal_calib_motor = JIG_TEST_mavlink_gimbal_get_state_calib_motor();
            uint8_t state_gimbal_calib_imu = JIG_TEST_mavlink_gimbal_get_state_calib_imu();
            
            #if (GIMBAL_FSTD_JIG_TEST == 0)
                JGI_TEST_DISPLAY_LINE_ONE_X = 30;
            #endif
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , "                             ", Black, Font_11x18);
            
            sprintf(buff, "%s", display.gimbalDevice);
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 11);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , buff, Black, Font_11x18);

            

            if(state_gimbal_calib_motor == 1 && state_gimbal_calib_imu == 1)
            {
                JGI_TEST_DISPLAY_LINE_FOUR_X = 30;
                JGI_TEST_DISPLAY_LINE_TWO_X = 20;
                
                JIG_TEST_display_writeData(   0
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , "                          ", White, Font_7x10);
                
                sprintf(buff, "GIMBAL STATE CALIB ");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(buff, 6);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_TWO_Y
                                            , buff, White, Font_6x8);
                
                sprintf(str_total, "MOTOR-IMU");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(str_total, 7);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , str_total, Black, Font_7x10);
                
                /// set flag check calib
                check_calib = true;
            }
            else if(state_gimbal_calib_motor == 2 && state_gimbal_calib_imu == 1)
            {
                JGI_TEST_DISPLAY_LINE_FOUR_X = 60;
                JGI_TEST_DISPLAY_LINE_TWO_X = 20;

                JIG_TEST_display_writeData(   0
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , "                          ", White, Font_7x10);
                
                sprintf(buff, "GIMBAL STATE CALIB");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(buff, 6);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_TWO_Y
                                            , buff, White, Font_6x8);
                
                sprintf(str_total, "IMU");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(str_total, 7);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , str_total, Black, Font_7x10);
                
                /// set flag check calib
                check_calib = true;
            }
            else if(state_gimbal_calib_motor == 1 && state_gimbal_calib_imu == 2)
            {
                JGI_TEST_DISPLAY_LINE_FOUR_X = 50;
                JGI_TEST_DISPLAY_LINE_TWO_X = 20;
                
                JIG_TEST_display_writeData(   0
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , "                          ", White, Font_7x10);
                
                sprintf(buff, "GIMBAL STATE CALIB ");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(buff, 6);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_TWO_Y
                                            , buff, White, Font_6x8);
                
                sprintf(str_total, "MOTOR");
                /// get pos start display line two
                pos_start = JIG_TEST_display_get_center_screen(str_total, 7);
                JIG_TEST_display_writeData(   pos_start
                                            , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                            , str_total, Black, Font_7x10);
                
                /// set flag check calib
                check_calib = true;
            }
        }
    }
}

/** @brief display_get_string_result
    @return none
*/
static void JIG_TEST_display_get_string_result(uint8_t mode_test, char *str1, char *str2, char *str3)
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

/** @brief display_mode_ALL
    @return none
*/
static void JIG_TEST_display_mode_ALL(JIG_TEST_gimbal_FSTD_mode_test_t mode)
{
    char buff[SSD1306_WIDTH];
    static uint8_t count;
    static uint8_t count_clear;
    static bool copy_data;
    static uint8_t usb_speed_data;
    static uint8_t total_result_count;
    uint16_t pos_start = 0;
    
    
    char buff_result_2[SSD1306_WIDTH];
    
    /// Setting line position
    JGI_TEST_DISPLAY_LINE_ONE_X = 40;
    JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

    JGI_TEST_DISPLAY_LINE_TWO_X = 52;
    JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

    JGI_TEST_DISPLAY_LINE_THREE_X = 5;
    JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

    JGI_TEST_DISPLAY_LINE_FOUR_X = 25;
    JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
    
    /// Update gimbal device 
    
    #if (GIMBAL_FSTD_JIG_TEST == 0)
        JGI_TEST_DISPLAY_LINE_ONE_X = 30;
    #endif
    
    sprintf(buff, "%s", display.gimbalDevice);
    
    /// get pos start display line two
    pos_start = JIG_TEST_display_get_center_screen(buff, 11);
    JIG_TEST_display_writeData(   pos_start
                                , JGI_TEST_DISPLAY_LINE_ONE_Y
                                , buff, Black, Font_11x18);

    
    if(mode == JIG_TEST_GIMBAL_MODE_SBUS)
    {
        sprintf(buff_result_2, "SBUS");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_PPM)
    {
        sprintf(buff_result_2, "PPM");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_CAN)
    {
        sprintf(buff_result_2, "CAN");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_COM)
    {
        sprintf(buff_result_2, "COM2");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_COM4)
    {
        sprintf(buff_result_2, "COM4");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_AUX)
    {
        sprintf(buff_result_2, "AUX");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_VIBRATE)
    {
        JGI_TEST_DISPLAY_LINE_TWO_X = 42;
        
        sprintf(buff_result_2, "VIBRATE");
    }
    else if(mode == JIG_TEST_GIMBAL_MODE_DONE)
    {
/*
//        ssd1306_TestFPS();
//        HAL_Delay(3000);
//        ssd1306_TestBorder();
//        ssd1306_TestFonts();
//        HAL_Delay(3000);
//        ssd1306_Fill(Black);
//        ssd1306_TestRectangle();
//        ssd1306_TestLine();
//        HAL_Delay(3000);
//        ssd1306_Fill(Black);
//        ssd1306_TestPolyline();
//        HAL_Delay(3000);
//        ssd1306_Fill(Black);
//        ssd1306_TestArc();
//        HAL_Delay(3000);
//        ssd1306_Fill(Black);
//        ssd1306_TestCircle();
//        HAL_Delay(3000);
*/
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
            if(display.total_result == false)
            {
                copy_data = true;
                
                if(gimbal_FSTD_comm.result_pushData != 0)
                {
                    for(uint8_t i = 1; i < JIG_TEST_GIMBAL_MODE_TOTAL - 1; i ++)
                    {
                        if(gimbal_FSTD_comm.mode_test_result[i] == true)
                        {
                            total_result_count ++;
                        }
                    }
                    #if (GIMBAL_FSTD_JIG_TEST == 1)
                    
                        /// kiem tra tong cac ket qua test
                        if(total_result_count == 8)
                        {
                            display.total_result = true;
                        }
                        
                    #else
                        /// lay gia tri result test usb speed
                        if(gimbal_FSTD_comm.usb_speed_result == 3)
                        {
                            total_result_count ++;
                        }
                    
                        /// kiem tra tong cac ket qua test
                        if(total_result_count == 9)
                        {
                            display.total_result = true;
                        }
                    
                    #endif
                }

            }
            #if (GIMBAL_FSTD_JIG_TEST == 0)
                /// copy data result jig test full gimbal signal
                for(uint8_t i = 0; i <= (number_of_result - 4); i++)
                {
                    if(i <= 3)
                    {
                        JIG_TEST_display_get_string_result(   display.total_result
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                    }
                    else if(i > 3)
                    {
                        JIG_TEST_display_get_string_result(   gimbal_FSTD_comm.mode_test_result[i - 3]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                    }
                }
            #else
                /// copy data result jig test full gimbal signal
                for(uint8_t i = 0; i <= number_of_result; i++)
                {
                    if(i <= 3)
                    {
                        JIG_TEST_display_get_string_result(   display.total_result
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                    }
                    else if(i > 3)
                    {
                        JIG_TEST_display_get_string_result(   gimbal_FSTD_comm.mode_test_result[i - 3]
                                                            , str_result[i]
                                                            , jig_test_result[i].result_ok
                                                            , jig_test_result[i].result_error);
                    }
                }
            #endif
            #if (GIMBAL_FSTD_JIG_TEST == 0)
            
            for(uint8_t i = (number_of_result - 3); i <= number_of_result; i ++)
            {
                if(gimbal_FSTD_comm.usb_speed_result == 0)
                {
                    strcpy(str_result[i], usb_speed_result_none_test);
                }
                else if(gimbal_FSTD_comm.usb_speed_result == 1)
                {
                    strcpy(str_result[i], usb_speed_result_no_usb);
                }
                else if(gimbal_FSTD_comm.usb_speed_result == 2)
                {
                    strcpy(str_result[i], usb_speed_result_low_speed);
                }
                else if(gimbal_FSTD_comm.usb_speed_result == 3)
                {
                    strcpy(str_result[i], usb_speed_result_passed);
                }
            }
            #endif
        }

        /// Update result after 0.25s
        if(get_timeOut(10, JIG_TEST_TIMEOUT_GIMBAL_MODE_SBUS))
        {
            
            if(count_clear % 2 == 0)
            {
                #if (GIMBAL_FSTD_JIG_TEST == 1)
                    if(count > 3)
                    JIG_TEST_display_clear_three_line();
                #else
                    if(count > 3 && count <= 11)
                    JIG_TEST_display_clear_three_line();
                #endif
                
                /// clear screen lan dau
                if(count == 0)
                {
                   JIG_TEST_display_clear_three_line();
                }
            }
            else
            {
                
//                ssd1306_Fill(Black);               
                if(count < 4)
                {
                    /// get pos start display line two
                    pos_start = JIG_TEST_display_get_center_screen(str_result[count], 16);
                    
                    /// write data to screen
                    JIG_TEST_display_writeData(   pos_start
                                                , 25
                                                , str_result[count], Black, Font_16x26);
                }
                else if(count == 10)
                {
                    /// get pos start display line two
                    pos_start = JIG_TEST_display_get_center_screen(str_result[count], 7);   
                    
                    /// write data to screen
                    JIG_TEST_display_writeData(   pos_start
                                                , 35
                                                , str_result[count], White, Font_7x10);
                }
                #if (GIMBAL_FSTD_JIG_TEST == 0)
                    else if(count >= (number_of_result - 3) && count <= number_of_result)
                    {
                        /// get pos start display line two
                        pos_start = JIG_TEST_display_get_center_screen(str_result[count], 6);   
                
                        /// write data to screen USB result
                        JIG_TEST_display_writeData(   pos_start
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

                            sprintf(str_read_speed_temp, "USB_r:%.2f", gimbal_FSTD_comm.value_read_speed);
                            sprintf(str_ref_read_speed, "/%.2f", gimbal_FSTD_comm.value_ref_read_speed);
                            
                            sprintf(str_write_speed_temp, "USB_w:%.2f", gimbal_FSTD_comm.value_write_speed);
                            sprintf(str_ref_write_speed, "/%.2f", gimbal_FSTD_comm.value_ref_write_speed);
                            
                            strcpy(str_read_speed, str_read_speed_temp);
                            strcat(str_read_speed, str_ref_read_speed);
                            
                            strcpy(str_write_speed, str_write_speed_temp);
                            strcat(str_write_speed, str_ref_write_speed);
                            
                        }
                        
                        /// write data to screen
                        /// get pos start display line two
                        pos_start = JIG_TEST_display_get_center_screen(str_read_speed, 6);   
                        JIG_TEST_display_writeData(pos_start, 35, str_read_speed, White, Font_6x8);
                        
                        /// get pos start display line two
                        pos_start = JIG_TEST_display_get_center_screen(str_write_speed, 6);   
                        JIG_TEST_display_writeData(pos_start, 45, str_write_speed, White, Font_6x8);
                    }
                #endif
                else
                {
                    /// get pos start display line two
                    pos_start = JIG_TEST_display_get_center_screen(str_result[count], 7);   
                    
                    /// write data to screen
                    JIG_TEST_display_writeData(pos_start, 35, str_result[count], White, Font_7x10);
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
    
    if(mode != JIG_TEST_GIMBAL_MODE_DONE)
    {
        /// get pos start display line two
        pos_start = JIG_TEST_display_get_center_screen(buff_result_2, 6);   
        
        /// write data to screen
        JIG_TEST_display_writeData(   pos_start
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , buff_result_2, White, Font_6x8);
 
        sprintf(buff, "Time test: %d", gimbal_FSTD_comm.total_time);
        
        /// get pos start display line two
        pos_start = JIG_TEST_display_get_center_screen(buff, 6);   
        /// write data to screen
        JIG_TEST_display_writeData(   pos_start
                                    , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                    , buff, White, Font_6x8);
    }
}


/** @brief display_get_device_name
    @return none
*/
static void JIG_TEST_display_login(bool login)
{
    char login_buff[100];
    uint16_t pos_start = 0;
    
    JGI_TEST_DISPLAY_LINE_FOUR_X = 40;
    JGI_TEST_DISPLAY_LINE_TWO_X = 27;
    
    JIG_TEST_display_writeData(   0
                                , JGI_TEST_DISPLAY_LINE_ONE_Y
                                , "                         ", White, Font_11x18);
    
    JIG_TEST_display_writeData(   0
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , "                          ", White, Font_7x10);
    
    JIG_TEST_display_writeData(   0
                                , JGI_TEST_DISPLAY_LINE_THREE_Y
                                , "                          ", White, Font_7x10);
    
    JIG_TEST_display_writeData(   0
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , "                          ", White, Font_7x10);
    

    sprintf(login_buff, "%s", "GREMSY");
    /// get pos start display line one
    pos_start = JIG_TEST_display_get_center_screen(login_buff, 11);
    
    JIG_TEST_display_writeData(   pos_start
                                , JGI_TEST_DISPLAY_LINE_ONE_Y
                                , login_buff, Black, Font_11x18);
    if(login == false)
    {
        sprintf(login_buff, "WAITING FOR USER");
    }
    else
    {
        sprintf(login_buff, "LOGINED IN USER");
    }
    
    /// get pos start display line two
    pos_start = JIG_TEST_display_get_center_screen(login_buff, 6);
    JIG_TEST_display_writeData(   pos_start
                                , JGI_TEST_DISPLAY_LINE_TWO_Y
                                , login_buff, White, Font_6x8);
    
    if(login == false)
    {
        sprintf(login_buff, "LOGIN");
    }
    else
    {
        sprintf(login_buff, "                        ");
    }
    
    /// get pos start display line two
    pos_start = JIG_TEST_display_get_center_screen(login_buff, 7);
    JIG_TEST_display_writeData(   pos_start
                                , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                , login_buff, Black, Font_7x10);
}

/** @brief display_get_device_name
    @return none
*/
static char* JIG_TEST_display_get_device_name(uint8_t id)
{
    for(uint8_t i = 0; i < 11 ; i++)
    {
        if(id == gremsy_device[i].id)
        {
            return gremsy_device[i].name;
        }
        else if(id == 0)
        {
            return "GREMSY";
        }
    }
}

/** @brief display_process
    @return none
*/
void JIG_TEST_display_process(void)
{
    static JIG_TEST_gimbal_FSTD_mode_test_t JIG_TEST_mode;
    char *str = "DISPLAY --->";
    uint16_t pos_start;
    static bool first_display_mode;
    static uint8_t use_gimbal_name;
    static uint8_t count;
    static uint8_t mode_pre;
    static uint8_t mode_new;
    bool screen_test = false;
   
    
    JIG_TEST_mode = gimbal_FSTD_comm.mode_test;
    
    mode_new = (uint8_t)JIG_TEST_mode;
    
    uint8_t check_new_mode = (mode_new - mode_pre);
    
    if( check_new_mode == 1 )
    {
        display.is_clearScreen = false;
    }
    
    /// kiem tra gimbal device
    if(mavlink_gimbal_COM2.vehicle_system_id != 0)
    {
        #if (GIMBAL_FSTD_JIG_TEST == 1)
            display.gimbalDevice = JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id);
        #endif
        
        if(use_gimbal_name == 0)
        {
            use_gimbal_name = 1;
        }
    }
    
    /// test hien thi man hinh khong map voi mode test gimbal
    if(screen_test == true)
    {
        JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_DONE); /// test only mode test done
        /// Update display
        JGI_TEST_display_UpdateScreen();
    }

    //// hien thi cac mode test gimbal
    if(display.is_deviceReady == true && gimbal_FSTD_comm.wait_init_after_reset == true)
    {
        if(raspberry_global.user_login == JIG_TEST_CLOUD_LOGINED)
        {
            if(gimbal_FSTD_comm.user_logined == false && gimbal_FSTD_comm.reciver_reset_msg_when_run_test == false)
            {
                JIG_TEST_display_login(true);
                
                /// ghi nho da co user login
                gimbal_FSTD_comm.user_logined = true;
                JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.user_logined, LL_RTC_BKP_DR13);
            }
            
            if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_IDLE)
            {
                if(raspberry_global.disconnect == true)
                {
                    JIG_TEST_display_mode_IDLE(0);
                }
                else
                {
                    JIG_TEST_display_mode_IDLE(1);
                }
                    
    //            JIG_TEST_console_write(str);
    //            JIG_TEST_console_write("JIG_TEST_GIMBAL_MODE_IDLE\n");
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_SBUS)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreenALL();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_SBUS;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_SBUS);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_PPM)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_PPM;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_PPM);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_CAN)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_CAN;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_CAN);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_COM)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_COM;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_COM);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_COM4)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_COM4;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_COM4);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_AUX)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_AUX;
                }
                else
                {
                    /// display procedure aux test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_AUX);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_VIBRATE)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreen_mode();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_VIBRATE;
                }
                else
                {
                    /// display procedure sbus test
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_VIBRATE);
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_DONE)
            {
                static bool clear_screen_for_result_total;
                
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreenALL();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_DONE;
                }
                else
                {
                    if(gimbal_FSTD_comm.result_pushData == 0)
                    {
                        char buff1[100];
                        char buff2[100];
                        
                        JGI_TEST_DISPLAY_LINE_FOUR_X = 25;
                        JGI_TEST_DISPLAY_LINE_TWO_X = 10;
                        
                        JIG_TEST_display_writeData(   0
                                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                                    , "                          ", White, Font_7x10);
                        
                        JIG_TEST_display_writeData(   0
                                                    , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                                    , "                          ", White, Font_7x10);
                        
                        sprintf(buff1, "WAITTING FOR PUSH");
                        
                        /// get pos start display line two
                        pos_start = JIG_TEST_display_get_center_screen(buff1, 6);
                        JIG_TEST_display_writeData(   pos_start
                                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                                    , buff1, White, Font_6x8);
                        
                        sprintf(buff2, "DATA CLOUD");
                        
                        /// get pos start display line two
                        pos_start = JIG_TEST_display_get_center_screen(buff2, 7);
                        JIG_TEST_display_writeData(   pos_start
                                                    , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                                    , buff2, Black, Font_7x10);
                    }
                    else
                    {
                        if(clear_screen_for_result_total == false)
                        {
                            clear_screen_for_result_total = true;
                            
                            //// clear line 4 oled
                            JIG_TEST_display_writeData(   0
                                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                                        , "                         ", White, Font_7x10);
                        }

                        
                        /// display on result mode test
                        JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_DONE);
                    }
                }
            }
            else if(JIG_TEST_mode == JIG_TEST_GIMBAL_MODE_ERROR)
            {
                /// clear SCreen
                if(display.is_clearScreen == false)
                {
                    /// reset flag clear SCreen
                    display.is_clearScreen = true;
                    
                    /// clear SCreen
                    JIG_TEST_display_clearScreenALL();
                    
                    /// get mode
                    mode_pre = (uint8_t)JIG_TEST_GIMBAL_MODE_ERROR;
                }
                else
                {
                    /// display on result mode test Error
                    JIG_TEST_display_mode_ALL(JIG_TEST_GIMBAL_MODE_DONE);
                }
            }
        }
        else
        {
            JIG_TEST_display_login(false);
            
            /// xoa ghi nho da co user login
            gimbal_FSTD_comm.user_logined = false;
            JIG_TEST_rtc_storage_register_value(gimbal_FSTD_comm.user_logined, LL_RTC_BKP_DR13);
        }
            /// Update display
        JGI_TEST_display_UpdateScreen();
    }
    else
    {
        /// display error
        JIG_TEST_display_ErrorHandle();
    }
}



/** @brief init_after_scan_barCode_done
    @return none
*/
static void JIG_TEST_display_init_after_scan_barCode_done(bool barCode_is_Done)
{
    char buff[100];
    char str_total[50];
    uint16_t pos_start = 0;
    
    if(barCode_is_Done == false)
    {
        /// Setting line position
        JGI_TEST_DISPLAY_LINE_ONE_X = 40;
        JGI_TEST_DISPLAY_LINE_ONE_Y = 0;

        JGI_TEST_DISPLAY_LINE_TWO_X = 20;
        JGI_TEST_DISPLAY_LINE_TWO_Y = 25;

        JGI_TEST_DISPLAY_LINE_THREE_X = 5;
        JGI_TEST_DISPLAY_LINE_THREE_Y = 35;

        
        JGI_TEST_DISPLAY_LINE_FOUR_Y = 45;
        
        sprintf(buff, "%s", display.gimbalDevice);
        /// get pos start display line two
        pos_start = JIG_TEST_display_get_center_screen(buff, 11);
        JIG_TEST_display_writeData(   pos_start
                                    , JGI_TEST_DISPLAY_LINE_ONE_Y
                                    , buff, Black, Font_11x18);
        
        sprintf(buff, "GIMBAL JIG TEST ");
        /// get pos start display line two
        pos_start = JIG_TEST_display_get_center_screen(buff, 6);
        JIG_TEST_display_writeData(   pos_start
                                    , JGI_TEST_DISPLAY_LINE_TWO_Y
                                    , buff, White, Font_6x8);
        
        #if (GIMBAL_FSTD_JIG_TEST == 1)

            sprintf(buff, "FSTD");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 7);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , buff, Black, Font_7x10);
        #else
            JGI_TEST_DISPLAY_LINE_FOUR_X = 40;
            sprintf(buff, "F.AC30K");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 7);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , buff, Black, Font_7x10);
        #endif
    }
    else if(barCode_is_Done == true)
    {
        if(gimbal_FSTD_comm.user_logined == true)
        {
            #if (GIMBAL_FSTD_JIG_TEST == 1)
                display.gimbalDevice = JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id);//(gimbal_FSTD_comm.display_gimbal_id_storage);
            #endif
            
            /// hien thi man hinh cho triger tu raspberry
            JGI_TEST_DISPLAY_LINE_FOUR_X = 50;
            JGI_TEST_DISPLAY_LINE_TWO_X = 30;
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , "                          ", White, Font_7x10);
            
            JIG_TEST_display_writeData(   0
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , "                          ", White, Font_7x10);
            
            sprintf(buff, "%s", JIG_TEST_display_get_device_name(mavlink_gimbal_COM2.vehicle_system_id));//(gimbal_FSTD_comm.display_gimbal_id_storage));
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 11);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_ONE_Y
                                        , buff, Black, Font_11x18);
            
            sprintf(buff, "BARCODE SCAN");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(buff, 6);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_TWO_Y
                                        , buff, White, Font_6x8);
            
            sprintf(str_total, "DONE");
            /// get pos start display line two
            pos_start = JIG_TEST_display_get_center_screen(str_total, 7);
            JIG_TEST_display_writeData(   pos_start
                                        , JGI_TEST_DISPLAY_LINE_FOUR_Y
                                        , str_total, Black, Font_7x10);
        }

    }
    
    ssd1306_UpdateScreen();
}

#endif
/**
    @}
*/

/** @group JIG_TEST_DISPLAY_CONFIGURATION
    @{
*/#ifndef JIG_TEST_DISPLAY_CONFIGURATION
#define JIG_TEST_DISPLAY_CONFIGURATION

/** @brief display_configuration
    @return none
*/
void JIG_TEST_display_configuration(void)
{
    if(ssd1306_Init() == 1)
    {
        display.is_deviceReady = true;
        
        /// write to console i2c device status
        JIG_TEST_console_write("------------- DISPLAY INIT DONE -------------------\n");
    }
    else
    {
        /// set flag displat error device ready
        display.error.is_Ready = true;
        
        /// write to console i2c device status
        JIG_TEST_console_write("------------- DISPLAY INIT ERROR -------------------\n");
    }
    
    #if (GIMBAL_FSTD_JIG_TEST == 1)
    /// set default gimbal Device
    display.gimbalDevice = "GREMSY";
    #else
        /// set default gimbal Device
        display.gimbalDevice = "AC30000";
    #endif
    
    /// display screen after reset or power supply
    JIG_TEST_display_init_after_scan_barCode_done(gimbal_FSTD_comm.scan_barCode_done);
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

