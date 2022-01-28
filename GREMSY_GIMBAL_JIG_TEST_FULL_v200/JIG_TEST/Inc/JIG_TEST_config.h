/** 
  ******************************************************************************
  * @file    JIG_TEST_config.h
  * @author  Gremsy Team
  * @version V2.0.1
  * @date    2021
  * @brief   This file contains all the functions prototypes for the | 
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

#ifndef __JIG_TEST_CONFIG_H
#define __JIG_TEST_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//*** <<< Use Configuration Wizard in Context Menu >>> ***

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

// <h> Jig test library

// <c1> Use LIBRARY_V1
// <i> Use LIBRARY_V1, and set the include file
//#define USE_LIBRARY_V1    1
//
// </c>

// <c1> Use LIBRARY_V2
// <i> Use LIBRARY_V2, and set the include file
#define USE_LIBRARY_V2      1
// </c>
	
// <c1> COMM_ESP32
// <i> COMM_ESP32, and set the include file
//#define COMM_ESP32      1
// </c>

// </h>

// <h> JIG TEST ID
// <o> select jig test
//  <i>Default: FSTD
//  <0x10=> FSTD
//  <0x20=> FAC30K
//  <0x11=> CLOUD_FSTD
//  <0x21=> CLOUD_FAC30K
//  <0x34=> GIMBAL 2 AXIS
#define JIG_TEST_ID 0x34
// </h>

// <h> Button
// <o> Set button count noise
//  <i>Default: 100000 
//  <0-16777215>
#define BUTTON_NOISE 100000

// <c1> Use BUTTON_NONE_STLINK
// <i> Use BUTTON_NONE_STLINK, and set the include file
//#define BUTTON_NONE_STLINK      1
// </c>
// </h>

// <h> Console

// <c1> Use Console comm raspberry
// <i> printf Console comm raspberry
//#define USE_CONSOLE_COMM_RAS_CLOUD      1
// </c>

// </h>

// <h> Virate

// <c1> Use S1v3 virate test
// <i> Use S1v3 virate test
//#define S1v3_VIRATE_TEST      1
// </c>

// </h>

// <h> Run sample code

// <c1> Run test sample code not main control
// <i> Run test sample code not main control
//#define RUN_SAMPLE_CODE      1
// </c>

// </h>

//*** <<< end of configuration section >>>    ***

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __JIG_TEST_CONFIG_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

