/*******************************************************************************
 * Copyright (c) 2021, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gGimbal.h
 * @author  The GremsyCo
 * @version V2.0.0
 * @date    2021
 * @brief   This file contains expand of gMavlink
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "../mavlink_v2/mavlink_avoid_errors.h"
/* Exported Define------------------------------------------------------------*/
// we have separate helpers disabled to make it possible
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */

#include "../mavlink_v2/ardupilotmega/version.h"

#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "../mavlink_v2/mavlink_types.h"

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Sanity check MAVLink channel
///
/// @param chan		Channel to send to
static inline bool valid_channel(mavlink_channel_t chan)
{
//#pragma clang diagnostic push
//#pragma clang diagnostic ignored "-Wtautological-constant-out-of-range-compare"
    return chan < MAVLINK_COMM_NUM_BUFFERS;
//#pragma clang diagnostic pop
}

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "../mavlink_v2/ardupilotmega/mavlink.h"

#define ONBOARD_CHANNEL                     MAVLINK_COMM_1
#define SYSID_ONBOARD                       4

#define SEARCH_MS                           60000   // search for gimbal for 1 minute after startup
#define TELEMETRY_MAVLINK_MAXRATE           50
#define TELEMETRY_MAVLINK_DELAY            (1000 / TELEMETRY_MAVLINK_MAXRATE)*300 // unit ms

/*
* Brief: Define expand command available mavlink
*/

typedef struct _mav_state
{
    mavlink_message_t   rxmsg;
    mavlink_status_t    status;
} mav_state_t;

/**
 * @brief  This is function initialize COM platform, protocol
 * @param   in: none
 * @param   out: none
 * @return none
 */
void mavlinkProtocol_init(void);

/** @brief serialPort3_init
    @return none
*/
void mavlinkProtocol_serialPort3_init(void);

/** @brief serialPort3_Deinit
    @return none
*/
void mavlinkProtocol_serialPort3_Deinit(void);

/** @brief serialPort4_init
    @return none
*/
void mavlinkProtocol_serialPort4_init(void);

/** @brief serialPort4_Deinit
    @return none
*/
void mavlinkProtocol_serialPort4_Deinit(void);

/** @brief serialPort5_init
    @return none
*/
void mavlinkProtocol_serialPort5_init(void);

/** @brief serialPort5_Deinit
    @return none
*/
void mavlinkProtocol_serialPort5_Deinit(void);

/*
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t mavlinkProtocol_serialPort4_readData(mav_state_t *mav);
uint8_t mavlinkProtocol_serialPort3_readData(mav_state_t *mav);

/**
 * @brief  This is function read data from raspberry
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t mavlinkProtocol_serialPort5_readData(mav_state_t *mav);

/** @brief send a buffer out a MAVLink channel
    @param[in] chan
    @param[in] buf
    @param[in] len
    @return none
*/
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

void gGimbal_Console(uint8_t *str);
/*********** Portions COPYRIGHT 2021 Gremsy.Co., Ltd.*****END OF FILE****/
