/*******************************************************************************
 * Copyright (c) 2018, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gGimbal.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    August-021-2018
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


enum ap_var_type {
	AP_PARAM_NONE    = 0,
	AP_PARAM_INT8,
	AP_PARAM_INT16,
	AP_PARAM_INT32,
	AP_PARAM_FLOAT,
	AP_PARAM_VECTOR3F,
	AP_PARAM_GROUP
};

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

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t chr)
{
    if (!valid_channel(chan)) {
        return;
    }
//    mavlink_comm_port[chan]->write(chr);
}

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
uint8_t comm_receive_ch(mavlink_channel_t chan);

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan);


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

/*
  return true if the MAVLink parser is idle, so there is no partly parsed
  MAVLink message being processed
 */
bool comm_is_idle(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "../mavlink_v2/ardupilotmega/mavlink.h"

#define ONBOARD_CHANNEL                     MAVLINK_COMM_1
#define SYSID_ONBOARD                       4

#define SEARCH_MS                           60000   // search for gimbal for 1 minute after startup
#define TELEMETRY_MAVLINK_MAXRATE           50
#define TELEMETRY_MAVLINK_DELAY            (1000 / TELEMETRY_MAVLINK_MAXRATE)*300 // unit ms


// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t);

typedef enum _gimbal_send_t {
    SEND_HEARTBEAT           = 1,
    SEND_STATUS              = 2,
    SEND_ATTITUDE            = 3,
    SEND_ENCODER_VAL         = 4,
    SEND_REQUEST_ATTITUDE    = 5,
    SEND_PARAM               = 6,
    SEND_RC_CHAN,
    SEND_UUID,
    SEND_DEBUG,
    SEND_RAW_IMU,
    SEND_REQ_DATA_STREAM_ATTI
} type_send_t;

/*
* Brief: Define expand command available mavlink
*/

typedef struct _mav_state
{
    mavlink_message_t   rxmsg;
    mavlink_status_t    status;
} mav_state_t;


typedef struct _protocol_t
{
    uint8_t   (*has_finished)(void);
    uint8_t   (*parse_msg)(mav_state_t *msg);
} protocol_t;
/**
 * @brief  This is function initialize COM platform, protocol
 * @param   in: none
 * @param   out: none
 * @return none
 */
void gProtocol_init(protocol_t *proto);
/*
 * @brief  This is function read data from gimbal
 * @param   in: none
 * @param   out: none
 * @return none
 */
uint8_t gProtocol_serialPort_COM4_read_data(mav_state_t *mav);
uint8_t gProtocol_read_data(mav_state_t *mav);
/**
 * @brief  this function will calculate the frequent to send data periodically 
 * @param   streamNum: Specific the message 
 * @param   chan:      Specific the channel want to send mav/gtune channel
 * @return return true/false
 */
int send_rate(type_send_t streamNum);
void gGimbal_Console(uint8_t *str);
/*********** Portions COPYRIGHT 2018 Gremsy.Co., Ltd.*****END OF FILE****/
