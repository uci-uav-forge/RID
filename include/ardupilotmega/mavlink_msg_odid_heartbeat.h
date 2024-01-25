#pragma once
// MESSAGE ODID_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_ODID_HEARTBEAT 12001


typedef struct __mavlink_odid_heartbeat_t {
 uint8_t count; /*<  */
} mavlink_odid_heartbeat_t;

#define MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN 1
#define MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN 1
#define MAVLINK_MSG_ID_12001_LEN 1
#define MAVLINK_MSG_ID_12001_MIN_LEN 1

#define MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC 11
#define MAVLINK_MSG_ID_12001_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ODID_HEARTBEAT { \
    12001, \
    "ODID_HEARTBEAT", \
    1, \
    {  { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_odid_heartbeat_t, count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ODID_HEARTBEAT { \
    "ODID_HEARTBEAT", \
    1, \
    {  { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_odid_heartbeat_t, count) }, \
         } \
}
#endif

/**
 * @brief Pack a odid_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param count  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odid_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#else
    mavlink_odid_heartbeat_t packet;
    packet.count = count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
}

/**
 * @brief Pack a odid_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param count  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odid_heartbeat_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#else
    mavlink_odid_heartbeat_t packet;
    packet.count = count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODID_HEARTBEAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#endif
}

/**
 * @brief Pack a odid_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param count  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_odid_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#else
    mavlink_odid_heartbeat_t packet;
    packet.count = count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ODID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
}

/**
 * @brief Encode a odid_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param odid_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odid_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_odid_heartbeat_t* odid_heartbeat)
{
    return mavlink_msg_odid_heartbeat_pack(system_id, component_id, msg, odid_heartbeat->count);
}

/**
 * @brief Encode a odid_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param odid_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odid_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_odid_heartbeat_t* odid_heartbeat)
{
    return mavlink_msg_odid_heartbeat_pack_chan(system_id, component_id, chan, msg, odid_heartbeat->count);
}

/**
 * @brief Encode a odid_heartbeat struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param odid_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_odid_heartbeat_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_odid_heartbeat_t* odid_heartbeat)
{
    return mavlink_msg_odid_heartbeat_pack_status(system_id, component_id, _status, msg,  odid_heartbeat->count);
}

/**
 * @brief Send a odid_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param count  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_odid_heartbeat_send(mavlink_channel_t chan, uint8_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN];
    _mav_put_uint8_t(buf, 0, count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODID_HEARTBEAT, buf, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#else
    mavlink_odid_heartbeat_t packet;
    packet.count = count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a odid_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_odid_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_odid_heartbeat_t* odid_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_odid_heartbeat_send(chan, odid_heartbeat->count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODID_HEARTBEAT, (const char *)odid_heartbeat, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_odid_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODID_HEARTBEAT, buf, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#else
    mavlink_odid_heartbeat_t *packet = (mavlink_odid_heartbeat_t *)msgbuf;
    packet->count = count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ODID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_ODID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN, MAVLINK_MSG_ID_ODID_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE ODID_HEARTBEAT UNPACKING


/**
 * @brief Get field count from odid_heartbeat message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_odid_heartbeat_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a odid_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param odid_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_odid_heartbeat_decode(const mavlink_message_t* msg, mavlink_odid_heartbeat_t* odid_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    odid_heartbeat->count = mavlink_msg_odid_heartbeat_get_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN;
        memset(odid_heartbeat, 0, MAVLINK_MSG_ID_ODID_HEARTBEAT_LEN);
    memcpy(odid_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
