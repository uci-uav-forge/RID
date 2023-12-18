#pragma once
// MESSAGE UAV_FOUND PACKING

#define MAVLINK_MSG_ID_UAV_FOUND 12000


typedef struct __mavlink_uav_found_t {
 double lat; /*<  */
 double lon; /*<  */
 double alt; /*<  */
} mavlink_uav_found_t;

#define MAVLINK_MSG_ID_UAV_FOUND_LEN 24
#define MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN 24
#define MAVLINK_MSG_ID_12000_LEN 24
#define MAVLINK_MSG_ID_12000_MIN_LEN 24

#define MAVLINK_MSG_ID_UAV_FOUND_CRC 200
#define MAVLINK_MSG_ID_12000_CRC 200



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_FOUND { \
    12000, \
    "UAV_FOUND", \
    3, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_uav_found_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_uav_found_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_uav_found_t, alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_FOUND { \
    "UAV_FOUND", \
    3, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_uav_found_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_uav_found_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_uav_found_t, alt) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_found message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  
 * @param lon  
 * @param alt  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double lat, double lon, double alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_FOUND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
}

/**
 * @brief Pack a uav_found message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  
 * @param lon  
 * @param alt  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               double lat, double lon, double alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_FOUND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#endif
}

/**
 * @brief Pack a uav_found message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  
 * @param lon  
 * @param alt  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double lat,double lon,double alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_FOUND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
}

/**
 * @brief Encode a uav_found struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_found C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_found_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_found_t* uav_found)
{
    return mavlink_msg_uav_found_pack(system_id, component_id, msg, uav_found->lat, uav_found->lon, uav_found->alt);
}

/**
 * @brief Encode a uav_found struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_found C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_found_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_found_t* uav_found)
{
    return mavlink_msg_uav_found_pack_chan(system_id, component_id, chan, msg, uav_found->lat, uav_found->lon, uav_found->alt);
}

/**
 * @brief Encode a uav_found struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uav_found C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_found_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uav_found_t* uav_found)
{
    return mavlink_msg_uav_found_pack_status(system_id, component_id, _status, msg,  uav_found->lat, uav_found->lon, uav_found->alt);
}

/**
 * @brief Send a uav_found message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  
 * @param lon  
 * @param alt  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_found_send(mavlink_channel_t chan, double lat, double lon, double alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, buf, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, (const char *)&packet, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#endif
}

/**
 * @brief Send a uav_found message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_found_send_struct(mavlink_channel_t chan, const mavlink_uav_found_t* uav_found)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_found_send(chan, uav_found->lat, uav_found->lon, uav_found->alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, (const char *)uav_found, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_FOUND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_found_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double lat, double lon, double alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, buf, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#else
    mavlink_uav_found_t *packet = (mavlink_uav_found_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, (const char *)packet, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_FOUND UNPACKING


/**
 * @brief Get field lat from uav_found message
 *
 * @return  
 */
static inline double mavlink_msg_uav_found_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field lon from uav_found message
 *
 * @return  
 */
static inline double mavlink_msg_uav_found_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field alt from uav_found message
 *
 * @return  
 */
static inline double mavlink_msg_uav_found_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Decode a uav_found message into a struct
 *
 * @param msg The message to decode
 * @param uav_found C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_found_decode(const mavlink_message_t* msg, mavlink_uav_found_t* uav_found)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_found->lat = mavlink_msg_uav_found_get_lat(msg);
    uav_found->lon = mavlink_msg_uav_found_get_lon(msg);
    uav_found->alt = mavlink_msg_uav_found_get_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_FOUND_LEN? msg->len : MAVLINK_MSG_ID_UAV_FOUND_LEN;
        memset(uav_found, 0, MAVLINK_MSG_ID_UAV_FOUND_LEN);
    memcpy(uav_found, _MAV_PAYLOAD(msg), len);
#endif
}
