#pragma once
// MESSAGE UAV_FOUND PACKING

#define MAVLINK_MSG_ID_UAV_FOUND 12000


typedef struct __mavlink_uav_found_t {
 double lat; /*<  */
 double lon; /*<  */
 double alt; /*<  */
 uint16_t heading; /*< [cdeg] Course over ground*/
 uint16_t hor_velocity; /*< [cm/s] The horizontal velocity*/
 int16_t ver_velocity; /*< [cm/s] The vertical velocity. Positive is up*/
 uint8_t mac[6]; /*<  */
} mavlink_uav_found_t;

#define MAVLINK_MSG_ID_UAV_FOUND_LEN 36
#define MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN 36
#define MAVLINK_MSG_ID_12000_LEN 36
#define MAVLINK_MSG_ID_12000_MIN_LEN 36

#define MAVLINK_MSG_ID_UAV_FOUND_CRC 242
#define MAVLINK_MSG_ID_12000_CRC 242

#define MAVLINK_MSG_UAV_FOUND_FIELD_MAC_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_FOUND { \
    12000, \
    "UAV_FOUND", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_uav_found_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_uav_found_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_uav_found_t, alt) }, \
         { "mac", NULL, MAVLINK_TYPE_UINT8_T, 6, 30, offsetof(mavlink_uav_found_t, mac) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_uav_found_t, heading) }, \
         { "hor_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_uav_found_t, hor_velocity) }, \
         { "ver_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_uav_found_t, ver_velocity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_FOUND { \
    "UAV_FOUND", \
    7, \
    {  { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_uav_found_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_uav_found_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_uav_found_t, alt) }, \
         { "mac", NULL, MAVLINK_TYPE_UINT8_T, 6, 30, offsetof(mavlink_uav_found_t, mac) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_uav_found_t, heading) }, \
         { "hor_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_uav_found_t, hor_velocity) }, \
         { "ver_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_uav_found_t, ver_velocity) }, \
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
 * @param mac  
 * @param heading [cdeg] Course over ground
 * @param hor_velocity [cm/s] The horizontal velocity
 * @param ver_velocity [cm/s] The vertical velocity. Positive is up
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double lat, double lon, double alt, const uint8_t *mac, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);
    _mav_put_uint16_t(buf, 24, heading);
    _mav_put_uint16_t(buf, 26, hor_velocity);
    _mav_put_int16_t(buf, 28, ver_velocity);
    _mav_put_uint8_t_array(buf, 30, mac, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.heading = heading;
    packet.hor_velocity = hor_velocity;
    packet.ver_velocity = ver_velocity;
    mav_array_memcpy(packet.mac, mac, sizeof(uint8_t)*6);
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
 * @param mac  
 * @param heading [cdeg] Course over ground
 * @param hor_velocity [cm/s] The horizontal velocity
 * @param ver_velocity [cm/s] The vertical velocity. Positive is up
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               double lat, double lon, double alt, const uint8_t *mac, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);
    _mav_put_uint16_t(buf, 24, heading);
    _mav_put_uint16_t(buf, 26, hor_velocity);
    _mav_put_int16_t(buf, 28, ver_velocity);
    _mav_put_uint8_t_array(buf, 30, mac, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.heading = heading;
    packet.hor_velocity = hor_velocity;
    packet.ver_velocity = ver_velocity;
    mav_array_memcpy(packet.mac, mac, sizeof(uint8_t)*6);
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
 * @param mac  
 * @param heading [cdeg] Course over ground
 * @param hor_velocity [cm/s] The horizontal velocity
 * @param ver_velocity [cm/s] The vertical velocity. Positive is up
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_found_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double lat,double lon,double alt,const uint8_t *mac,uint16_t heading,uint16_t hor_velocity,int16_t ver_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);
    _mav_put_uint16_t(buf, 24, heading);
    _mav_put_uint16_t(buf, 26, hor_velocity);
    _mav_put_int16_t(buf, 28, ver_velocity);
    _mav_put_uint8_t_array(buf, 30, mac, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_FOUND_LEN);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.heading = heading;
    packet.hor_velocity = hor_velocity;
    packet.ver_velocity = ver_velocity;
    mav_array_memcpy(packet.mac, mac, sizeof(uint8_t)*6);
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
    return mavlink_msg_uav_found_pack(system_id, component_id, msg, uav_found->lat, uav_found->lon, uav_found->alt, uav_found->mac, uav_found->heading, uav_found->hor_velocity, uav_found->ver_velocity);
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
    return mavlink_msg_uav_found_pack_chan(system_id, component_id, chan, msg, uav_found->lat, uav_found->lon, uav_found->alt, uav_found->mac, uav_found->heading, uav_found->hor_velocity, uav_found->ver_velocity);
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
    return mavlink_msg_uav_found_pack_status(system_id, component_id, _status, msg,  uav_found->lat, uav_found->lon, uav_found->alt, uav_found->mac, uav_found->heading, uav_found->hor_velocity, uav_found->ver_velocity);
}

/**
 * @brief Send a uav_found message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  
 * @param lon  
 * @param alt  
 * @param mac  
 * @param heading [cdeg] Course over ground
 * @param hor_velocity [cm/s] The horizontal velocity
 * @param ver_velocity [cm/s] The vertical velocity. Positive is up
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_found_send(mavlink_channel_t chan, double lat, double lon, double alt, const uint8_t *mac, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_FOUND_LEN];
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);
    _mav_put_uint16_t(buf, 24, heading);
    _mav_put_uint16_t(buf, 26, hor_velocity);
    _mav_put_int16_t(buf, 28, ver_velocity);
    _mav_put_uint8_t_array(buf, 30, mac, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, buf, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#else
    mavlink_uav_found_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.heading = heading;
    packet.hor_velocity = hor_velocity;
    packet.ver_velocity = ver_velocity;
    mav_array_memcpy(packet.mac, mac, sizeof(uint8_t)*6);
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
    mavlink_msg_uav_found_send(chan, uav_found->lat, uav_found->lon, uav_found->alt, uav_found->mac, uav_found->heading, uav_found->hor_velocity, uav_found->ver_velocity);
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
static inline void mavlink_msg_uav_found_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double lat, double lon, double alt, const uint8_t *mac, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, lat);
    _mav_put_double(buf, 8, lon);
    _mav_put_double(buf, 16, alt);
    _mav_put_uint16_t(buf, 24, heading);
    _mav_put_uint16_t(buf, 26, hor_velocity);
    _mav_put_int16_t(buf, 28, ver_velocity);
    _mav_put_uint8_t_array(buf, 30, mac, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_FOUND, buf, MAVLINK_MSG_ID_UAV_FOUND_MIN_LEN, MAVLINK_MSG_ID_UAV_FOUND_LEN, MAVLINK_MSG_ID_UAV_FOUND_CRC);
#else
    mavlink_uav_found_t *packet = (mavlink_uav_found_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->heading = heading;
    packet->hor_velocity = hor_velocity;
    packet->ver_velocity = ver_velocity;
    mav_array_memcpy(packet->mac, mac, sizeof(uint8_t)*6);
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
 * @brief Get field mac from uav_found message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_uav_found_get_mac(const mavlink_message_t* msg, uint8_t *mac)
{
    return _MAV_RETURN_uint8_t_array(msg, mac, 6,  30);
}

/**
 * @brief Get field heading from uav_found message
 *
 * @return [cdeg] Course over ground
 */
static inline uint16_t mavlink_msg_uav_found_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field hor_velocity from uav_found message
 *
 * @return [cm/s] The horizontal velocity
 */
static inline uint16_t mavlink_msg_uav_found_get_hor_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field ver_velocity from uav_found message
 *
 * @return [cm/s] The vertical velocity. Positive is up
 */
static inline int16_t mavlink_msg_uav_found_get_ver_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
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
    uav_found->heading = mavlink_msg_uav_found_get_heading(msg);
    uav_found->hor_velocity = mavlink_msg_uav_found_get_hor_velocity(msg);
    uav_found->ver_velocity = mavlink_msg_uav_found_get_ver_velocity(msg);
    mavlink_msg_uav_found_get_mac(msg, uav_found->mac);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_FOUND_LEN? msg->len : MAVLINK_MSG_ID_UAV_FOUND_LEN;
        memset(uav_found, 0, MAVLINK_MSG_ID_UAV_FOUND_LEN);
    memcpy(uav_found, _MAV_PAYLOAD(msg), len);
#endif
}
