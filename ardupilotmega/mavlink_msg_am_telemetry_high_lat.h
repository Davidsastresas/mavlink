#pragma once
// MESSAGE AM_TELEMETRY_HIGH_LAT PACKING

#define MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT 187

MAVPACKED(
typedef struct __mavlink_am_telemetry_high_lat_t {
 uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags.*/
 int32_t latitude; /*< [degE7] Latitude*/
 int32_t longitude; /*< [degE7] Longitude*/
 int16_t roll; /*< [cdeg] roll*/
 int16_t pitch; /*< [cdeg] pitch*/
 uint16_t heading; /*< [cdeg] heading*/
 int16_t heading_sp; /*< [cdeg] heading setpoint*/
 int16_t altitude_amsl; /*< [m] Altitude above mean sea level*/
 int16_t altitude_sp; /*< [m] Altitude setpoint relative to the home position*/
 uint16_t wp_distance; /*< [m] distance to target*/
 uint8_t base_mode; /*<  Bitmap of enabled system modes.*/
 uint8_t landed_state; /*<  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.*/
 int8_t throttle; /*< [%] throttle (percentage)*/
 uint8_t airspeed; /*< [m/s] airspeed*/
 uint8_t airspeed_sp; /*< [m/s] airspeed setpoint*/
 uint8_t groundspeed; /*< [m/s] groundspeed*/
 int8_t climb_rate; /*< [m/s] climb rate*/
 uint8_t gps_nsat; /*<  Number of satellites visible. If unknown, set to 255*/
 uint8_t gps_fix_type; /*<  GPS Fix type.*/
 uint8_t battery_remaining; /*< [%] Remaining battery (percentage)*/
 int8_t temperature; /*< [degC] Autopilot temperature (degrees C)*/
 int8_t temperature_air; /*< [degC] Air temperature (degrees C) from airspeed sensor*/
 uint8_t failsafe; /*<  failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)*/
 uint8_t wp_num; /*<  current waypoint number*/
 uint8_t rssi_gsm1; /*<  rssi gsm modem 1*/
 uint8_t rssi_gsm2; /*<  rssi gsm modem 2*/
 uint8_t rssi_gsm3; /*<  rssi gsm modem 3*/
 uint8_t rssi_sbd; /*<  rssi sbd modem*/
 uint8_t comp_status_bitmask; /*<  companion status bitmask*/
 uint8_t telem_status_bitmask; /*<  telemetry status bitmask*/
}) mavlink_am_telemetry_high_lat_t;

#define MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN 46
#define MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN 46
#define MAVLINK_MSG_ID_187_LEN 46
#define MAVLINK_MSG_ID_187_MIN_LEN 46

#define MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC 231
#define MAVLINK_MSG_ID_187_CRC 231



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AM_TELEMETRY_HIGH_LAT { \
    187, \
    "AM_TELEMETRY_HIGH_LAT", \
    30, \
    {  { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_am_telemetry_high_lat_t, base_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_am_telemetry_high_lat_t, custom_mode) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_am_telemetry_high_lat_t, landed_state) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_am_telemetry_high_lat_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_am_telemetry_high_lat_t, pitch) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_am_telemetry_high_lat_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_am_telemetry_high_lat_t, throttle) }, \
         { "heading_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_am_telemetry_high_lat_t, heading_sp) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_am_telemetry_high_lat_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_am_telemetry_high_lat_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_am_telemetry_high_lat_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_am_telemetry_high_lat_t, altitude_sp) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_am_telemetry_high_lat_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_am_telemetry_high_lat_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_am_telemetry_high_lat_t, groundspeed) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_am_telemetry_high_lat_t, climb_rate) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_am_telemetry_high_lat_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_am_telemetry_high_lat_t, gps_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_am_telemetry_high_lat_t, battery_remaining) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_am_telemetry_high_lat_t, temperature) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_am_telemetry_high_lat_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_am_telemetry_high_lat_t, failsafe) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_am_telemetry_high_lat_t, wp_num) }, \
         { "wp_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_am_telemetry_high_lat_t, wp_distance) }, \
         { "rssi_gsm1", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm1) }, \
         { "rssi_gsm2", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm2) }, \
         { "rssi_gsm3", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm3) }, \
         { "rssi_sbd", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_am_telemetry_high_lat_t, rssi_sbd) }, \
         { "comp_status_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_am_telemetry_high_lat_t, comp_status_bitmask) }, \
         { "telem_status_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_am_telemetry_high_lat_t, telem_status_bitmask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AM_TELEMETRY_HIGH_LAT { \
    "AM_TELEMETRY_HIGH_LAT", \
    30, \
    {  { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_am_telemetry_high_lat_t, base_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_am_telemetry_high_lat_t, custom_mode) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_am_telemetry_high_lat_t, landed_state) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_am_telemetry_high_lat_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_am_telemetry_high_lat_t, pitch) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_am_telemetry_high_lat_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_am_telemetry_high_lat_t, throttle) }, \
         { "heading_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_am_telemetry_high_lat_t, heading_sp) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_am_telemetry_high_lat_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_am_telemetry_high_lat_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_am_telemetry_high_lat_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_am_telemetry_high_lat_t, altitude_sp) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_am_telemetry_high_lat_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_am_telemetry_high_lat_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_am_telemetry_high_lat_t, groundspeed) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_am_telemetry_high_lat_t, climb_rate) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_am_telemetry_high_lat_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_am_telemetry_high_lat_t, gps_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_am_telemetry_high_lat_t, battery_remaining) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_am_telemetry_high_lat_t, temperature) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_am_telemetry_high_lat_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_am_telemetry_high_lat_t, failsafe) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_am_telemetry_high_lat_t, wp_num) }, \
         { "wp_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_am_telemetry_high_lat_t, wp_distance) }, \
         { "rssi_gsm1", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm1) }, \
         { "rssi_gsm2", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm2) }, \
         { "rssi_gsm3", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_am_telemetry_high_lat_t, rssi_gsm3) }, \
         { "rssi_sbd", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_am_telemetry_high_lat_t, rssi_sbd) }, \
         { "comp_status_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_am_telemetry_high_lat_t, comp_status_bitmask) }, \
         { "telem_status_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_am_telemetry_high_lat_t, telem_status_bitmask) }, \
         } \
}
#endif

/**
 * @brief Pack a am_telemetry_high_lat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param base_mode  Bitmap of enabled system modes.
 * @param custom_mode  A bitfield for use for autopilot-specific flags.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll [cdeg] roll
 * @param pitch [cdeg] pitch
 * @param heading [cdeg] heading
 * @param throttle [%] throttle (percentage)
 * @param heading_sp [cdeg] heading setpoint
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude_amsl [m] Altitude above mean sea level
 * @param altitude_sp [m] Altitude setpoint relative to the home position
 * @param airspeed [m/s] airspeed
 * @param airspeed_sp [m/s] airspeed setpoint
 * @param groundspeed [m/s] groundspeed
 * @param climb_rate [m/s] climb rate
 * @param gps_nsat  Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type  GPS Fix type.
 * @param battery_remaining [%] Remaining battery (percentage)
 * @param temperature [degC] Autopilot temperature (degrees C)
 * @param temperature_air [degC] Air temperature (degrees C) from airspeed sensor
 * @param failsafe  failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num  current waypoint number
 * @param wp_distance [m] distance to target
 * @param rssi_gsm1  rssi gsm modem 1
 * @param rssi_gsm2  rssi gsm modem 2
 * @param rssi_gsm3  rssi gsm modem 3
 * @param rssi_sbd  rssi sbd modem
 * @param comp_status_bitmask  companion status bitmask
 * @param telem_status_bitmask  telemetry status bitmask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance, uint8_t rssi_gsm1, uint8_t rssi_gsm2, uint8_t rssi_gsm3, uint8_t rssi_sbd, uint8_t comp_status_bitmask, uint8_t telem_status_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);
    _mav_put_uint8_t(buf, 40, rssi_gsm1);
    _mav_put_uint8_t(buf, 41, rssi_gsm2);
    _mav_put_uint8_t(buf, 42, rssi_gsm3);
    _mav_put_uint8_t(buf, 43, rssi_sbd);
    _mav_put_uint8_t(buf, 44, comp_status_bitmask);
    _mav_put_uint8_t(buf, 45, telem_status_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN);
#else
    mavlink_am_telemetry_high_lat_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;
    packet.rssi_gsm1 = rssi_gsm1;
    packet.rssi_gsm2 = rssi_gsm2;
    packet.rssi_gsm3 = rssi_gsm3;
    packet.rssi_sbd = rssi_sbd;
    packet.comp_status_bitmask = comp_status_bitmask;
    packet.telem_status_bitmask = telem_status_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
}

/**
 * @brief Pack a am_telemetry_high_lat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param base_mode  Bitmap of enabled system modes.
 * @param custom_mode  A bitfield for use for autopilot-specific flags.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll [cdeg] roll
 * @param pitch [cdeg] pitch
 * @param heading [cdeg] heading
 * @param throttle [%] throttle (percentage)
 * @param heading_sp [cdeg] heading setpoint
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude_amsl [m] Altitude above mean sea level
 * @param altitude_sp [m] Altitude setpoint relative to the home position
 * @param airspeed [m/s] airspeed
 * @param airspeed_sp [m/s] airspeed setpoint
 * @param groundspeed [m/s] groundspeed
 * @param climb_rate [m/s] climb rate
 * @param gps_nsat  Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type  GPS Fix type.
 * @param battery_remaining [%] Remaining battery (percentage)
 * @param temperature [degC] Autopilot temperature (degrees C)
 * @param temperature_air [degC] Air temperature (degrees C) from airspeed sensor
 * @param failsafe  failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num  current waypoint number
 * @param wp_distance [m] distance to target
 * @param rssi_gsm1  rssi gsm modem 1
 * @param rssi_gsm2  rssi gsm modem 2
 * @param rssi_gsm3  rssi gsm modem 3
 * @param rssi_sbd  rssi sbd modem
 * @param comp_status_bitmask  companion status bitmask
 * @param telem_status_bitmask  telemetry status bitmask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t base_mode,uint32_t custom_mode,uint8_t landed_state,int16_t roll,int16_t pitch,uint16_t heading,int8_t throttle,int16_t heading_sp,int32_t latitude,int32_t longitude,int16_t altitude_amsl,int16_t altitude_sp,uint8_t airspeed,uint8_t airspeed_sp,uint8_t groundspeed,int8_t climb_rate,uint8_t gps_nsat,uint8_t gps_fix_type,uint8_t battery_remaining,int8_t temperature,int8_t temperature_air,uint8_t failsafe,uint8_t wp_num,uint16_t wp_distance,uint8_t rssi_gsm1,uint8_t rssi_gsm2,uint8_t rssi_gsm3,uint8_t rssi_sbd,uint8_t comp_status_bitmask,uint8_t telem_status_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);
    _mav_put_uint8_t(buf, 40, rssi_gsm1);
    _mav_put_uint8_t(buf, 41, rssi_gsm2);
    _mav_put_uint8_t(buf, 42, rssi_gsm3);
    _mav_put_uint8_t(buf, 43, rssi_sbd);
    _mav_put_uint8_t(buf, 44, comp_status_bitmask);
    _mav_put_uint8_t(buf, 45, telem_status_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN);
#else
    mavlink_am_telemetry_high_lat_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;
    packet.rssi_gsm1 = rssi_gsm1;
    packet.rssi_gsm2 = rssi_gsm2;
    packet.rssi_gsm3 = rssi_gsm3;
    packet.rssi_sbd = rssi_sbd;
    packet.comp_status_bitmask = comp_status_bitmask;
    packet.telem_status_bitmask = telem_status_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
}

/**
 * @brief Encode a am_telemetry_high_lat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param am_telemetry_high_lat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_am_telemetry_high_lat_t* am_telemetry_high_lat)
{
    return mavlink_msg_am_telemetry_high_lat_pack(system_id, component_id, msg, am_telemetry_high_lat->base_mode, am_telemetry_high_lat->custom_mode, am_telemetry_high_lat->landed_state, am_telemetry_high_lat->roll, am_telemetry_high_lat->pitch, am_telemetry_high_lat->heading, am_telemetry_high_lat->throttle, am_telemetry_high_lat->heading_sp, am_telemetry_high_lat->latitude, am_telemetry_high_lat->longitude, am_telemetry_high_lat->altitude_amsl, am_telemetry_high_lat->altitude_sp, am_telemetry_high_lat->airspeed, am_telemetry_high_lat->airspeed_sp, am_telemetry_high_lat->groundspeed, am_telemetry_high_lat->climb_rate, am_telemetry_high_lat->gps_nsat, am_telemetry_high_lat->gps_fix_type, am_telemetry_high_lat->battery_remaining, am_telemetry_high_lat->temperature, am_telemetry_high_lat->temperature_air, am_telemetry_high_lat->failsafe, am_telemetry_high_lat->wp_num, am_telemetry_high_lat->wp_distance, am_telemetry_high_lat->rssi_gsm1, am_telemetry_high_lat->rssi_gsm2, am_telemetry_high_lat->rssi_gsm3, am_telemetry_high_lat->rssi_sbd, am_telemetry_high_lat->comp_status_bitmask, am_telemetry_high_lat->telem_status_bitmask);
}

/**
 * @brief Encode a am_telemetry_high_lat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param am_telemetry_high_lat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_am_telemetry_high_lat_t* am_telemetry_high_lat)
{
    return mavlink_msg_am_telemetry_high_lat_pack_chan(system_id, component_id, chan, msg, am_telemetry_high_lat->base_mode, am_telemetry_high_lat->custom_mode, am_telemetry_high_lat->landed_state, am_telemetry_high_lat->roll, am_telemetry_high_lat->pitch, am_telemetry_high_lat->heading, am_telemetry_high_lat->throttle, am_telemetry_high_lat->heading_sp, am_telemetry_high_lat->latitude, am_telemetry_high_lat->longitude, am_telemetry_high_lat->altitude_amsl, am_telemetry_high_lat->altitude_sp, am_telemetry_high_lat->airspeed, am_telemetry_high_lat->airspeed_sp, am_telemetry_high_lat->groundspeed, am_telemetry_high_lat->climb_rate, am_telemetry_high_lat->gps_nsat, am_telemetry_high_lat->gps_fix_type, am_telemetry_high_lat->battery_remaining, am_telemetry_high_lat->temperature, am_telemetry_high_lat->temperature_air, am_telemetry_high_lat->failsafe, am_telemetry_high_lat->wp_num, am_telemetry_high_lat->wp_distance, am_telemetry_high_lat->rssi_gsm1, am_telemetry_high_lat->rssi_gsm2, am_telemetry_high_lat->rssi_gsm3, am_telemetry_high_lat->rssi_sbd, am_telemetry_high_lat->comp_status_bitmask, am_telemetry_high_lat->telem_status_bitmask);
}

/**
 * @brief Send a am_telemetry_high_lat message
 * @param chan MAVLink channel to send the message
 *
 * @param base_mode  Bitmap of enabled system modes.
 * @param custom_mode  A bitfield for use for autopilot-specific flags.
 * @param landed_state  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll [cdeg] roll
 * @param pitch [cdeg] pitch
 * @param heading [cdeg] heading
 * @param throttle [%] throttle (percentage)
 * @param heading_sp [cdeg] heading setpoint
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude_amsl [m] Altitude above mean sea level
 * @param altitude_sp [m] Altitude setpoint relative to the home position
 * @param airspeed [m/s] airspeed
 * @param airspeed_sp [m/s] airspeed setpoint
 * @param groundspeed [m/s] groundspeed
 * @param climb_rate [m/s] climb rate
 * @param gps_nsat  Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type  GPS Fix type.
 * @param battery_remaining [%] Remaining battery (percentage)
 * @param temperature [degC] Autopilot temperature (degrees C)
 * @param temperature_air [degC] Air temperature (degrees C) from airspeed sensor
 * @param failsafe  failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num  current waypoint number
 * @param wp_distance [m] distance to target
 * @param rssi_gsm1  rssi gsm modem 1
 * @param rssi_gsm2  rssi gsm modem 2
 * @param rssi_gsm3  rssi gsm modem 3
 * @param rssi_sbd  rssi sbd modem
 * @param comp_status_bitmask  companion status bitmask
 * @param telem_status_bitmask  telemetry status bitmask
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_am_telemetry_high_lat_send(mavlink_channel_t chan, uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance, uint8_t rssi_gsm1, uint8_t rssi_gsm2, uint8_t rssi_gsm3, uint8_t rssi_sbd, uint8_t comp_status_bitmask, uint8_t telem_status_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);
    _mav_put_uint8_t(buf, 40, rssi_gsm1);
    _mav_put_uint8_t(buf, 41, rssi_gsm2);
    _mav_put_uint8_t(buf, 42, rssi_gsm3);
    _mav_put_uint8_t(buf, 43, rssi_sbd);
    _mav_put_uint8_t(buf, 44, comp_status_bitmask);
    _mav_put_uint8_t(buf, 45, telem_status_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT, buf, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
#else
    mavlink_am_telemetry_high_lat_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;
    packet.rssi_gsm1 = rssi_gsm1;
    packet.rssi_gsm2 = rssi_gsm2;
    packet.rssi_gsm3 = rssi_gsm3;
    packet.rssi_sbd = rssi_sbd;
    packet.comp_status_bitmask = comp_status_bitmask;
    packet.telem_status_bitmask = telem_status_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT, (const char *)&packet, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
#endif
}

/**
 * @brief Send a am_telemetry_high_lat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_am_telemetry_high_lat_send_struct(mavlink_channel_t chan, const mavlink_am_telemetry_high_lat_t* am_telemetry_high_lat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_am_telemetry_high_lat_send(chan, am_telemetry_high_lat->base_mode, am_telemetry_high_lat->custom_mode, am_telemetry_high_lat->landed_state, am_telemetry_high_lat->roll, am_telemetry_high_lat->pitch, am_telemetry_high_lat->heading, am_telemetry_high_lat->throttle, am_telemetry_high_lat->heading_sp, am_telemetry_high_lat->latitude, am_telemetry_high_lat->longitude, am_telemetry_high_lat->altitude_amsl, am_telemetry_high_lat->altitude_sp, am_telemetry_high_lat->airspeed, am_telemetry_high_lat->airspeed_sp, am_telemetry_high_lat->groundspeed, am_telemetry_high_lat->climb_rate, am_telemetry_high_lat->gps_nsat, am_telemetry_high_lat->gps_fix_type, am_telemetry_high_lat->battery_remaining, am_telemetry_high_lat->temperature, am_telemetry_high_lat->temperature_air, am_telemetry_high_lat->failsafe, am_telemetry_high_lat->wp_num, am_telemetry_high_lat->wp_distance, am_telemetry_high_lat->rssi_gsm1, am_telemetry_high_lat->rssi_gsm2, am_telemetry_high_lat->rssi_gsm3, am_telemetry_high_lat->rssi_sbd, am_telemetry_high_lat->comp_status_bitmask, am_telemetry_high_lat->telem_status_bitmask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT, (const char *)am_telemetry_high_lat, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_am_telemetry_high_lat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance, uint8_t rssi_gsm1, uint8_t rssi_gsm2, uint8_t rssi_gsm3, uint8_t rssi_sbd, uint8_t comp_status_bitmask, uint8_t telem_status_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);
    _mav_put_uint8_t(buf, 40, rssi_gsm1);
    _mav_put_uint8_t(buf, 41, rssi_gsm2);
    _mav_put_uint8_t(buf, 42, rssi_gsm3);
    _mav_put_uint8_t(buf, 43, rssi_sbd);
    _mav_put_uint8_t(buf, 44, comp_status_bitmask);
    _mav_put_uint8_t(buf, 45, telem_status_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT, buf, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
#else
    mavlink_am_telemetry_high_lat_t *packet = (mavlink_am_telemetry_high_lat_t *)msgbuf;
    packet->custom_mode = custom_mode;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->heading = heading;
    packet->heading_sp = heading_sp;
    packet->altitude_amsl = altitude_amsl;
    packet->altitude_sp = altitude_sp;
    packet->wp_distance = wp_distance;
    packet->base_mode = base_mode;
    packet->landed_state = landed_state;
    packet->throttle = throttle;
    packet->airspeed = airspeed;
    packet->airspeed_sp = airspeed_sp;
    packet->groundspeed = groundspeed;
    packet->climb_rate = climb_rate;
    packet->gps_nsat = gps_nsat;
    packet->gps_fix_type = gps_fix_type;
    packet->battery_remaining = battery_remaining;
    packet->temperature = temperature;
    packet->temperature_air = temperature_air;
    packet->failsafe = failsafe;
    packet->wp_num = wp_num;
    packet->rssi_gsm1 = rssi_gsm1;
    packet->rssi_gsm2 = rssi_gsm2;
    packet->rssi_gsm3 = rssi_gsm3;
    packet->rssi_sbd = rssi_sbd;
    packet->comp_status_bitmask = comp_status_bitmask;
    packet->telem_status_bitmask = telem_status_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT, (const char *)packet, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_MIN_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_CRC);
#endif
}
#endif

#endif

// MESSAGE AM_TELEMETRY_HIGH_LAT UNPACKING


/**
 * @brief Get field base_mode from am_telemetry_high_lat message
 *
 * @return  Bitmap of enabled system modes.
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_base_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field custom_mode from am_telemetry_high_lat message
 *
 * @return  A bitfield for use for autopilot-specific flags.
 */
static inline uint32_t mavlink_msg_am_telemetry_high_lat_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field landed_state from am_telemetry_high_lat message
 *
 * @return  The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_landed_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field roll from am_telemetry_high_lat message
 *
 * @return [cdeg] roll
 */
static inline int16_t mavlink_msg_am_telemetry_high_lat_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field pitch from am_telemetry_high_lat message
 *
 * @return [cdeg] pitch
 */
static inline int16_t mavlink_msg_am_telemetry_high_lat_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field heading from am_telemetry_high_lat message
 *
 * @return [cdeg] heading
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field throttle from am_telemetry_high_lat message
 *
 * @return [%] throttle (percentage)
 */
static inline int8_t mavlink_msg_am_telemetry_high_lat_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  28);
}

/**
 * @brief Get field heading_sp from am_telemetry_high_lat message
 *
 * @return [cdeg] heading setpoint
 */
static inline int16_t mavlink_msg_am_telemetry_high_lat_get_heading_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field latitude from am_telemetry_high_lat message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_am_telemetry_high_lat_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from am_telemetry_high_lat message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_am_telemetry_high_lat_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude_amsl from am_telemetry_high_lat message
 *
 * @return [m] Altitude above mean sea level
 */
static inline int16_t mavlink_msg_am_telemetry_high_lat_get_altitude_amsl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field altitude_sp from am_telemetry_high_lat message
 *
 * @return [m] Altitude setpoint relative to the home position
 */
static inline int16_t mavlink_msg_am_telemetry_high_lat_get_altitude_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field airspeed from am_telemetry_high_lat message
 *
 * @return [m/s] airspeed
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field airspeed_sp from am_telemetry_high_lat message
 *
 * @return [m/s] airspeed setpoint
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_airspeed_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field groundspeed from am_telemetry_high_lat message
 *
 * @return [m/s] groundspeed
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field climb_rate from am_telemetry_high_lat message
 *
 * @return [m/s] climb rate
 */
static inline int8_t mavlink_msg_am_telemetry_high_lat_get_climb_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  32);
}

/**
 * @brief Get field gps_nsat from am_telemetry_high_lat message
 *
 * @return  Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_gps_nsat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field gps_fix_type from am_telemetry_high_lat message
 *
 * @return  GPS Fix type.
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_gps_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field battery_remaining from am_telemetry_high_lat message
 *
 * @return [%] Remaining battery (percentage)
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field temperature from am_telemetry_high_lat message
 *
 * @return [degC] Autopilot temperature (degrees C)
 */
static inline int8_t mavlink_msg_am_telemetry_high_lat_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field temperature_air from am_telemetry_high_lat message
 *
 * @return [degC] Air temperature (degrees C) from airspeed sensor
 */
static inline int8_t mavlink_msg_am_telemetry_high_lat_get_temperature_air(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  37);
}

/**
 * @brief Get field failsafe from am_telemetry_high_lat message
 *
 * @return  failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_failsafe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field wp_num from am_telemetry_high_lat message
 *
 * @return  current waypoint number
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_wp_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field wp_distance from am_telemetry_high_lat message
 *
 * @return [m] distance to target
 */
static inline uint16_t mavlink_msg_am_telemetry_high_lat_get_wp_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field rssi_gsm1 from am_telemetry_high_lat message
 *
 * @return  rssi gsm modem 1
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_rssi_gsm1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field rssi_gsm2 from am_telemetry_high_lat message
 *
 * @return  rssi gsm modem 2
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_rssi_gsm2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field rssi_gsm3 from am_telemetry_high_lat message
 *
 * @return  rssi gsm modem 3
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_rssi_gsm3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field rssi_sbd from am_telemetry_high_lat message
 *
 * @return  rssi sbd modem
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_rssi_sbd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field comp_status_bitmask from am_telemetry_high_lat message
 *
 * @return  companion status bitmask
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_comp_status_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field telem_status_bitmask from am_telemetry_high_lat message
 *
 * @return  telemetry status bitmask
 */
static inline uint8_t mavlink_msg_am_telemetry_high_lat_get_telem_status_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Decode a am_telemetry_high_lat message into a struct
 *
 * @param msg The message to decode
 * @param am_telemetry_high_lat C-struct to decode the message contents into
 */
static inline void mavlink_msg_am_telemetry_high_lat_decode(const mavlink_message_t* msg, mavlink_am_telemetry_high_lat_t* am_telemetry_high_lat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    am_telemetry_high_lat->custom_mode = mavlink_msg_am_telemetry_high_lat_get_custom_mode(msg);
    am_telemetry_high_lat->latitude = mavlink_msg_am_telemetry_high_lat_get_latitude(msg);
    am_telemetry_high_lat->longitude = mavlink_msg_am_telemetry_high_lat_get_longitude(msg);
    am_telemetry_high_lat->roll = mavlink_msg_am_telemetry_high_lat_get_roll(msg);
    am_telemetry_high_lat->pitch = mavlink_msg_am_telemetry_high_lat_get_pitch(msg);
    am_telemetry_high_lat->heading = mavlink_msg_am_telemetry_high_lat_get_heading(msg);
    am_telemetry_high_lat->heading_sp = mavlink_msg_am_telemetry_high_lat_get_heading_sp(msg);
    am_telemetry_high_lat->altitude_amsl = mavlink_msg_am_telemetry_high_lat_get_altitude_amsl(msg);
    am_telemetry_high_lat->altitude_sp = mavlink_msg_am_telemetry_high_lat_get_altitude_sp(msg);
    am_telemetry_high_lat->wp_distance = mavlink_msg_am_telemetry_high_lat_get_wp_distance(msg);
    am_telemetry_high_lat->base_mode = mavlink_msg_am_telemetry_high_lat_get_base_mode(msg);
    am_telemetry_high_lat->landed_state = mavlink_msg_am_telemetry_high_lat_get_landed_state(msg);
    am_telemetry_high_lat->throttle = mavlink_msg_am_telemetry_high_lat_get_throttle(msg);
    am_telemetry_high_lat->airspeed = mavlink_msg_am_telemetry_high_lat_get_airspeed(msg);
    am_telemetry_high_lat->airspeed_sp = mavlink_msg_am_telemetry_high_lat_get_airspeed_sp(msg);
    am_telemetry_high_lat->groundspeed = mavlink_msg_am_telemetry_high_lat_get_groundspeed(msg);
    am_telemetry_high_lat->climb_rate = mavlink_msg_am_telemetry_high_lat_get_climb_rate(msg);
    am_telemetry_high_lat->gps_nsat = mavlink_msg_am_telemetry_high_lat_get_gps_nsat(msg);
    am_telemetry_high_lat->gps_fix_type = mavlink_msg_am_telemetry_high_lat_get_gps_fix_type(msg);
    am_telemetry_high_lat->battery_remaining = mavlink_msg_am_telemetry_high_lat_get_battery_remaining(msg);
    am_telemetry_high_lat->temperature = mavlink_msg_am_telemetry_high_lat_get_temperature(msg);
    am_telemetry_high_lat->temperature_air = mavlink_msg_am_telemetry_high_lat_get_temperature_air(msg);
    am_telemetry_high_lat->failsafe = mavlink_msg_am_telemetry_high_lat_get_failsafe(msg);
    am_telemetry_high_lat->wp_num = mavlink_msg_am_telemetry_high_lat_get_wp_num(msg);
    am_telemetry_high_lat->rssi_gsm1 = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm1(msg);
    am_telemetry_high_lat->rssi_gsm2 = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm2(msg);
    am_telemetry_high_lat->rssi_gsm3 = mavlink_msg_am_telemetry_high_lat_get_rssi_gsm3(msg);
    am_telemetry_high_lat->rssi_sbd = mavlink_msg_am_telemetry_high_lat_get_rssi_sbd(msg);
    am_telemetry_high_lat->comp_status_bitmask = mavlink_msg_am_telemetry_high_lat_get_comp_status_bitmask(msg);
    am_telemetry_high_lat->telem_status_bitmask = mavlink_msg_am_telemetry_high_lat_get_telem_status_bitmask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN? msg->len : MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN;
        memset(am_telemetry_high_lat, 0, MAVLINK_MSG_ID_AM_TELEMETRY_HIGH_LAT_LEN);
    memcpy(am_telemetry_high_lat, _MAV_PAYLOAD(msg), len);
#endif
}
