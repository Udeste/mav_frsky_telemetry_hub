#include "mavlink-parser.h"
#include "utils.h"

MavlinkParser::MavlinkParser(mavlink_fc_cache_t* cache,  HardwareSerial* serial) {
  this->cache = cache;
  this->serial = serial;
}

void MavlinkParser::setLedsPins(uint16_t hb_to_fc_led, uint16_t hb_frm_fc_led) {
  this->hb_to_fc_led_pin = hb_to_fc_led;
  this->hb_from_fc_led_pin = hb_frm_fc_led;
  pinMode(this->hb_to_fc_led_pin,   OUTPUT);
  pinMode(this->hb_from_fc_led_pin, OUTPUT);
}

void MavlinkParser::begin(unsigned long baud) {
  serial->begin(baud);
  // serial->setRxBufferSize(4096);
}

void MavlinkParser::readFC() {
  mavlink_status_t status;
  mavlink_message_t message;
  while (serial->available()) {
    uint16_t byte = serial->read();
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status)) {
      parseMavlinkMsg(message);
    }
  }
}

void MavlinkParser::parseMavlinkMsg(mavlink_message_t message) {
  switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      parseHBFromFC(message);
      break;
    case MAVLINK_MSG_ID_STATUSTEXT:
      parseSTATUSTEXT(message);
      break;
    case MAVLINK_MSG_ID_SYS_STATUS:
      parseSYS_STATUS(message);
      break;
    case MAVLINK_MSG_ID_ATTITUDE:
      parseATTITUDE(message);
      break;
    case MAVLINK_MSG_ID_VFR_HUD:
      parseVFR_HUD(message);
      break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
      parseGPS_RAW_INT(message);
      break;
    case MAVLINK_MSG_ID_RAW_IMU:
      parseRAW_IMU(message);
      break;
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
      parseSCALED_PRESSURE(message);
      break;
    case MAVLINK_MSG_ID_SYSTEM_TIME:
      parseSYSTEM_TIME(message);
      break;
    case MAVLINK_MSG_ID_SCALED_IMU:
    case MAVLINK_MSG_ID_SCALED_IMU2:
    case MAVLINK_MSG_ID_SCALED_IMU3:
    case MAVLINK_MSG_ID_GPS_STATUS:
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
    case MAVLINK_MSG_ID_RC_CHANNELS:
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    case MAVLINK_MSG_ID_POWER_STATUS:
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    case MAVLINK_MSG_ID_SENSOR_OFFSETS:
    case MAVLINK_MSG_ID_MEMINFO:
    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RANGEFINDER:
    case MAVLINK_MSG_ID_AHRS2:
    case MAVLINK_MSG_ID_BATTERY2:
    case MAVLINK_MSG_ID_AHRS3:
    default:  break;
  }
}

void MavlinkParser::parseHBFromFC(mavlink_message_t msg) {
  turnLedON(this->hb_from_fc_led_pin);
  uint8_t type_tmp = mavlink_msg_heartbeat_get_type(&msg);

  if (type_tmp != MAV_TYPE_QUADROTOR)
    return;
  cache->hb_type            = type_tmp;
  cache->hb_autopilot       = mavlink_msg_heartbeat_get_autopilot(&msg);
  cache->hb_base_mode       = mavlink_msg_heartbeat_get_base_mode(&msg);
  cache->hb_copter_mode     = mavlink_msg_heartbeat_get_custom_mode(&msg);
  cache->hb_system_status   = mavlink_msg_heartbeat_get_system_status(&msg);
  cache->hb_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);

  last_hb_from_fc_ms = millis();

  if (hb_from_fc_count++ % 2 == 0 )
    turnLedOFF(this->hb_from_fc_led_pin);
}

void MavlinkParser::parseSTATUSTEXT(mavlink_message_t msg) {
  cache->status_severity  = mavlink_msg_statustext_get_severity(&msg);
  cache->status_text_len  = mavlink_msg_statustext_get_text(&msg, cache->status_text);
}

void MavlinkParser::parseSYS_STATUS(mavlink_message_t msg) {
  cache->sys_onbrd_ctrl_sens_hlth =
      mavlink_msg_sys_status_get_onboard_control_sensors_health(&msg);
  cache->sys_voltage_battery1 =
      mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1000 = 1V  i.e mV
  cache->sys_current_battery1 =
      mavlink_msg_sys_status_get_current_battery(&msg);  //  100 = 1A, i.e dA
}

void MavlinkParser::parseSYSTEM_TIME(mavlink_message_t msg) {
  cache->time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(&msg);
  cache->time_boot_ms   = mavlink_msg_system_time_get_time_boot_ms(&msg);
}

void MavlinkParser::parseATTITUDE(mavlink_message_t msg) {
  cache->att_roll       = mavlink_msg_attitude_get_roll(&msg);
  cache->att_pitch      = mavlink_msg_attitude_get_pitch(&msg);
  cache->att_yaw        = mavlink_msg_attitude_get_yaw(&msg);
  cache->att_rollspeed  = mavlink_msg_attitude_get_rollspeed(&msg);
  cache->att_pitchspeed = mavlink_msg_attitude_get_pitchspeed(&msg);
  cache->att_yawspeed   = mavlink_msg_attitude_get_yawspeed(&msg);
}

void MavlinkParser::parseVFR_HUD(mavlink_message_t msg) {
  cache->vfr_hud_airspeed     = mavlink_msg_vfr_hud_get_airspeed(&msg);
  cache->vfr_hud_groundspeed  = mavlink_msg_vfr_hud_get_groundspeed(&msg);
  cache->vfr_hud_heading      = mavlink_msg_vfr_hud_get_heading(&msg);
  cache->vfr_hud_throttle     = mavlink_msg_vfr_hud_get_throttle(&msg);
  cache->vfr_hud_alt          = mavlink_msg_vfr_hud_get_alt(&msg);
  cache->vfr_hud_climb        = mavlink_msg_vfr_hud_get_climb(&msg);
}

void MavlinkParser::parseGPS_RAW_INT(mavlink_message_t msg) {
  // cache->gps_time_usec          = mavlink_msg_gps_raw_int_get_time_usec(&msg);
  cache->gps_fix_type           = mavlink_msg_gps_raw_int_get_fix_type(&msg);
  cache->gps_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
  if(cache->gps_fix_type > GPS_FIX_TYPE_2D_FIX)  {
    cache->gps_lat              = mavlink_msg_gps_raw_int_get_lat(&msg);
    cache->gps_lon              = mavlink_msg_gps_raw_int_get_lon(&msg);
    cache->gps_alt              = mavlink_msg_gps_raw_int_get_alt(&msg);
    cache->gps_eph              = mavlink_msg_gps_raw_int_get_eph(&msg);
    cache->gps_epv              = mavlink_msg_gps_raw_int_get_epv(&msg);
    cache->gps_vel              = mavlink_msg_gps_raw_int_get_vel(&msg);
    cache->gps_cog              = mavlink_msg_gps_raw_int_get_cog(&msg);
    cache->gps_alt_ellipsoid    = mavlink_msg_gps_raw_int_get_alt_ellipsoid(&msg);
    cache->gps_h_acc            = mavlink_msg_gps_raw_int_get_h_acc(&msg);
    cache->gps_v_acc            = mavlink_msg_gps_raw_int_get_v_acc(&msg);
    cache->gps_vel_acc          = mavlink_msg_gps_raw_int_get_vel_acc(&msg);
    cache->gps_hdg_acc          = mavlink_msg_gps_raw_int_get_hdg_acc(&msg);
  }
}

void MavlinkParser::parseRAW_IMU(mavlink_message_t msg) {
  cache->raw_imu_xacc          = mavlink_msg_raw_imu_get_xacc(&msg);
  cache->raw_imu_yacc          = mavlink_msg_raw_imu_get_yacc(&msg);
  cache->raw_imu_zacc          = mavlink_msg_raw_imu_get_zacc(&msg);
  cache->raw_imu_xgyro         = mavlink_msg_raw_imu_get_xgyro(&msg);
  cache->raw_imu_ygyro         = mavlink_msg_raw_imu_get_ygyro(&msg);
  cache->raw_imu_zgyro         = mavlink_msg_raw_imu_get_zgyro(&msg);
  cache->raw_imu_xmag          = mavlink_msg_raw_imu_get_xmag(&msg);
  cache->raw_imu_ymag          = mavlink_msg_raw_imu_get_ymag(&msg);
  cache->raw_imu_zmag          = mavlink_msg_raw_imu_get_zmag(&msg);
  cache->raw_imu_temperature   = mavlink_msg_raw_imu_get_temperature(&msg);
}

void MavlinkParser::parseSCALED_PRESSURE(mavlink_message_t msg) {
  cache->scaled_press_abs   = mavlink_msg_scaled_pressure_get_press_abs(&msg);
  cache->scaled_press_diff  = mavlink_msg_scaled_pressure_get_press_diff(&msg);
  cache->scaled_temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
}

void MavlinkParser::sendHBToFC() {
  if (millis() - last_hb_to_fc_ms > HEARTBEAT_TO_FC_MS) {
    turnLedON(this->hb_to_fc_led_pin);

    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(BRIDGE_SYSTEM_ID,
                               BRIDGE_COMPONENT_ID,
                               &message,
                               BRIDGE_TYPE,
                               BRIDGE_AUTOPILOT,
                               BRIDGE_BASE_MODE,
                               BRIDGE_SYSTEM_STATE,
                               0);
    writeToFC(message);
    last_hb_to_fc_ms = millis();
  if (hb_to_fc_count++ % 2 == 0 )
    turnLedOFF(this->hb_to_fc_led_pin);
  }
}

void MavlinkParser::writeToFC(mavlink_message_t msg) {
  uint16_t len = mavlink_msg_to_send_buffer(fc_buff, &msg);
  serial->write(fc_buff, len);
}
