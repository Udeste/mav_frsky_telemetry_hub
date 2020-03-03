#include "frsky-pt-encoder.h"

void FrSkyPassThroughEncoder::encode() {
  while (this->frsky_s_port->available() >= 2)
    if (this->frsky_s_port->read() == FRSKY_POLL_HEADER)
      while (this->frsky_s_port->available())
        if (this->frsky_s_port->read() == FRSKY_POLL_ID_FUEL)
          this->sendPT();
}

/**
 * Send Passthrough.
 */
void FrSkyPassThroughEncoder::sendPT() {
  switch (next_sensor) {
    case FRSKY_PT_SENSOR_ID_AP_STATUS: // 0x5001 AP status
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_AP_STATUS, calcApStatus());
      break;
    case FRSKY_PT_SENSOR_ID_GPS_STATUS: // 0x5002 GPS Status
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_GPS_STATUS, calcGPSStatus());
      break;
    case FRSKY_PT_SENSOR_ID_BATTERY_1: // 0x5003 Battery 1 status
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_BATTERY_1, calcBattery1());
      break;
    case FRSKY_PT_SENSOR_ID_HOME: // 0x5004 Home
      // this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_HOME, calcHome());
      break;
    case FRSKY_PT_SENSOR_ID_VEL_YAW: // 0x5005 Vel and Yaw
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_VEL_YAW, calcVelYaw());
      break;
    case FRSKY_PT_SENSOR_ID_STATUS_TEXT: { // 0x5000 status text
      uint32_t chunk = calcNextStatusTextChunk();
      if (chunk) {
        this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_STATUS_TEXT, chunk);
        break;
      }
    }
    case FRSKY_PT_SENSOR_ID_ATT_RNG: // 0x5006 Attitude and range
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_ATT_RNG, calcAttitude());
      break;
    case FRSKY_PT_SENSOR_ID_PARAMETERS: // 0x5007 parameters
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_PARAMETERS, calcParameters());
      break;
    case FRSKY_PT_SENSOR_ID_BATTERY_2: // 0x5008 Battery 2 status
      // this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_BATTERY_2, calcBattery2());
      break;
    case FRSKY_PT_SENSOR_ID_GPS_LAT: // 0x800 GPS lat
      this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_LONG_LATI_FIRST, mavToFrskyGPS(this->cache->global_pos_int_lat, true));
      break;
    case FRSKY_PT_SENSOR_ID_GPS_LON: // 0x801 GPS lon
      this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_LONG_LATI_FIRST, mavToFrskyGPS(this->cache->global_pos_int_lat, false));
      break;
    case FRSKY_PT_SENSOR_ID_VFR_HUD: // 0x50F2 VFR
      this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_VFR_HUD, calcVFRHud());
      break;
    default:
      break;
  }
  next_sensor = calcNextSensorToSend();
}

uint32_t FrSkyPassThroughEncoder::calcNextStatusTextChunk() {

  if (!status_text_available) {
    if (cache->status_text_buff.isEmpty()) return 0;
    parsing_text = cache->status_text_buff.shift();
    status_text_available = true;
    char_index = 0;
  }

  if (status_text_available) {
    uint32_t chunk = 0;
    uint8_t character = 0;
    for (int i = 3; i > -1 && char_index < sizeof(parsing_text.text); i--) {
      character = parsing_text.text[char_index++];

      if(!character) {
        break;
      }
      chunk |= (uint32_t)character << i * 8;
    }

    if (!character || char_index == sizeof(parsing_text.text)) {
      char_index = 0;
      status_text_available = false;
      chunk |= (uint32_t)(parsing_text.severity & 0x4) << 21;
      chunk |= (uint32_t)(parsing_text.severity & 0x2) << 14;
      chunk |= (uint32_t)(parsing_text.severity & 0x1) << 7;
    }
    return chunk;
  }

  return 0;
}

uint32_t FrSkyPassThroughEncoder::calcGPSStatus() {
  // number of GPS satellites visible (limit to 15 (0xF) since the value is stored on 4 bits)
  uint8_t num_sats = cache->gps_satellites_visible < GPS_SATS_LIMIT
                     ? cache->gps_satellites_visible
                     : GPS_SATS_LIMIT;
  // GPS receiver status (limit to 0-3 (0x3) since the value is stored on 2 bits: NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D or GPS_OK_FIX_3D_DGPS or GPS_OK_FIX_3D_RTK_FLOAT or GPS_OK_FIX_3D_RTK_FIXED = 3)
  uint8_t fix_type = cache->gps_fix_type < GPS_STATUS_LIMIT
                     ? cache->gps_fix_type
                     : GPS_STATUS_LIMIT;
  // GPS horizontal dilution of precision in dm
  uint8_t h_dop = prepNumber(roundf(cache->gps_epv / 10), 2, 1);
  // GPS receiver advanced status (0: no advanced fix, 1: GPS_OK_FIX_3D_DGPS, 2: GPS_OK_FIX_3D_RTK_FLOAT, 3: GPS_OK_FIX_3D_RTK_FIXED)
  uint8_t adv_fix_type = cache->gps_fix_type > GPS_STATUS_LIMIT
                         ? cache->gps_fix_type - GPS_STATUS_LIMIT
                         : 0;
  // Altitude MSL in dm
  uint16_t altmsl = prepNumber(roundf(cache->global_pos_int_alt / 10),2,2);

  uint32_t gps_status = 0;
  gps_status |= (uint32_t)num_sats     << 0;
  gps_status |= (uint32_t)fix_type     << 4;
  gps_status |= (uint32_t)h_dop        << 6;
  gps_status |= (uint32_t)adv_fix_type << 14;
  gps_status |= (uint32_t)altmsl       << 22;
  return gps_status;
}

uint32_t FrSkyPassThroughEncoder::calcAttitude() {
  // roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800]
  uint16_t atti_roll = (uint16_t)((radToDeg(cache->att_roll) * 100) + 18000) * 0.05f;
  atti_roll &= ATTIANDRNG_ROLL_LIMIT; // (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
  // pitch from [-9000;9000] centidegrees to unsigned .2 degree increments [0;900]
  uint16_t atti_pitch = (uint16_t)((radToDeg(cache->att_pitch) * 100) + 9000) * 0.05f;
  atti_pitch &= ATTIANDRNG_PITCH_LIMIT; // (just in case, limit to 1023 (0x3FF) since the value is stored on 10 bits)
  // rangefinder measurement in cm
  uint16_t range_finder = prepNumber(0, 3, 1);

  uint32_t attiandrng = 0;
  attiandrng |= (uint32_t)atti_roll    << 0;
  attiandrng |= (uint32_t)atti_pitch   << 11;
  attiandrng |= (uint32_t)range_finder << 21;
  return attiandrng;
}

uint32_t FrSkyPassThroughEncoder::calcVelYaw() {
  // vertical velocity in dm/s
  uint16_t v_vel = prepNumber(cache->vfr_hud_climb * 10, 2, 1);
  // horizontal velocity in dm/s
  uint16_t h_vel = prepNumber(cache->vfr_hud_groundspeed * 10, 2, 1);
  // yaw from [0;36000] centidegrees to .2 degree increments [0;1800]
  uint16_t yaw = (uint16_t)((cache->vfr_hud_heading * 10) * 0.5f);
  yaw &= VELANDYAW_YAW_LIMIT; // (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)

  uint32_t velandyaw = 0;
  velandyaw |= (uint32_t)v_vel << 0;
  velandyaw |= (uint32_t)h_vel << 9;
  velandyaw |= (uint32_t)yaw   << 17;

  return velandyaw;
}

uint32_t FrSkyPassThroughEncoder::calcApStatus() {
  uint8_t mode          = (uint8_t)(cache->hb_copter_mode + 1) & AP_CONTROL_MODE_LIMIT;
  uint8_t simple        = false;
  uint8_t s_simple      = false;
  uint8_t is_armed      = armed(cache);
  uint8_t land_complete = !is_armed;
  uint8_t batt_fs       = false;
  uint8_t ekf_fs        = false;
  uint8_t imu_temp      = (uint8_t)roundf(cache->raw_imu_temperature / 100) - AP_IMU_TEMP_MIN;

  uint32_t ap_status = 0;
  ap_status |= (uint32_t)mode          << 0;
  ap_status |= (uint32_t)simple        << 5;
  ap_status |= (uint32_t)s_simple      << 6;
  ap_status |= (uint32_t)land_complete << 7;
  ap_status |= (uint32_t)is_armed      << 8;
  ap_status |= (uint32_t)batt_fs       << 9;
  ap_status |= (uint32_t)ekf_fs        << 10;
  ap_status |= (uint32_t)imu_temp      << 26;

  return ap_status;
}

uint32_t FrSkyPassThroughEncoder::calcParameters() {
  // reuse sensor_polls array with id 27 as counter
  uint32_t param = 0;
  uint16_t param_id = this->sensor_polls[27] + 1;
  switch(param_id) {
    case 1: param |= cache->hb_type; break;
    case 2: // was used to send the battery failsafe voltage
    case 3: // was used to send the battery failsafe capacity in mAh
    case 4: // battery pack 1 capacity in mAh
    case 5: // battery pack 2 capacity in mAh
    default: break;
  }
  param |= (uint32_t)(param_id) << 24;

  this->updateSensorPollsCount(27, 4);

  return param;
}

uint32_t FrSkyPassThroughEncoder::calcBattery1() {
  uint16_t voltage       = ((uint16_t)(this->cache->sys_voltage_battery1 / 100) & BATT_VOLTAGE_LIMIT);
  uint16_t current       = prepNumber((uint16_t)this->cache->sys_current_battery1 / 100, 2, 1);
  uint16_t current_drawn = this->cache->battery_consumed;

  uint32_t battery_status = 0;
  battery_status |= (uint32_t)voltage       << 0;
  battery_status |= (uint32_t)current       << 9;
  battery_status |= (uint32_t)current_drawn << 17;
  return battery_status;
}

uint32_t FrSkyPassThroughEncoder::calcVFRHud() {
 uint16_t air_speed = prepNumber(roundf(cache->vfr_hud_airspeed * 10), 2, 1);
 uint16_t throttle  = cache->vfr_hud_throttle;
 uint16_t baro_alt  = prepNumber(roundf(cache->vfr_hud_alt * 10), 3, 2);

 uint32_t vfr = 0;
 vfr |= (uint32_t)air_speed << 0;
 vfr |= (uint32_t)throttle  << 8;
 vfr |= (uint32_t)baro_alt  << 15;
 vfr |= (uint32_t)(baro_alt < 0 ? 1 : 0) << 27;
 return vfr;
}

uint16_t FrSkyPassThroughEncoder::calcNextSensorToSend() {
  for (uint8_t i=0; i<NUM_SENSORS; i++) {
    if (millis() - sensor_polls[i] > sensors_timings[i]) {
      sensor_polls[i] = millis();
      return sensors_map[i];
    }
  }
  // Send again last sensor if no one is found
  return next_sensor;
}