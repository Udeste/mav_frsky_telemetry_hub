#include "frsky-encoder.h"

FrSkyEncoder::FrSkyEncoder(mavlink_fc_cache_t* cache,
                           uint16_t rx_pin,
                           uint16_t tx_pin,
                           uint16_t led_pin) {
  this->cache = cache;
  this->frsky_s_port = new FrskySPort(rx_pin, tx_pin);
  this->frsky_s_port->setLedPin(led_pin);
}

void FrSkyEncoder::encode() {
  while (this->frsky_s_port->available()) {
    if (this->frsky_s_port->read() == FRSKY_POLL_HEADER) {
       while (this->frsky_s_port->available()) {
        switch (this->frsky_s_port->read()) {
#if defined FRSKY_TELEMETRY_MODE_SPORT
          case FRSKY_POLL_ID_VARIO:  this->sendVario();  break;
          // case FRSKY_POLL_ID_FLVSS:                      break; /* Real MLVSS present on the S-Port bus. No need to send data from FC*/
          case FRSKY_POLL_ID_FAS:    this->sendFAS();    break;
          case FRSKY_POLL_ID_GPS:    this->sendGPS();    break;
          // case FRSKY_POLL_ID_RPM:
          // case FRSKY_POLL_ID_6:
          case FRSKY_POLL_ID_SP2R:   this->sendSP2R();   break;
          // case FRSKY_POLL_ID_8:
          // case FRSKY_POLL_ID_9:
          case FRSKY_POLL_ID_ASS:    this->sendASS();    break;
          // case FRSKY_POLL_ID_11:
          // case FRSKY_POLL_ID_12:
          // case FRSKY_POLL_ID_13:
          // case FRSKY_POLL_ID_14:
          // case FRSKY_POLL_ID_15:
          // case FRSKY_POLL_ID_16:
          // case FRSKY_POLL_ID_17:
          // case FRSKY_POLL_ID_18:
          // case FRSKY_POLL_ID_19:
          // case FRSKY_POLL_ID_20:
          // case FRSKY_POLL_ID_21:
          // case FRSKY_POLL_ID_22:
          // case FRSKY_POLL_ID_23:
          case FRSKY_POLL_ID_IMU:    this->sendIMU();    break;
          // case FRSKY_POLL_ID_25:
          // case FRSKY_POLL_ID_26:
          case FRSKY_POLL_ID_TEMP:   this->sendTEMP();   break;
#elif defined FRSKY_TELEMETRY_MODE_PASSTROUGH
          case FRSKY_POLL_ID_FUEL:   this->sendPT();     break;
#endif
        }
      };
    }
  };
}

#if defined FRSKY_TELEMETRY_MODE_SPORT
  void FrSkyEncoder::sendVario() {
    switch(this->sensor_polls[0]) {
      case 0:
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_ALT_FIRST,   cache->vfr_hud_alt * 100);
        break;
      case 1:
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_VARIO_FIRST, cache->vfr_hud_climb * 100);
        break;
    }
    this->updateSensorPollsCount(0, 1);
  }

  void FrSkyEncoder::sendGPS() {
    switch (this->sensor_polls[4]) {
      case 0:
        // LAT
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_LONG_LATI_FIRST, mavToFrskyGPS(this->cache->global_pos_int_lat, true));
        break;
      case 1:
        // LON
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_LONG_LATI_FIRST, mavToFrskyGPS(this->cache->global_pos_int_lon, false));
        break;
      case 2:
        //SPEED
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_SPEED_FIRST, (float)this->cache->gps_vel *  0.01944f);
        break;
      case 3:
        // ALT
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_ALT_FIRST, this->cache->global_pos_int_alt / 10);
        break;
      case 4:
        // COURSE
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_COURSE_FIRST, this->cache->vfr_hud_heading * 100);
        break;
      case 5:
        // DATA
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_TIME_DATE_FIRST, mavToFrskyDateTime(true));
        break;
      case 6:
        // TIME
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_GPS_TIME_DATE_FIRST, mavToFrskyDateTime(false));
        break;
    }
    this->updateSensorPollsCount(4, 6);
  }

  void FrSkyEncoder::sendSP2R() {
    switch (this->sensor_polls[7]) {
      case 0:
        break;
      case 1:
        break;
    }
    this->updateSensorPollsCount(7, 1);
  }

  void FrSkyEncoder::sendASS() {
    switch (this->sensor_polls[10]) {
      case 0:
        // AirSpeed
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_AIR_SPEED_FIRST, this->cache->vfr_hud_airspeed / 100);
        break;
    }
    this->updateSensorPollsCount(10, 0);
  }

  void FrSkyEncoder::sendFAS() {
    switch (this->sensor_polls[2]) {
      case 0:
        // BATTERY %
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_FUEL_FIRST, this->cache->sys_battery_remaining);
        break;
      case 1:
        // BATTERY VOLTAGE
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_VFAS_FIRST, (uint16_t)roundf(this->cache->sys_voltage_battery1 / 10));
        break;
      case 2:
        // BATTERY CURRENT
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_CURR_FIRST, (uint16_t)roundf(this->cache->sys_current_battery1 / 10));
        break;
    }
    this->updateSensorPollsCount(2, 2);
  }

  void FrSkyEncoder::sendIMU() {
    switch(this->sensor_polls[24]) {
      case 0:
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_ACCX_FIRST, (float)radToDeg(this->cache->att_pitch) * 100);
        break;
      case 1:
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_ACCY_FIRST, (float)radToDeg(this->cache->att_roll) * 100);
        break;
      case 2:
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_ACCZ_FIRST, (float)radToDeg(this->cache->att_yaw) * 100);
        break;
    }
    this->updateSensorPollsCount(24, 2);
  }

  void FrSkyEncoder::sendTEMP() {
    switch (this->sensor_polls[27]) {
      case 0:
        // TEMP1
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_T1_FIRST, (float)this->cache->scaled_temperature / 100);
      break;
      case 1:
        // TEMP2
        this->frsky_s_port->sendData(FRSKY_SENSOR_ID_T2_FIRST, (float)this->cache->raw_imu_temperature / 100);
      break;
    }
    this->updateSensorPollsCount(27, 1);
  }

#elif defined FRSKY_TELEMETRY_MODE_PASSTHROUGH
  /**
   * Send Passthrough.
   */
  void FrSkyEncoder::sendPT() {
    switch (this->sensor_polls[28]) {
      case 0: // 0x5000 status text
          // this->frsky_s_port->sendData(FRSKY_PT_SENSOR_ID_STATUS_TEXT, s);
          break;
      case 1: // 0x5006 Attitude and range
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+6, calc_attiandrng());
          break;
      case 2: // 0x800 GPS lat
          // sample both lat and lon at the same time
          // send_uint32(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(&_passthrough.send_latitude)); // gps latitude or longitude
          // _passthrough.gps_lng_sample = calc_gps_latlng(&_passthrough.send_latitude);
          // force the scheduler to select GPS lon as packet that's been waiting the most
          // this guarantees that gps coords are sent at max 
          // _passthrough.avg_polling_period*number_of_downlink_sensors time separation
          //  _passthrough.packet_timer[3] = _passthrough.packet_timer[2] - 10000;
          break;
      case 3: // 0x800 GPS lon
          // send_uint32(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, _passthrough.gps_lng_sample); // gps longitude
          break;
      case 4: // 0x5005 Vel and Yaw
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+5, calc_velandyaw());
          break;
      case 5: // 0x5001 AP status
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+1, calc_ap_status());
          break;
      case 6: // 0x5002 GPS Status
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+2, calc_gps_status());
          break;
      case 7: // 0x5004 Home
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+4, calc_home());
          break;
      case 8: // 0x5008 Battery 2 status
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+8, calc_batt(1));
          break;
      case 9: // 0x5003 Battery 1 status
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+3, calc_batt(0));
          break;
      case 10: // 0x5007 parameters
          // send_uint32(SPORT_DATA_FRAME, DIY_FIRST_ID+7, calc_param());
          break;
    }
    this->updateSensorPollsCount(28, 10);
  }
#endif

void FrSkyEncoder::updateSensorPollsCount(uint16_t index, uint16_t max) {
  if (this->sensor_polls[index]++ > max) this->sensor_polls[index] = 0;
}

/**
 * Mavlink GPS data is lat * 1E7
 * So (60 * 10000) / 1E7 = 0.06
 **/
uint32_t FrSkyEncoder::mavToFrskyGPS(float latLon, bool isLat) {
  uint32_t data = (uint32_t)(latLon * 0.06) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000; // South or West

  return data;
}

uint32_t FrSkyEncoder::mavToFrskyDateTime(bool is_date) {
  parseTimestamp(this->cache->time_unix_usec / 1E6, &date_time);
  uint32_t data = 0x00000000;
  data |= (uint32_t)(is_date ? date_time.year : date_time.hours)  << 24;
  data |= (uint32_t)(is_date ? date_time.month : date_time.minutes) << 16;
  data |= (uint32_t)(is_date ? date_time.day : date_time.seconds)   << 8;

  return is_date ? data |= 0x000000ff : data;
}