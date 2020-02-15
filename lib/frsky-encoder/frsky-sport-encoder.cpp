#include "frsky-sport-encoder.h"

void FrSkySPortEncoder::encode() {
  while (this->frsky_s_port->available()) {
    if (this->frsky_s_port->read() == FRSKY_POLL_HEADER) {
       while (this->frsky_s_port->available()) {
        switch (this->frsky_s_port->read()) {
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
        }
      };
    }
  };
}

void FrSkySPortEncoder::sendVario() {
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

void FrSkySPortEncoder::sendGPS() {
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

void FrSkySPortEncoder::sendSP2R() {
  switch (this->sensor_polls[7]) {
    case 0:
      break;
    case 1:
      break;
  }
  this->updateSensorPollsCount(7, 1);
}

void FrSkySPortEncoder::sendASS() {
  switch (this->sensor_polls[10]) {
    case 0:
      // AirSpeed
      this->frsky_s_port->sendData(FRSKY_SENSOR_ID_AIR_SPEED_FIRST, this->cache->vfr_hud_airspeed / 100);
      break;
  }
  this->updateSensorPollsCount(10, 0);
}

void FrSkySPortEncoder::sendFAS() {
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

void FrSkySPortEncoder::sendIMU() {
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

void FrSkySPortEncoder::sendTEMP() {
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

/** S-PORT HELPERS **/

uint32_t FrSkySPortEncoder::mavToFrskyDateTime(bool is_date) {
  parseTimestamp(this->cache->time_unix_usec / 1E6, &date_time);
  uint32_t data = 0x00000000;
  data |= (uint32_t)(is_date ? date_time.year : date_time.hours)  << 24;
  data |= (uint32_t)(is_date ? date_time.month : date_time.minutes) << 16;
  data |= (uint32_t)(is_date ? date_time.day : date_time.seconds)   << 8;

  return is_date ? data |= 0x000000ff : data;
}