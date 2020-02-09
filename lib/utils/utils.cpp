#include "utils.h"

void turnLedON(uint16_t ledPin) {
  if (ledPin)
    digitalWrite(ledPin, HIGH);
}

void turnLedOFF(uint16_t ledPin) {
  if (ledPin)
    digitalWrite(ledPin, LOW);
}

bool armed(mavlink_fc_cache* cache) {
  return cache->hb_base_mode >> 7;
}

int32_t debud_ms;

void printserialCache(mavlink_fc_cache* cache, HardwareSerial* serial) {
  if (millis() - debud_ms > 1000) {
    serial->print(">>> HEARTBEAT >>>   ");
    serial->print("type=");
    serial->print(cache->hb_type);
    serial->print(" | ");
    serial->print("autopilot=");
    serial->print(cache->hb_autopilot);
    serial->print(" | ");
    serial->print("base_mode=");
    serial->print(cache->hb_base_mode);
    serial->print(" | ");
    serial->print("custom_mode=");
    serial->print(cache->hb_copter_mode);
    serial->print(" | ");
    serial->print("system_status=");
    serial->print(cache->hb_system_status);
    serial->print(" | ");
    serial->print("mavlink_version=");
    serial->print(cache->hb_mavlink_version);

    serial->println();

    serial->print(">>> STATUSTEXT >>>   ");
    serial->print("severity=");
    serial->print(cache->status_severity);
    serial->print(" | ");
    serial->print("text=");
    serial->print(cache->status_text);

    serial->println();

    serial->print(">>> ATTITUDE >>>    ");
    serial->print("sensors_health=");
    serial->print(cache->sys_onbrd_ctrl_sens_hlth);  // 32b bitwise 0:
                                                         // error, 1: healthy.
    serial->print(" | ");
    serial->print("batt_volt=");
    serial->print(cache->sys_voltage_battery1);  // now V
    serial->print(" | ");
    serial->print("batt_amps=");
    serial->print(cache->sys_current_battery1);  // now A
    serial->print(" | ");
    serial->print("roll=");
    serial->print(cache->att_roll);
    serial->print(" | ");
    serial->print("pitch=");
    serial->print(cache->att_pitch);
    serial->print(" | ");
    serial->print("yaw=");
    serial->print(cache->att_yaw);

    serial->println();

    serial->print(">>> VFR_HUD >>>     ");
    serial->print("airspeed=");
    serial->print((float)cache->vfr_hud_airspeed, 3);
    serial->print(" | ");
    serial->print("groundspeed=");
    serial->print((float)cache->vfr_hud_groundspeed, 3);
    serial->print(" | ");
    serial->print("heading=");
    serial->print((float)cache->vfr_hud_heading, 1);
    serial->print(" | ");
    serial->print("throttle=");
    serial->print(cache->vfr_hud_throttle);
    serial->print(" | ");
    serial->print("alt=");
    serial->print((float)cache->vfr_hud_alt, 3);
    serial->print(" | ");
    serial->print("climb=");
    serial->print((float)cache->vfr_hud_climb, 3);

    serial->println();

    serial->print(">>> GPS_RAW_INT >>>    ");
    // serial->print("time_usec=");
    // serial->print((uint32_t)cache->gps_time_usec / 1000);
    // serial->print(" | ");
    serial->print("fix_type=");
    serial->print(cache->gps_fix_type);
    serial->print(" | ");
    serial->print("satellites_visible=");
    serial->print(cache->gps_satellites_visible);
    serial->print(" | ");
    serial->print("lat=");
    serial->print((float)cache->gps_lat / 1E7);
    serial->print(" | ");
    serial->print("lon=");
    serial->print((float)cache->gps_lon / 1E7);
    serial->print(" | ");
    serial->print("alt=");
    serial->print((float)cache->gps_alt / 1000);
    serial->print(" | ");
    serial->print("eph=");
    serial->print((float)cache->gps_eph / 100);
    serial->print(" | ");
    serial->print("epv=");
    serial->print(cache->gps_epv / 100);
    serial->print(" | ");
    serial->print("vel=");
    serial->print(cache->gps_vel);
    serial->print(" | ");
    serial->print("cog=");
    serial->print(cache->gps_cog);
    serial->print(" | ");
    serial->print("alt_ellipsoid=");
    serial->print(cache->gps_alt_ellipsoid);
    serial->print(" | ");
    serial->print("h_acc=");
    serial->print(cache->gps_h_acc);
    serial->print(" | ");
    serial->print("v_acc=");
    serial->print(cache->gps_v_acc);
    serial->print(" | ");
    serial->print("vel_acc=");
    serial->print(cache->gps_vel_acc);
    serial->print(" | ");
    serial->print("hdg_acc=");
    serial->print(cache->gps_hdg_acc);

    serial->println();

    serial->print(">>> RAW_IMU >>>    ");
    serial->print("raw_imu_xacc=");
    serial->print((float)cache->raw_imu_xacc / 1000); // X = x * 1000.0f / 9.80665f 
    serial->print(" | ");
    serial->print("raw_imu_yacc=");
    serial->print((float)cache->raw_imu_yacc / 1000);
    serial->print(" | ");
    serial->print("raw_imu_zacc=");
    serial->print((float)cache->raw_imu_zacc / 1000);
    serial->print(" | ");
    serial->print("raw_imu_xgyro=");
    serial->print((float)cache->raw_imu_xgyro / 1000);
    serial->print(" | ");
    serial->print("raw_imu_ygyro=");
    serial->print((float)cache->raw_imu_ygyro / 1000);
    serial->print(" | ");
    serial->print("raw_imu_zgyro=");
    serial->print((float)cache->raw_imu_zgyro / 1000);
    serial->print(" | ");
    serial->print("raw_imu_xmag=");
    serial->print(cache->raw_imu_xmag);
    serial->print(" | ");
    serial->print("raw_imu_ymag=");
    serial->print(cache->raw_imu_ymag);
    serial->print(" | ");
    serial->print("raw_imu_zmag=");
    serial->print(cache->raw_imu_zmag);
    serial->print(" | ");
    serial->print("raw_imu_temperature=");
    serial->print(cache->raw_imu_temperature);

    serial->println();

    serial->print(">>> SCALED_PRESSURE >>>    ");
    serial->print("scaled_press_abs=");
    serial->print(cache->scaled_press_abs);
    serial->print(" | ");
    serial->print("scaled_press_diff=");
    serial->print(cache->scaled_press_diff);
    serial->print(" | ");
    serial->print("scaled_temperature=");
    serial->print(cache->scaled_temperature);

    serial->println();
    serial->println();
    serial->println();
    debud_ms = millis();
  }
}

float radToDeg(float rad) {
  return rad * RAD_TO_DEG;
}

float degToRad(float deg) {
  return deg / RAD_TO_DEG;
}

/**
 * Mavlink GPS data is lat * 1E7
 * So (60 * 10000) / 1E7 = 0.06
 **/
uint32_t mavToFrskyGPS(float latLon, bool isLat) {
  uint32_t data = (uint32_t)(latLon * 0.06) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000; // South or West

  return data;
}

uint32_t mavToFrskyDateTime(uint64_t timestamp, DateTime_t &date_time, bool is_date) {
  parseTimestamp(timestamp, date_time);
  uint32_t data = 0x00000000;
  data |= (uint32_t)date_time.year  << 24;
  data |= (uint32_t)date_time.month << 16;
  data |= (uint32_t)date_time.day   << 8;

  return is_date ? data |= 0x000000ff : data;
}
