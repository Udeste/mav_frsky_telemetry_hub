#include "utils.h"

void turnLedON(uint16_t ledPin) {
  if (ledPin)
    digitalWrite(ledPin, HIGH);
}

void turnLedOFF(uint16_t ledPin) {
  if (ledPin)
    digitalWrite(ledPin, LOW);
}

bool armed(mavlink_fc_cache_t* cache) {
  return cache->hb_base_mode >> 7;
}

int32_t debud_ms;
void printserialCache(mavlink_fc_cache_t* cache, HardwareSerial* serial) {
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
    // serial->print(" | ");
    // serial->print("system_status=");
    // serial->print(cache->hb_system_status);
    // serial->print(" | ");
    // serial->print("mavlink_version=");
    // serial->print(cache->hb_mavlink_version);

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
    // serial->print((float)cache->vfr_hud_groundspeed, 3);
    serial->print(" | ");
    serial->print("heading=");
    serial->print((float)cache->vfr_hud_heading, 1);
    serial->print(" | ");
    serial->print("throttle=");
    // serial->print(cache->vfr_hud_throttle);
    serial->print(" | ");
    serial->print("alt=");
    serial->print((float)cache->vfr_hud_alt, 3);
    serial->print(" | ");
    serial->print("climb=");
    serial->print((float)cache->vfr_hud_climb, 3);

    serial->println();

    serial->print(">>> GPS_RAW_INT >>>    ");
    // serial->print("time_usec=");
    // serial->print((long)cache->gps_time_usec / 1E6);
    // serial->print(" | ");
    serial->print("fix_type=");
    serial->print(cache->gps_fix_type);
    serial->print(" | ");
    serial->print("satellites_visible=");
    serial->print(cache->gps_satellites_visible);
    serial->print(" | ");
    serial->print("lat=");
    // serial->print((float)cache->gps_lat / 1E7);
    serial->print(" | ");
    serial->print("lon=");
    // serial->print((float)cache->gps_lon / 1E7);
    serial->print(" | ");
    serial->print("alt=");
    // serial->print((float)cache->gps_alt / 1000);
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
    // serial->print(cache->gps_cog);
    serial->print(" | ");
    serial->print("alt_ellipsoid=");
    // serial->print(cache->gps_alt_ellipsoid);
    serial->print(" | ");
    serial->print("h_acc=");
    // serial->print(cache->gps_h_acc);
    serial->print(" | ");
    serial->print("v_acc=");
    // serial->print(cache->gps_v_acc);
    serial->print(" | ");
    serial->print("vel_acc=");
    // serial->print(cache->gps_vel_acc);
    serial->print(" | ");
    serial->print("hdg_acc=");
    // serial->print(cache->gps_hdg_acc);

    serial->println();

    serial->print(">>> RAW_IMU >>>    ");
    // serial->print("raw_imu_xacc=");
    // serial->print((float)cache->raw_imu_xacc / 1000); // X = x * 1000.0f / 9.80665f 
    // serial->print(" | ");
    // serial->print("raw_imu_yacc=");
    // serial->print((float)cache->raw_imu_yacc / 1000);
    // serial->print(" | ");
    // serial->print("raw_imu_zacc=");
    // serial->print((float)cache->raw_imu_zacc / 1000);
    // serial->print(" | ");
    // serial->print("raw_imu_xgyro=");
    // serial->print((float)cache->raw_imu_xgyro / 1000);
    // serial->print(" | ");
    // serial->print("raw_imu_ygyro=");
    // serial->print((float)cache->raw_imu_ygyro / 1000);
    // serial->print(" | ");
    // serial->print("raw_imu_zgyro=");
    // serial->print((float)cache->raw_imu_zgyro / 1000);
    // serial->print(" | ");
    // serial->print("raw_imu_xmag=");
    // serial->print(cache->raw_imu_xmag);
    // serial->print(" | ");
    // serial->print("raw_imu_ymag=");
    // serial->print(cache->raw_imu_ymag);
    // serial->print(" | ");
    // serial->print("raw_imu_zmag=");
    // serial->print(cache->raw_imu_zmag);
    // serial->print(" | ");
    serial->print("raw_imu_temperature=");
    serial->print(cache->raw_imu_temperature);

    serial->println();

    serial->print(">>> SCALED_PRESSURE >>>    ");
    // serial->print("scaled_press_abs=");
    // serial->print(cache->scaled_press_abs);
    // serial->print(" | ");
    // serial->print("scaled_press_diff=");
    // serial->print(cache->scaled_press_diff);
    // serial->print(" | ");
    serial->print("scaled_temperature=");
    serial->print(cache->scaled_temperature);

    serial->println();

    serial->print(">>> BATTERY_STATUS >>>      ");
    // serial->print(" | ");
    // serial->print("battery_id=");
    // serial->print(cache->battery_id);
    // serial->print(" | ");
    // serial->print("battery_function=");
    // serial->print(cache->battery_function);
    // serial->print(" | ");
    // serial->print("battery_type=");
    // serial->print(cache->battery_type);
    // serial->print(" | ");
    // serial->print("battery_temperature=");
    // serial->print(cache->battery_temperature);
    // serial->print(" | ");
    // serial->print("battery_voltages=");
    // serial->print(cache->battery_voltages[0]);
    // serial->print(" | ");
    // serial->print("battery_current=");
    // serial->print(cache->battery_current);
    // serial->print(" | ");
    serial->print("battery_consumed=");
    serial->print(cache->battery_consumed);
    // serial->print(" | ");
    // serial->print("battery_energy_consumed=");
    // serial->print(cache->battery_energy_consumed);
    serial->print(" | ");
    // serial->print("battery_remaining=");
    // serial->print(cache->battery_remaining);
    // serial->print(" | ");
    // serial->print("battery_time_remaining=");
    // serial->print(cache->battery_time_remaining);
    // serial->print(" | ");
    // serial->print("battery_charge_state=");
    // serial->print(cache->battery_charge_state);

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


/*
 * CODE FROM ardupilot 4.0
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t prepNumber(int32_t number, uint8_t digits, uint8_t power) {
  uint16_t res = 0;
  uint32_t abs_number = abs(number);

  if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
      if (abs_number < 100) {
          res = abs_number<<1;
      } else if (abs_number < 1270) {
          res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
      } else { // transmit max possible value (0x7F x 10^1 = 1270)
          res = 0xFF;
      }
      if (number < 0) { // if number is negative, add sign bit in front
          res |= 0x1<<8;
      }
  } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
      if (abs_number < 100) {
          res = abs_number<<2;
      } else if (abs_number < 1000) {
          res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
      } else if (abs_number < 10000) {
          res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
      } else if (abs_number < 127000) {
          res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
      } else { // transmit max possible value (0x7F x 10^3 = 127000)
          res = 0x1FF;
      }
      if (number < 0) { // if number is negative, add sign bit in front
          res |= 0x1<<9;
      }
  } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
      if (abs_number < 1000) {
          res = abs_number<<1;
      } else if (abs_number < 10240) {
          res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
      } else { // transmit max possible value (0x3FF x 10^1 = 10240)
          res = 0x7FF;
      }
      if (number < 0) { // if number is negative, add sign bit in front
          res |= 0x1<<11;
      }
  } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
      if (abs_number < 1000) {
          res = abs_number<<2;
      } else if (abs_number < 10000) {
          res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
      } else if (abs_number < 100000) {
          res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
      } else if (abs_number < 1024000) {
          res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
      } else { // transmit max possible value (0x3FF x 10^3 = 127000)
          res = 0xFFF;
      }
      if (number < 0) { // if number is negative, add sign bit in front
          res |= 0x1<<12;
      }
  }
  return res;
}
