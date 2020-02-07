#include "Arduino.h"

#ifndef MAVLINK_FC_CACHE
#define MAVLINK_FC_CACHE
  struct mavlink_fc_cache {
    /* HEARTBEAT Message from FC https://mavlink.io/en/messages/common.html#HEARTBEAT */
    uint8_t    hb_type                = 0; // https://mavlink.io/en/messages/common.html#MAV_TYPE
    uint8_t    hb_autopilot           = 0; // https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT
    uint8_t    hb_base_mode           = 0; // https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
    uint32_t   hb_copter_mode         = 0; // https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE
    uint8_t    hb_system_status       = 0; // https://mavlink.io/en/messages/common.html#MAV_STATE
    uint8_t    hb_mavlink_version     = 0; // https://mavlink.io/en/messages/common.html#MAV_STATE

    /* STATUSTEXT Message https://mavlink.io/en/messages/common.html#STATUSTEXT */
    uint8_t    status_severity		 = 0;  //https://mavlink.io/en/messages/common.html#MAV_SEVERITY
    char       status_text[50]     = ""; //Status text message, without null termination character
    uint16_t   status_text_len     = 0;

    /* SYS_STATUS Message https://mavlink.io/en/messages/common.html#SYS_STATUS */
    uint32_t   sys_onbrd_ctrl_sens_hlth = 0; //https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR
    uint16_t   sys_voltage_battery1     = 0; // mV
    uint16_t   sys_current_battery1     = 0; // cA
    int8_t     sys_battery_remaining    = 0; // %

    /* ATTITUDE Message https://mavlink.io/en/messages/common.html#ATTITUDE */
    uint32_t   att_time_boot_ms        = 0; // (time since system boot).
    float      att_roll                = 0; // rad	Roll angle (-pi..+pi)
    float      att_pitch               = 0; // rad	Pitch angle (-pi..+pi)
    float      att_yaw                 = 0; // rad	Yaw angle (-pi..+pi)
    float      att_rollspeed           = 0; // rad/s	Roll angular speed
    float      att_pitchspeed          = 0; // rad/s	Pitch angular speed
    float      att_yawspeed            = 0; // rad/s	Yaw angular speed

    /* VFR_HUD Message https://mavlink.io/en/messages/common.html#VFR_HUD */
    float      vfr_hud_airspeed            = 0; //	m/s	Current indicated airspeed (IAS).
    float      vfr_hud_groundspeed         = 0; //	m/s	Current ground speed.
    int16_t    vfr_hud_heading             = 0; //	deg	Current heading in compass units (0-360, 0=north).
    uint16_t   vfr_hud_throttle            = 0; //	%	Current throttle setting (0 to 100).
    float      vfr_hud_alt                 = 0; //	m	Current altitude (MSL).
    float      vfr_hud_climb               = 0; //	m/s	Current climb rate.

    /* GPS_STATUS Message https://mavlink.io/en/messages/common.html#GPS_STATUS */
    // uint8_t     satellites_visible	       = 0;	// Number of satellites visible
    // uint8_t[20] satellite_prn		  	     = 0; // Global satellite ID
    // uint8_t[20] satellite_used	           = 0; // 0: Satellite not used, 1: used for localization
    // uint8_t[20] satellite_elevation	     = 0; // deg	Elevation (0: right on top of receiver, 90: on the horizon) of satellite
    // uint8_t[20] satellite_azimuth	       = 0; // deg	Direction of satellite, 0: 0 deg, 255: 360 deg.
    // uint8_t[20] satellite_snr	           = 0; // dB	Signal to noise ratio of satellite

    uint64_t    gps_time_usec              = 0; //	us		Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
    uint8_t     gps_fix_type               = 0; //	https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
    int32_t     gps_lat                    = 0; //	degE7	Latitude (WGS84, EGM96 ellipsoid)
    int32_t     gps_lon                    = 0; //	degE7	Longitude (WGS84, EGM96 ellipsoid)
    int32_t     gps_alt                    = 0; //	mm		Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
    uint16_t    gps_eph                    = 0; //				GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    uint16_t    gps_epv                    = 0; //				GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    uint16_t    gps_vel                    = 0; //	cm/s	GPS ground speed. If unknown, set to: UINT16_MAX
    uint16_t    gps_cog                    = 0; //	cdeg	Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    uint8_t     gps_satellites_visible     = 0; //				Number of satellites visible. If unknown, set to 255
    int32_t     gps_alt_ellipsoid          = 0; //	mm		Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    uint32_t    gps_h_acc                  = 0; //	mm		Position uncertainty. Positive for up.
    uint32_t    gps_v_acc                  = 0; //	mm		Altitude uncertainty. Positive for up.
    uint32_t    gps_vel_acc                = 0; //	mm		Speed uncertainty. Positive for up.
    uint32_t    gps_hdg_acc                = 0; //	degE5		Heading / track uncertainty

    /* SCALED_IMU Message https://mavlink.io/en/messages/common.html#SCALED_IMU */
    uint32_t    imu_time_boot_ms           = 0;  // ms	Timestamp (time since system boot).
    int16_t     imu_xacc                   = 0; // mG	X acceleration
    int16_t     imu_yacc                   = 0; // mG	Y acceleration
    int16_t     imu_zacc                   = 0; // mG	Z acceleration
    int16_t     imu_xgyro                  = 0; // mrad/s	Angular speed around X axis
    int16_t     imu_ygyro                  = 0; // mrad/s	Angular speed around Y axis
    int16_t     imu_zgyro                  = 0; // mrad/s	Angular speed around Z axis
    int16_t     imu_xmag                   = 0; // mgauss	X Magnetic field
    int16_t     imu_ymag                   = 0; // mgauss	Y Magnetic field
    int16_t     imu_zmag                   = 0; // mgauss	Z Magnetic field
    int16_t     imu_temperature            = 0; // cdegC	Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).

    /* SCALED_PRESSURE Message https://mavlink.io/en/messages/common.html#SCALED_PRESSURE */
    float       scaled_press_abs                  = 0; // hPa	Absolute pressure
    float       scaled_press_diff                 = 0; // hPa	Differential pressure 1
    int16_t     scaled_temperature                = 0; // cdegC	Temperature
  };
#endif

void printDebugCache(mavlink_fc_cache* cache, HardwareSerial* serial);