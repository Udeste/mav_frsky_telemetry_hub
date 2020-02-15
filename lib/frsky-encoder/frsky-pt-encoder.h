#include "frsky-encoder.h"

class FrSkyPassThroughEncoder: public FrSkyEncoder {

  using                 FrSkyEncoder::FrSkyEncoder;

  public:
  void                  encode      ();

  private:
  void                  sendPT      ();

  #define HIGH_TIMING_MS  100
  #define MID_TIMING_MS   500
  #define LOW_TIMING_MS   1000

  uint32_t      last_live_timing_ms = 0;
  uint32_t      last_high_timing_ms = 0;
  uint32_t      last_mid_timing_ms = 0;
  uint32_t      last_low_timing_ms = 0;

  uint16_t      live_timing[2] = {
    FRSKY_PT_SENSOR_ID_VEL_YAW,
    FRSKY_PT_SENSOR_ID_ATT_RNG
  };
  uint16_t      high_timing[1] = {
    FRSKY_PT_SENSOR_ID_STATUS_TEXT
  };
  uint16_t      mid_timing[6] = {
    FRSKY_PT_SENSOR_ID_GPS_STATUS,
    FRSKY_SENSOR_ID_BATT,
    FRSKY_PT_SENSOR_ID_GPS_LAT,
    FRSKY_PT_SENSOR_ID_GPS_LON,
  };
  uint16_t      low_timing[3] = {
    FRSKY_PT_SENSOR_ID_AP_STATUS,
    FRSKY_PT_SENSOR_ID_HOME,
    FRSKY_PT_SENSOR_ID_PARAMETERS
  };


  /** AP_STATUS **/
  #define AP_IMU_TEMP_MIN       19.0f
  #define AP_IMU_TEMP_MAX       82.0f
  #define AP_CONTROL_MODE_LIMIT 0x1F
  uint32_t              calcApStatus();

  /** PARAMS **/
  #define PARAM_VALUE_LIMIT     0xFFFFFF
  uint32_t              calcParameters();

  /** GPS STATUS **/
  #define GPS_SATS_LIMIT            0xF // 15
  #define GPS_STATUS_LIMIT          0x3
  uint32_t              calcGPSStatus();

  /** ATTITUDE **/
  #define ATTIANDRNG_ROLL_LIMIT     0x7FF
  #define ATTIANDRNG_PITCH_LIMIT    0x3FF
  #define ROTATION_PITCH_270        25U
  uint32_t              calcAttitude();

  /** VELOCITY AND YAW **/
  #define VELANDYAW_YAW_LIMIT       0x7FF
  uint32_t              calcVelYaw();

  uint16_t              calcNextSensorToSend();
};
