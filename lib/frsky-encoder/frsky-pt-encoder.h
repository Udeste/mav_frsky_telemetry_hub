#include "frsky-encoder.h"

/** AP_STATUS **/
#define AP_IMU_TEMP_MIN        19.0f
#define AP_IMU_TEMP_MAX        82.0f
#define AP_CONTROL_MODE_LIMIT  0x1F

/** PARAMS **/
#define PARAM_VALUE_LIMIT      0xFFFFFF

/** GPS STATUS **/
#define GPS_SATS_LIMIT         0xF // 15
#define GPS_STATUS_LIMIT       0x3

/** ATTITUDE **/
#define ATTIANDRNG_ROLL_LIMIT  0x7FF
#define ATTIANDRNG_PITCH_LIMIT 0x3FF
#define ROTATION_PITCH_270     25U

/** VELOCITY AND YAW **/
#define VELANDYAW_YAW_LIMIT    0x7FF

/** BATTERY **/
#define BATT_VOLTAGE_LIMIT     0x1FF

class FrSkyPassThroughEncoder: public FrSkyEncoder {

  using     FrSkyEncoder::FrSkyEncoder;

  public:
  void      encode();

  private:
  // Inital sensor to send whenever the communication begin
  uint16_t  next_sensor = FRSKY_PT_SENSOR_ID_PARAMETERS;

  #define NUM_SENSORS 11

  uint16_t  sensors_map[NUM_SENSORS] = {
    // FRSKY_PT_SENSOR_ID_WIND,
    // FRSKY_PT_SENSOR_ID_BATTERY_2,
    // FRSKY_PT_SENSOR_ID_SERVO_RAW,
    // FRSKY_PT_SENSOR_ID_WAYPOINTS_MISSIONS,
    FRSKY_PT_SENSOR_ID_PARAMETERS,
    FRSKY_PT_SENSOR_ID_HOME,
    FRSKY_PT_SENSOR_ID_GPS_STATUS,
    FRSKY_PT_SENSOR_ID_GPS_LAT,
    FRSKY_PT_SENSOR_ID_GPS_LON,
    FRSKY_PT_SENSOR_ID_STATUS_TEXT,
    FRSKY_PT_SENSOR_ID_BATTERY_1,
    FRSKY_PT_SENSOR_ID_AP_STATUS,
    FRSKY_PT_SENSOR_ID_VEL_YAW,
    FRSKY_PT_SENSOR_ID_VFR_HUD,
    FRSKY_PT_SENSOR_ID_ATT_RNG
  };

  // The lower the higher priority
  // in milliseconds
  uint16_t  sensors_priority[NUM_SENSORS] = {
    // 0, // FRSKY_PT_SENSOR_ID_WIND
    // 0, // FRSKY_PT_SENSOR_ID_BATTERY_2
    // 0, // FRSKY_PT_SENSOR_ID_SERVO_RAW
    // 0, // FRSKY_PT_SENSOR_ID_WAYPOINTS_MISSIONS
    60000, // FRSKY_PT_SENSOR_ID_PARAMETERS
    11000, // FRSKY_PT_SENSOR_ID_HOME
    900,  // FRSKY_PT_SENSOR_ID_GPS_STATUS
    820,  // FRSKY_PT_SENSOR_ID_GPS_LAT
    800,  // FRSKY_PT_SENSOR_ID_GPS_LON
    750,  // FRSKY_PT_SENSOR_ID_STATUS_TEXT
    700,  // FRSKY_PT_SENSOR_ID_BATTERY_1
    500,  // FRSKY_PT_SENSOR_ID_AP_STATUS
    300,  // FRSKY_PT_SENSOR_ID_VEL_YAW
    130,  // FRSKY_PT_SENSOR_ID_VFR_HUD
    50    // FRSKY_PT_SENSOR_ID_ATT_RNG
  };

  uint32_t  sensor_last_ms[NUM_SENSORS] = {
    // 0, // FRSKY_PT_SENSOR_ID_WIND
    // 0, // FRSKY_PT_SENSOR_ID_BATTERY_2
    // 0, // FRSKY_PT_SENSOR_ID_SERVO_RAW
    // 0, // FRSKY_PT_SENSOR_ID_WAYPOINTS_MISSIONS
    0, // FRSKY_PT_SENSOR_ID_PARAMETERS
    0, // FRSKY_PT_SENSOR_ID_HOME
    0, // FRSKY_PT_SENSOR_ID_GPS_STATUS
    0, // FRSKY_PT_SENSOR_ID_GPS_LAT
    0, // FRSKY_PT_SENSOR_ID_GPS_LON
    0, // FRSKY_PT_SENSOR_ID_STATUS_TEXT
    0, // FRSKY_PT_SENSOR_ID_BATTERY_1
    0, // FRSKY_PT_SENSOR_ID_AP_STATUS
    0, // FRSKY_PT_SENSOR_ID_VEL_YAW
    0, // FRSKY_PT_SENSOR_ID_VFR_HUD
    0, // FRSKY_PT_SENSOR_ID_ATT_RNG
  };


  void      sendPT();
  uint16_t  calcNextSensorToSend();
  uint32_t  calcApStatus();
  uint32_t  calcParameters();
  uint32_t  calcGPSStatus();
  uint32_t  calcAttitude();
  uint32_t  calcVelYaw();
  uint32_t  calcBattery1();
  uint32_t  calcVFRHud();
};
