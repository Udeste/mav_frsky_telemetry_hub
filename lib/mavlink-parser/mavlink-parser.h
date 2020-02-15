#include "mavlink-fc-cache.h"
#undef F
#include <common/mavlink.h>
#include <mavlink_types.h>
#include <ardupilotmega/ardupilotmega.h>

/**
 * Mavlink configuration for this parser
 */
#define HEARTBEAT_TO_FC_MS          1000
#define BRIDGE_SYSTEM_ID            9
#define BRIDGE_COMPONENT_ID         1
#define BRIDGE_TYPE                 MAV_TYPE_GCS                // https://mavlink.io/en/messages/common.html#MAV_TYPE
#define BRIDGE_AUTOPILOT            MAV_AUTOPILOT_ARDUPILOTMEGA // https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT
#define BRIDGE_BASE_MODE            0
#define BRIDGE_SYSTEM_STATE         MAV_STATE_ACTIVE             // https://mavlink.io/en/messages/common.html#MAV_STATE


class MavlinkParser {
 public:
  MavlinkParser              (mavlink_fc_cache_t* cache, HardwareSerial* serial);
  void    begin              (unsigned long baud);
  void    readFC             ();
  void    sendHBToFC         ();
  void    setLedsPins        (uint16_t hb_to_fc_led, uint16_t hb_frm_fc_led);

 private:
  mavlink_fc_cache_t*   cache;
  HardwareSerial*     serial;
  uint8_t             fc_buff[300]        = {};
  uint32_t            last_hb_to_fc_ms    = 0;
  uint32_t            last_hb_from_fc_ms  = 0;
  uint16_t            hb_from_fc_led_pin  = 0;
  uint16_t            hb_to_fc_led_pin    = 0;
  uint32_t            hb_from_fc_count    = 0;
  uint32_t            hb_to_fc_count      = 0;

  void    writeToFC                  (mavlink_message_t msg);
  void    parseMavlinkMsg            (mavlink_message_t msg);
  void    parseHBFromFC              (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#HEARTBEAT
  void    parseSTATUSTEXT            (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#STATUSTEXT
  void    parseSYS_STATUS            (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#SYS_STATUS
  void    parseSYSTEM_TIME           (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#SYSTEM_TIME
  void    parseATTITUDE              (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#ATTITUDE
  void    parseVFR_HUD               (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#VFR_HUD
  void    parseGPS_RAW_INT           (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#GPS_RAW_INT
  void    parseGLOBAL_POSITION_INT   (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
  void    parseRAW_IMU               (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#RAW_IMU
  void    parseSCALED_PRESSURE       (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#SCALED_PRESSURE
  void    parseBATTERY_STATUS        (mavlink_message_t msg); // https://mavlink.io/en/messages/common.html#BATTERY_STATUS
};