#include "Arduino.h"
#include "SoftwareSerial.h"
#undef F
#include <common/mavlink.h>
#include <mavlink_types.h>
#include <ardupilotmega/ardupilotmega.h>

//  extern "C" {
//     // Espressif SDK
//     #include "user_interface.h"
// }

/**
 * LEDS settings
 */
#define HAL_GPIO_A_LED_PIN          13 // LED_BUILTIN
#define HAL_GPIO_B_LED_PIN          31
#define HAL_GPIO_C_LED_PIN          30
#define HEARTHBEAT_TO_FC_LED_PIN    HAL_GPIO_C_LED_PIN
#define HEARTHBEAT_FROM_FC_LED_PIN  HAL_GPIO_A_LED_PIN

/**
 * FRSKY communication settings
 */
#define FRSKY_SWSERIAL_RX_PIN       12
#define FRSKY_SWSERIAL_TX_PIN       14
#define FRSKY_SWSERIAL_BAUD         57600
#define FRSKY_LED_PIN               LED_BUILTIN

/**
 * FLIGHT CONTROLLER commuication settings
 */
#define FC_SERIAL                   Serial1
#define FC_SERIAL_BAUD              115200

/**
 * BRIDGE mavlink configuration for this bridge
 */
#define HEARTBEAT_TO_FC_MS          2000
#define BRIDGE_SYSTEM_ID            9
#define BRIDGE_COMPONENT_ID         1
#define BRIDGE_TYPE                 MAV_TYPE_GCS
#define BRIDGE_AUTOPILOT            MAV_AUTOPILOT_ARDUPILOTMEGA
#define BRIDGE_BASE_MODE            0
#define BRIDGE_SYSTEM_STATUS        MAV_STATE_ACTIVE

/**
 * WIFI config section
 */
#define WIFI_SSID                   "ardupilot"
#define WIFI_PASS                   "ardupilot"
#define WIFI_CHAN                   9

#define DEBUG                       Serial
#define DEBUG_BAUD                  115200
