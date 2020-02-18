#include "Arduino.h"
#include "mavlink-parser.h"
#include "time.h"

/**
 * Board specific settings
 */
#if defined ESP8266
   extern "C" {
    // Espressif SDK
    #include "user_interface.h"
  }
  #define WIFI
  #define WIFI_SSID                   "..::Ardupilot::.."
  #define WIFI_PASS                   "ardupilot"
  #define WIFI_CHAN                   9
  #define FRSKY_SWSERIAL_RX_PIN       12
  #define FRSKY_SWSERIAL_TX_PIN       14
  #define FC_SERIAL                   Serial
  #define FC_SERIAL_BAUD              57600
  #define FRSKY_LED_PIN               LED_BUILTIN
#elif defined(__AVR_ATmega328P__)
  // #define HEARTHBEAT_TO_FC_LED_PIN    HAL_GPIO_C_LED_PIN
  // #define HEARTHBEAT_FROM_FC_LED_PIN  HAL_GPIO_A_LED_PIN
  #define FRSKY_LED_PIN               LED_BUILTIN
  #define FRSKY_SWSERIAL_RX_PIN       5
  #define FRSKY_SWSERIAL_TX_PIN       6
  #define FC_SERIAL                   Serial
  #define FC_SERIAL_BAUD              57600
#elif defined(__AVR_ATmega2560__)
  #define HAL_GPIO_A_LED_PIN          13 // LED_BUILTIN
  #define HAL_GPIO_B_LED_PIN          31
  #define HAL_GPIO_C_LED_PIN          30
  #define HEARTHBEAT_TO_FC_LED_PIN    HAL_GPIO_C_LED_PIN
  #define HEARTHBEAT_FROM_FC_LED_PIN  HAL_GPIO_A_LED_PIN
  #define FRSKY_LED_PIN               HAL_GPIO_B_LED_PIN
  #define FRSKY_SWSERIAL_RX_PIN       A9
  #define FRSKY_SWSERIAL_TX_PIN       A8
  #define FC_SERIAL                   Serial2
  #define FC_SERIAL_BAUD              57600
#endif

// #define FRSKY_TELEMETRY_MODE_SPORT
#define FRSKY_TELEMETRY_MODE_PASSTHROUGH
