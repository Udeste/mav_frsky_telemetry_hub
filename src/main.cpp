#include "settings.h"
#include "utils.h"
#include "mavlink-parser.h"
#include "frsky-encoder.h"
#include "time.h"

mavlink_fc_cache_t cache;
MavlinkParser mav_parser(&cache, &FC_SERIAL);
FrSkyEncoder frsky_encoder(&cache,
                          FRSKY_SWSERIAL_RX_PIN,
                          FRSKY_SWSERIAL_TX_PIN,
                          FRSKY_LED_PIN);
                          date_time_t dt;
#ifdef WIFI
  WifiHandler wifiHandler(WIFI_SSID,
                          WIFI_PASS,
                          WIFI_CHAN);
#endif

void setup() {
#ifdef DEBUG
  DEBUG.begin(DEBUG_BAUD);
  DEBUG.println("START SETUP");
#endif

#if defined HEARTHBEAT_TO_FC_LED_PIN && defined HEARTHBEAT_TO_FC_LED_PIN
  mav_parser.setLedsPins(HEARTHBEAT_TO_FC_LED_PIN,
                         HEARTHBEAT_FROM_FC_LED_PIN);
#endif

#ifdef WIFI
  wifiHandler.disconnect();
#endif

  mav_parser.begin(FC_SERIAL_BAUD);

#ifdef DEBUG
  DEBUG.println("SETUP COMPLETE");
#endif
}

void loop() {
#ifdef WIFI
  if (wifiHandler.isWifiON()) {
    if (wifiHandler.isClientConnected()) {
      // Handle phone communication
    } else if (armed(&cache)) {
      wifiHandler.powerOff();
    }
  } else {
#else
  if (true) {
#endif
    mav_parser.readFC();
    delay(0);
    mav_parser.sendHBToFC();
    delay(0);
    frsky_encoder.encode();
    delay(0);
  }

#ifdef DEBUG
  // printserialCache(&cache, &DEBUG);
#endif
}