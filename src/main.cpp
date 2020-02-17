#include "settings.h"
#ifdef FRSKY_TELEMETRY_MODE_PASSTHROUGH
  #include "frsky-pt-encoder.h"
#else
  #include "frsky-sport-encoder.h"
#endif

mavlink_fc_cache_t cache;
date_time_t dt;
MavlinkParser mav_parser(&cache, &FC_SERIAL);

#ifdef FRSKY_TELEMETRY_MODE_PASSTHROUGH
  FrSkyPassThroughEncoder frsky_encoder(&cache,
                                        FRSKY_SWSERIAL_RX_PIN,
                                        FRSKY_SWSERIAL_TX_PIN,
                                        FRSKY_LED_PIN);
#else
  FrSkySPortEncoder frsky_encoder(&cache,
                                  FRSKY_SWSERIAL_RX_PIN,
                                  FRSKY_SWSERIAL_TX_PIN,
                                  FRSKY_LED_PIN);
#endif

#ifdef WIFI
  WifiHandler wifiHandler(WIFI_SSID,
                          WIFI_PASS,
                          WIFI_CHAN);
#endif

void setup() {
#if defined HEARTHBEAT_TO_FC_LED_PIN && defined HEARTHBEAT_TO_FC_LED_PIN
  mav_parser.setLedsPins(HEARTHBEAT_TO_FC_LED_PIN,
                         HEARTHBEAT_FROM_FC_LED_PIN);
#endif

#ifdef WIFI
  wifiHandler.disconnect();
#endif

  mav_parser.begin(FC_SERIAL_BAUD);
}

void loop() {

#ifdef WIFI
  if (!armed(&cache)) {
    if(!wifiHandler.isWifiOn()) {
      wifiHandler.startAP();
    }
    if (wifiHandler.isClientConnected()) {
      // Handle phone communication
    }
  } else if(wifiHandler.isWifiOn()) {
    wifiHandler.powerOff();
  }
#endif

  mav_parser.readFC();
  delay(0);
  mav_parser.sendHBToFC();
  delay(0);
  frsky_encoder.encode();
  delay(0);
}