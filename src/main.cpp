#include "settings.h"
#ifdef WIFI
  #include "wifi-handler.h"
  #include "upd-bridge.h"
#endif
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
  WifiHandler wifi_handler(WIFI_SSID,
                           WIFI_PASS,
                           WIFI_CHAN);
  UDPBridge udp_bridge(&FC_SERIAL);
#endif

void setup() {
#if defined HEARTHBEAT_TO_FC_LED_PIN && defined HEARTHBEAT_TO_FC_LED_PIN
  mav_parser.setLedsPins(HEARTHBEAT_TO_FC_LED_PIN,
                         HEARTHBEAT_FROM_FC_LED_PIN);
#endif

#ifdef WIFI
  wifi_handler.disconnect();
#endif

  mav_parser.begin(FC_SERIAL_BAUD);
  wifi_handler.startAP();
}

void loop() {

#ifdef WIFI
  if (!armed(&cache)) {
    if(!wifi_handler.isWifiON()) {
      wifi_handler.startAP();
    }
    if (wifi_handler.isClientConnected()) {
      if (!udp_bridge.initialized) {
        udp_bridge.begin(UDP_LOCAL_PORT);
      } else {
        if(udp_bridge.readGCS())
          udp_bridge.writeFC();
        delay(0);
        if(udp_bridge.readFC())
          udp_bridge.writeGCS();
      }
    } else {
      delay(500);
    }
  } else {
    if(wifi_handler.isWifiON()) {
      udp_bridge.stop();
      wifi_handler.powerOff();
    }
#endif

  mav_parser.readFC();
  delay(0);
  mav_parser.sendHBToFC();
  delay(0);
  frsky_encoder.encode();
  delay(0);

#ifdef WIFI
  }
#endif
}