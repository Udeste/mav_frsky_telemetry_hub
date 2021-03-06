#include "settings.h"
#include "Arduino.h"
#include "mavlink-parser.h"
#include "fc-handler.h"

#ifdef WIFI
  #include "wifi-handler.h"
  #include "gcs-handler.h"
#endif
#ifdef FRSKY_TELEMETRY_MODE_PASSTHROUGH
  #include "frsky-pt-encoder.h"
#else
  #include "frsky-sport-encoder.h"
#endif

mavlink_fc_cache_t      cache;
mavlink_message_t       mav_message;
mavlink_message_t       gcs_message;
bool                    mav_pkt_available = false;

FCHandler               fc_handler(&FC_SERIAL);

#ifdef WIFI
  GCShandler              gcs_handler(UDP_LOCAL_PORT);
#endif

MavlinkParser           mav_parser(&cache);

#ifdef FRSKY_TELEMETRY_MODE_PASSTHROUGH
  FrSkyPassThroughEncoder frsky_encoder(&cache,
                                        FRSKY_SWSERIAL_RX_PIN,
                                        FRSKY_SWSERIAL_TX_PIN);
#else
  FrSkySPortEncoder frsky_encoder(&cache,
                                  FRSKY_SWSERIAL_RX_PIN,
                                  FRSKY_SWSERIAL_TX_PIN);
#endif

#ifdef WIFI
  ESP8266WifiHandler wifi_handler(WIFI_SSID,
                           WIFI_PASS,
                           WIFI_CHAN);
#endif


void setup() {
#if defined HEARTHBEAT_TO_FC_LED_PIN && defined HEARTHBEAT_TO_FC_LED_PIN
  mav_parser.setLedsPins(HEARTHBEAT_TO_FC_LED_PIN,
                         HEARTHBEAT_FROM_FC_LED_PIN);
#endif

  fc_handler.begin(FC_SERIAL_BAUD);
  frsky_encoder.begin();

#ifdef WIFI
  wifi_handler.disconnect();
  wifi_handler.powerOff();
#endif
}

void loop() {
  mav_pkt_available = fc_handler.read(&mav_message);
  if (mav_pkt_available) {
    mav_parser.parseMavlinkMsg(mav_message);
    frsky_encoder.encode();
  }

#ifdef WIFI

#ifdef DISABLE_WIFI_WHEN_ARMED
  if (armed(&cache) && wifi_handler.isWifiON()) {
    gcs_handler.stop();
    wifi_handler.powerOff();
  } else
#endif

  if(!wifi_handler.isWifiON()) {
    wifi_handler.startAP();
    //-- Boost power to Max
    wifi_handler.setWifiPower(MAX_WIFI_OUTPUT_POWER);
  }

  if (wifi_handler.isClientConnected()) {
    if (mav_pkt_available)
      gcs_handler.write(mav_message);

    if (gcs_handler.read(&gcs_message))
      fc_handler.write(gcs_message);

  } else if (gcs_handler.initialized){
    gcs_handler.stop();
  }


#endif
  if (mav_parser.makeHBMessage(&mav_message)) {
    fc_handler.write(mav_message);
  }
}