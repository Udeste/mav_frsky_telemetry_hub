#include "settings.h"
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
GCShandler              gcs_handler(UDP_LOCAL_PORT);
MavlinkParser           mav_parser(&cache);

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
#endif


void setup() {
#if defined HEARTHBEAT_TO_FC_LED_PIN && defined HEARTHBEAT_TO_FC_LED_PIN
  mav_parser.setLedsPins(HEARTHBEAT_TO_FC_LED_PIN,
                         HEARTHBEAT_FROM_FC_LED_PIN);
#endif

  fc_handler.begin(FC_SERIAL_BAUD);
}

void loop() {
  mav_pkt_available = fc_handler.read(&mav_message);
  if (mav_pkt_available) {
    mav_parser.parseMavlinkMsg(mav_message);
    delay(0);
    frsky_encoder.encode();
    delay(0);
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
  }

  if (wifi_handler.isClientConnected()) {
    if (mav_pkt_available)
      gcs_handler.write(mav_message);

    delay(0);

    if (gcs_handler.read(&gcs_message))
      fc_handler.write(gcs_message);

  } else if (gcs_handler.initialized){
    gcs_handler.stop();
  }


#endif
  if (mav_parser.makeHBMessage(&mav_message)) {
    fc_handler.write(mav_message);
    delay(0);
  }
}