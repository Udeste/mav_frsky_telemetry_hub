#include "mavlink-fc-cache.h"
#include "frsky-s-port.h"
#include "frsky-sensors-id.h"
#include "Arduino.h"

class FrSkyEncoder {
  public:
  FrSkyEncoder          (mavlink_fc_cache* cache, uint16_t rxPin, uint16_t txPin, uint16_t ledPin = LED_BUILTIN);

  void                  encode();

  private:
  mavlink_fc_cache*     cache;
  FrskySPort*           frsky_s_port;
  uint16_t              sensor_polls[28] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
  };

  void                  sendVario       ();
  void                  sendGPS         ();
  void                  sendSP2R        ();
  void                  sendASS         ();
  void                  sendFAS         ();
  void                  sendIMU         ();
  void                  sendTEMP        ();
  void                  sendPT          ();

  void                  updateSensorPollsCount(uint16_t index, uint16_t max);
};
