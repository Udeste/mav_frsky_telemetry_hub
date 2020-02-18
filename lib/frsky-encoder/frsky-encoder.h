#include "mavlink-fc-cache.h"
#include "frsky-s-port.h"
#include "frsky-sensors-id.h"
#include "Arduino.h"
#include "utils.h"

class FrSkyEncoder {
  public:
  FrSkyEncoder          (mavlink_fc_cache_t* cache,
                         uint16_t rx_pin,
                         uint16_t tx_pin,
                         uint16_t led_pin = LED_BUILTIN);

  virtual void encode() = 0;

  protected:
  mavlink_fc_cache_t*   cache;
  FrskySPort*           frsky_s_port;
  uint32_t              sensor_polls[28] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
  };

  void                  updateSensorPollsCount(uint16_t index, uint16_t max);

  uint32_t              mavToFrskyGPS(float latLon, bool isLat);
};
