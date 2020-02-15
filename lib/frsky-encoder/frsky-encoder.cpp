#include "frsky-encoder.h"

FrSkyEncoder::FrSkyEncoder(mavlink_fc_cache_t* cache,
                           uint16_t rx_pin,
                           uint16_t tx_pin,
                           uint16_t led_pin) {
  this->cache = cache;
  this->frsky_s_port = new FrskySPort(rx_pin, tx_pin);
  this->frsky_s_port->setLedPin(led_pin);
}

void FrSkyEncoder::updateSensorPollsCount(uint16_t index, uint16_t max) {
  if (this->sensor_polls[index]++ > max) this->sensor_polls[index] = 0;
}

/**
 * Mavlink GPS data is lat * 1E7
 * So (60 * 10000) / 1E7 = 0.06
 **/
uint32_t FrSkyEncoder::mavToFrskyGPS(float latLon, bool isLat) {
  uint32_t data = (uint32_t)(latLon * 0.06) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000; // South or West

  return data;
}