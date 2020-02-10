#include "Arduino.h"
#include "mavlink-fc-cache.h"
#include "time.h"

void turnLedON(uint16_t ledPin);
void turnLedOFF(uint16_t ledPin);
bool armed(mavlink_fc_cache_t* cache);
float radToDeg(float rad);
float degToRad(float deg);
#ifdef DEBUG
  void printserialCache(mavlink_fc_cache_t* cache, HardwareSerial* serial);
#endif