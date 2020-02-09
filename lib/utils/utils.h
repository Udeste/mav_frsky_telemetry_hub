#include "Arduino.h"
#include "mavlink-fc-cache.h"

void turnLedON(uint16_t ledPin);
void turnLedOFF(uint16_t ledPin);
bool armed(mavlink_fc_cache* cache);
void printserialCache(mavlink_fc_cache* cache, HardwareSerial* serial);
float radToDeg(float rad);
float degToRad(float deg);