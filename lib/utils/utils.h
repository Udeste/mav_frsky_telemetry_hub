#include "Arduino.h"
#include "mavlink-fc-cache.h"
#include "time.h"

void turnLedON(uint16_t ledPin);
void turnLedOFF(uint16_t ledPin);
bool armed(mavlink_fc_cache* cache);
void printserialCache(mavlink_fc_cache* cache, HardwareSerial* serial);
float radToDeg(float rad);
float degToRad(float deg);
uint32_t mavToFrskyGPS(float latLon, bool isLat);
uint32_t mavToFrskyDateTime(uint64_t timestamp, DateTime_t &date_time, bool is_date);