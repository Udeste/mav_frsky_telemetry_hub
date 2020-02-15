#include "Arduino.h"
#include "mavlink-fc-cache.h"
#include "time.h"

void        turnLedON(uint16_t ledPin);
void        turnLedOFF(uint16_t ledPin);
bool        armed(mavlink_fc_cache_t* cache);
float       radToDeg(float rad);
float       degToRad(float deg);
uint16_t    prepNumber(int32_t number, uint8_t digits, uint8_t power);

uint8_t  sizeOfArray(uint16_t* arr);

void printserialCache(mavlink_fc_cache_t* cache, HardwareSerial* serial);