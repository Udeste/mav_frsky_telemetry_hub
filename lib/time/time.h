#ifndef TIME
#define TIME

#include "Arduino.h"
typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} DateTime_t;

void parseTimestamp(uint64_t timestamp, DateTime_t &date_time);
bool isLeapYear(uint16_t year);

#endif