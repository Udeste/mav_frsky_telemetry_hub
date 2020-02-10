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
  uint32_t last_timestamp;
} date_time_t;

void parseTimestamp(uint32_t timestamp, date_time_t* date_time);
bool isLeapYear(uint16_t year);

#endif