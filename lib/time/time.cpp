#include "time.h"

static const uint8_t mthdays[]={31,28,31,30,31,30,31,31,30,31,30,31};

void parseTimestamp(uint32_t timestamp, date_time_t* date_time) {
  if (!timestamp || timestamp == date_time->last_timestamp) {
    return;
  }
  date_time->last_timestamp = timestamp;
  date_time->seconds = timestamp % 60;
  date_time->minutes = (timestamp / 60) % 60;
  date_time->hours   = (timestamp / 3600) % 24;

  timestamp /= 86400;

  // To increase performance do not recalculate day, month and year.
  if (!date_time->year && !date_time->month && !date_time->day) {
    uint8_t mthDays;
    unsigned long days = 0;
    date_time->year = 0;
    while((unsigned)(days += (isLeapYear(date_time->year) ? 366 : 365)) <= timestamp) {
      date_time->year++;
    }

    days -= isLeapYear(date_time->year) ? 366 : 365;
    timestamp -= days;

    days = 0;
    date_time->month = 0;
    mthDays=0;
    for (date_time->month = 0; date_time->month < 12; date_time->month++) {
      if (date_time->month == 1) { // february
        if (isLeapYear(date_time->year)) {
          mthDays = 29;
        } else {
          mthDays = 28;
        }
      } else {
        mthDays = mthdays[date_time->month];
      }

      if (timestamp >= mthDays) {
        timestamp -= mthDays;
      } else {
          break;
      }
    }
    date_time->day = timestamp + 1;
    date_time->month = date_time->month + 1;
    date_time->year = 70 - date_time->year;
  }
}

bool isLeapYear(uint16_t year) {
  return ((1970+year)>0) && !((1970+year)%4) && ( ((1970+year)%100) || !((1970+year)%400) );  
}