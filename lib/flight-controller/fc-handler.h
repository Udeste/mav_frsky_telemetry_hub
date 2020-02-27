#ifndef FC_COMM_HANDLER
#define FC_COMM_HANDLER

#include "Arduino.h"
#undef F
#include <common/mavlink.h>
#include <mavlink_types.h>
#include <ardupilotmega/ardupilotmega.h>

#define BUFFER_SIZE 300


class FCHandler {
 public:
  FCHandler           (HardwareSerial* serial);
  void                begin        (unsigned long baud);
  bool                read         (mavlink_message_t* message);
  void                setLedsPins  (uint16_t hb_to_fc_led, uint16_t hb_frm_fc_led);
  void                write        (mavlink_message_t message);

 private:
  HardwareSerial*     serial;
  mavlink_status_t    status;
  uint32_t            heartbeats = 0;
  uint8_t             buff[BUFFER_SIZE] = {};
};

#endif