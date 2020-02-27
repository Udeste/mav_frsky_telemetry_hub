#ifndef GCS_COMM_HANDLER
#define GCS_COMM_HANDLER

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#undef F
#include <common/mavlink.h>
#include <mavlink_types.h>
#include <ardupilotmega/ardupilotmega.h>

#define BUFFER_SIZE 300

class GCShandler {
 public:
  GCShandler          (uint16_t local_port);
  bool                initialized = false;

  void                begin      ();
  void                stop       ();
  bool                hasPacket  ();
  bool                read       (mavlink_message_t* msg);
  void                write      (mavlink_message_t msg);

 private:
  uint16_t            local_port = 0;
  uint32_t            heartbeats = 0;
  mavlink_status_t    status;
  uint8_t             buff[BUFFER_SIZE] = {};
  WiFiUDP             udp;
  IPAddress           client_ip;
  uint16_t            client_port;
};

#endif