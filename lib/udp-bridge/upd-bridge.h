#ifndef UDP_BRIDGE
#define UDP_BRIDGE

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#undef F
#include <common/mavlink.h>
#include <mavlink_types.h>
#include <ardupilotmega/ardupilotmega.h>

#define BUFFER_SIZE 300

class UDPBridge {
  public:
    UDPBridge         (HardwareSerial* serial);
    void  begin       (uint16_t local_port);
    void  stop        ();
    bool  readGCS     ();
    void  writeGCS    ();
    bool  readFC      ();
    void  writeFC     ();

    bool  initialized = false;
  private:
    char              gcs_buf[BUFFER_SIZE];
    uint32_t          gcs_heartbeats = 0;
    mavlink_status_t  gcs_status;
    mavlink_message_t gcs_message;

    char              fc_buf[BUFFER_SIZE];
    uint32_t          fc_heartbeats = 0;
    mavlink_status_t  fc_status;
    mavlink_message_t fc_message;

    HardwareSerial*   serial;
    WiFiUDP           udp;
    IPAddress         client_ip;
    uint16_t          client_port;
};

#endif
