#ifdef WIFI

#include "gcs-handler.h"

GCShandler::GCShandler(uint16_t local_port) {
  this->local_port = local_port;
}

void GCShandler::begin() {
  udp.begin(local_port);
  initialized = true;
}

void GCShandler::stop() {
  udp.stop();
  heartbeats = 0;
  initialized = false;
}

bool GCShandler::read(mavlink_message_t* msg) {
  if (!initialized) begin();

  int udp_count = udp.parsePacket();
  memset(msg, 0, sizeof(*msg));
  memset(&status,  0, sizeof(status));

  if (udp_count == 0) return false;

  while(udp_count--) {
    int result = udp.read();
    if (result >= 0) {
      if (mavlink_parse_char(MAVLINK_COMM_2, result, msg, &status)) {
        if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          heartbeats++;
        }

        return true;
      }
    }
  }
  return false;
}

void GCShandler::write(mavlink_message_t msg) {
  if (heartbeats == 0) return;

  if (!initialized) begin();

  uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buff, &msg);
  if(udp.beginPacket(udp.remoteIP(), udp.remotePort())) {
    udp.write((uint8_t*)(void*)buff, len);
    udp.endPacket();
  }
}

#endif
