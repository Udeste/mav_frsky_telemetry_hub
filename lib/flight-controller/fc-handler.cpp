#include "fc-handler.h"

FCHandler::FCHandler(HardwareSerial* serial) {
  this->serial = serial;
}

void FCHandler::begin(unsigned long baud) {
  serial->begin(baud);
#if defined ESP8266
  serial->setRxBufferSize(4096);
#endif
}

bool FCHandler::read(mavlink_message_t* message) {

  int pkt_count = serial->available();
  if (pkt_count == 0) return false;

  memset(message, 0, sizeof(*message));
  memset(&status,  0, sizeof(status));

  while (pkt_count--) {
    uint16_t byte = serial->read();
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, message, &status)) {
      if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        heartbeats++;
      }
      return true;
    }
  }
  return false;
}

void FCHandler::write(mavlink_message_t message) {
  if (heartbeats == 0) return;

  uint16_t len = mavlink_msg_to_send_buffer((uint8_t *)buff, &message);
  serial->write((uint8_t*)(void*)buff, len);
}
