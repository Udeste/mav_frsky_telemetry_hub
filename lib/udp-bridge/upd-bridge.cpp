#include "upd-bridge.h"

UDPBridge::UDPBridge(HardwareSerial* serial) {
  this->serial = serial;
}

void UDPBridge::begin(uint16_t local_port) {
  memset(&gcs_message, 0, sizeof(gcs_message));
  memset(&gcs_status,  0, sizeof(gcs_status));
  memset(&fc_message,  0, sizeof(fc_message));
  memset(&fc_status,   0, sizeof(fc_status));
  udp.begin(local_port);
  initialized = true;
}

void UDPBridge::stop() {
  gcs_heartbeats = 0;
  fc_heartbeats = 0;
  udp.stop();
  initialized = false;
}

bool UDPBridge::readGCS() {
  bool msgReceived = false;
  int udp_count = udp.parsePacket();

  if (udp_count == 0) return msgReceived;

  while(udp_count--) {
    int result = udp.read();
    if (result >= 0) {
      msgReceived = mavlink_parse_char(MAVLINK_COMM_2, result, &gcs_message, &gcs_status);
      if (msgReceived) {
        if (fc_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
          gcs_heartbeats++;

        break;
      }
    }
  }
  return msgReceived;
}

bool UDPBridge::readFC() {
  uint16_t pkt_count = serial->available();
  bool msgReceived = false;

  if (pkt_count == 0) return msgReceived;

  while (pkt_count--) {
    int byte = serial->read();
    if (byte >= 0) {
      msgReceived = mavlink_parse_char(MAVLINK_COMM_1, byte, &fc_message, &fc_status);
      if(msgReceived) {
        if (fc_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
          fc_heartbeats++;

        break;
      }
    }
  }
  return msgReceived;
}

void UDPBridge::writeGCS() {
  if (gcs_heartbeats == 0) return;

  uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)gcs_buf, &fc_message);
  if(udp.beginPacket(udp.remoteIP(), udp.remotePort())) {
    udp.write((uint8_t*)(void*)gcs_buf, len);
    udp.endPacket();
    memset(&fc_message, 0, sizeof(fc_message));
    memset(&fc_status,  0, sizeof(fc_status));
  }
}

void UDPBridge::writeFC() {
  if (fc_heartbeats == 0) return;

  uint16_t len = mavlink_msg_to_send_buffer((uint8_t *)fc_buf, &gcs_message);
  serial->write((uint8_t*)(void*)fc_buf,len);
  memset(&gcs_message, 0, sizeof(gcs_message));
  memset(&gcs_status,  0, sizeof(gcs_status));
}
