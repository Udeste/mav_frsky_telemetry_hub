#include "WifiHandler.h"

#include <ESP8266WiFi.h>

#include "Arduino.h"

WifiHandler::WifiHandler(const char* ssid, const char* password, uint8_t chan) {
  this->ssid = ssid;
  this->password = password;
  this->chan = chan;
}

void WifiHandler::startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.encryptionType(AUTH_WPA2_PSK);
  WiFi.softAP(ssid, password, chan);
  //-- Boost power to Max
  setWifiPower(20.5);
  local_IP = WiFi.softAPIP();
}

void WifiHandler::startSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.encryptionType(AUTH_WPA2_PSK);
  WiFi.softAP(ssid, password, chan);
  //-- Boost power to Max
  setWifiPower(20.5);
  local_IP = WiFi.localIP();
  WiFi.setAutoReconnect(true);
}

bool WifiHandler::isWifiON() {
  return WiFi.getMode() != WIFI_OFF;
}

void WifiHandler::disconnect() {
  WiFi.disconnect();
}

bool WifiHandler::isClientConnected() {
  return isWifiON() && wifi_softap_get_station_num() > 0;
}

void WifiHandler::setWifiPower(float dbm) {
  WiFi.setOutputPower(dbm);
}

void WifiHandler::powerOff() {
  disconnect();
  WiFi.mode(WIFI_OFF);
}
