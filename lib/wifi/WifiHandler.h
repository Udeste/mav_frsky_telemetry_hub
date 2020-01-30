#include "IPAddress.h"

class WifiHandler {
  public:
    WifiHandler         (const char* ssid, const char* password, uint8_t chan);
    void                startAP               ();
    void                startSTA              ();
    void                disconnect            ();
    bool                isClientConnected     ();
    void                setWifiPower          (float dbm);
    void                powerOff              ();
    bool                isWifiON              ();

  private:
    const char*         ssid;
    const char*         password;
    uint8_t             chan;
    IPAddress           local_IP;
};
