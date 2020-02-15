#include "frsky-encoder.h"
#include "frsky-sensors-id.h"

class FrSkySPortEncoder: public FrSkyEncoder {

  using                 FrSkyEncoder::FrSkyEncoder;

  public:
  void                  encode          ();

  private:
  date_time_t           date_time;

  void                  sendVario       ();
  void                  sendGPS         ();
  void                  sendSP2R        ();
  void                  sendASS         ();
  void                  sendFAS         ();
  void                  sendIMU         ();
  void                  sendTEMP        ();


  uint32_t              mavToFrskyDateTime(bool is_date);
};
