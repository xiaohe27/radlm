#include "thermostat_node.h"

namespace house_thermo {

class Thermostat {
 private:
  float set_temp;
  bool status;
  float tol;
 public:
  Thermostat();
  void step(_in_thermostat * inmsgs, _flags_thermostat*, _out_thermostat * outmsgs);
};

}
