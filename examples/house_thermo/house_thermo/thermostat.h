#include "thermostat_node.h"

class Thermostat {
 private:
  float set_temp;
  bool status;
  float tol;
 public:
  Thermostat();
  void step(const _in_t*, const _in_flags_t*, _out_t*, _out_flags_t*);
};
