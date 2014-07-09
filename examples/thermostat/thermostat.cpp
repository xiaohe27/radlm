
#include "thermostat.h"

Thermostat::Thermostat() {
  this->set_temp = 75.0;
  this->status = false;
  this->tol = 2.0;
}

void Thermostat::step(const _in_t * in, const _in_flags_t* inflags,
                      _out_t * out, _out_flags_t* outflags){
}

