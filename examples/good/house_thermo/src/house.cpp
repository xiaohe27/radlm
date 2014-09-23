#include "house.h"

House::House() {
  this->leak_rate = 0.1;
  this->interval = 0.02;
  this->temp = 70.0;
}

void House::step(const _in_t * in, const _in_flags_t* iflags,
                 _out_t * out, _out_flags_t * oflags) {
  this->temp += this->interval*(in->heater_rate->rate - this->leak_rate);
  out->house_temp->temp = this->temp;
}


