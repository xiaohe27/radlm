#include "house.h"

namespace house_thermo {

House::House() {
  this->leak_rate = 0.1;
  this->interval = 0.02;
  this->temp = 70.0;
}

void House::step(_in_house * in, _flags_house* flags,_out_house * out) {
  this->temp += this->interval*(in->heater_rate->rate - this->leak_rate);
  out->house_temp->temp = this->temp;
}

}


