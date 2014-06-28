#include "heater.h"

namespace house_thermo {

void Heater::step(_in_heater * in, _flags_heater* flags, _out_heater * out) {
  if (in->heater_switch->switch_on) {
    out->heater_rate->rate = 3.0;
  } else {
    out->heater_rate->rate = 0.0;
  }
}

}

