#include "heater.h"

void Heater::step(const _in_t * in, const _in_flags_t* inflags,
                  _out_t * out, _out_flags_t* outflags) {
  if (in->heater_switch->switch_on) {
    out->heater_rate->rate = 3.0;
  } else {
    out->heater_rate->rate = 0.0;
  }
}


