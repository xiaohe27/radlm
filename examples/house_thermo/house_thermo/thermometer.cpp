#include "thermometer.h"

#include <stdlib.h>

namespace house_thermo {

void Thermometer::step(_in_thermometer * inmsgs, _flags_thermometer* flags, _out_thermometer * outmsgs) {
  float noise = (float)(rand()%20000 - 10000)/10000;
  outmsgs->thermometer_temp->temp = inmsgs->house_temp->temp + noise;
}

}

