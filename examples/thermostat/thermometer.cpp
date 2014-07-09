#include "thermometer.h"

#include <stdlib.h>

void Thermometer::step(const _in_t * in, const _in_flags_t* inflags,
                       _out_t * out, _out_flags_t* outflags){
  float noise = (float)(rand()%20000 - 10000)/10000;
  out->thermometer_temp->temp = in->house_temp->temp + noise;
}


