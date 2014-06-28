#include "thermometer_node.h"

namespace house_thermo {

class Thermometer {
 public: 
  void step(_in_thermometer * inmsgs, _flags_thermometer*, _out_thermometer * outmsgs);
};

}
