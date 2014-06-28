#include "house_node.h"

namespace house_thermo {

class House {
 private:
  float temp;
  float leak_rate;
  float interval;
 public:
  House();
  void step(_in_house * inmsgs, _flags_house*, _out_house * outmsgs);
};

}



