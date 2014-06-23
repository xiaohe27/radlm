#include "house_node.h"

class House {
 private:
  float temp;
  float leak_rate;
  float interval;
 public:
  House();
  void step(const _in_t*, const _in_flags_t*, _out_t*, _out_flags_t*);
};



