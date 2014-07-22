#include <iostream>
#include "n2_node.h"
class N2 {
public:
  void step(const _in_t* i, const _in_flags_t* f,
        _out_t* o, _out_flags_t* of) {
    std::cout << i->i2->x1;
  }
};

