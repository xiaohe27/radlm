#include <iostream>
#include "n2_node.h"
using namespace std;
using namespace radl;

class N2 {
public:

  void step(const _in_t* i, const _in_flags_t* f,
        _out_t* o, _out_flags_t* of) {
    std::cout << i->i2->x1 << " " << is_stale(f->i2) <<" "<< is_timeout(f->i2) << endl;
  }
};

