#include "n1_node.h"
class N1 {
  int32_t cpt;
public:
  void step(const _in_t* i, const _in_flags_t* f,
        _out_t* o, _out_flags_t* of) {
    o->o1->x1 = cpt;
    cpt++;
  }
};

