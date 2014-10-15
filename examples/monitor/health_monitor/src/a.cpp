#include "a.h"
#include "ros/ros.h"

A::A() {
  this->counter = 0;
}

void A::step(const _in_t * in, const _in_flags_t* inflags,
                  _out_t * out, _out_flags_t* outflags) {
    counter++;
    counter = counter%10000;
    out->pub_a->t_a = counter;
    ROS_INFO("sending: %d", counter);
}
