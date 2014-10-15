#include "b.h"
#include "ros/ros.h"

void B::step(const _in_t * in, const _in_flags_t* inflags,
                  _out_t * out, _out_flags_t* outflags) {
    ROS_INFO("received: %d, flags: %d", in->pub_a->t_a, inflags->pub_a);
}


