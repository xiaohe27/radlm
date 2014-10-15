#include "monitor.h"
#include "ros/ros.h"

void Monitor::step(const _in_t * in, const _in_flags_t* inflags,
                  _out_t * out, _out_flags_t* outflags) {
   if(radl::is_timeout(inflags->a_report)) {
       ROS_INFO("node a is dead");
   }
   if(radl::is_timeout(inflags->b_report)) {
       ROS_INFO("node b is dead");
   } else {
       if(radl::is_timeout(in->b_report->flag)) {
           ROS_INFO("node b receives timeout messages");
       }
   }
}


