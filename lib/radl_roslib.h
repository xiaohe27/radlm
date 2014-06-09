/*
 * Created on May, 2014
 * @author: Léonard Gérard leonard.gerard@sri.com
 */

#pragma once

#include "ros/ros.h"
#include "radl_flags.h"
#include <stdint.h> //TODO use cstdint when c++11 is used

namespace radl {

//// Represent nanoseconds
//typedef uint64_t duration_t;
//inline ros::Duration convert_t(duration_t t) {
//    return ros::Duration((uint32_t) (t>>32), (uint32_t) (t));
//};

template <typename msg_type, ros::Duration max_latency>
class Default_sub {

private:
    typedef typename msg_type::ConstPtr msg_ptr;
    static const ros::Duration timeout = max_latency;

    msg_ptr mailbox;
    flags_t flags;
    ros::Time reception_date;

public:
  Default_sub(const msg_ptr& init) {
    this->mailbox = init;
    this->flags = 0;
    //Wait for ros time to be synchronized
    ros::Time::waitForValid();
    //Set reception date to an invalid date
    this->reception_date = ros::Time();
  }

  void operator()(const msg_ptr& msg){
    this->mailbox = msg;
    this->flags = msg->radl__flags;
    this->reception_date = ros::Time::now();
  }

  flags_t flags(){
    flags_t f = this->flags;
    //Compute timeout
    if (timeout < (ros::Time::now() - this->reception_date)) {
        turn_on(TIMEOUT, f);
    }
    turn_on(STALE, this->flags); //for next time
    return f
  }

  msg_ptr value(){
    return this->mailbox;
  }
};


template <typename msg_type>
class Default_pub {

private:
  ros::Publisher* pub;

public:
  Default_pub(ros::Publisher& pub) {
    this->pub = &pub; 
  }
  void operator()(const msg_type& msg) {
    this->pub->publish(msg);
  }
};

}
