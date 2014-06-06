#include "ros/ros.h"

/*
 * Flags handling
 *
 * Use an uint instead of a bitfield to be compatible.
 */

typedef flag_t std::uint_8;

// Functionnal flags are the first 4 bits
const flag_t FUNCTIONAL_FLAG = 15;
const flag_t STALE = 1;

// Failure flags are the others
const flag_t FAILURE_FLAGS = ~FUNCTIONAL_FLAG;
const flag_t TIMEOUT = 16;

// Handy user shortcuts
inline bool is_stale(flag_t f) { return f & STALE; };
inline bool is_timeout(flag_t f) { return f & TIMEOUT; };
inline bool is_failing(flag_t f) { return f & ERROR_FLAGS; };

inline void turn_on(flag_t f, flag_t & x) { x |= f; };
inline void turn_off(flag_t f, flag_t & x) { x &= ~f; };


// Represent nanoseconds
typedef duration_t std::uint_64;
inline ros::Duration convert_t(duration_t t) {
    return ros::Duration((std::uint_32) (t>>32), (std::uint_32) (t));
};


template <typename msg_type, uint64 pub_period, uint64 sub_period, uint64 max_latency>
class Default_sub {

private:
    typedef typename msg_type::ConstPtr msg_ptr;
    static const ros::Duration timeout = convert_t(pub_period + max_latency);

    msg_ptr mailbox;
    flag_t flags;
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
    this->flags = msg->_flags;
    this->reception_date = ros::Time::now();
  }

  flag_t flags(){
    flag_t f = this->flags;
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
