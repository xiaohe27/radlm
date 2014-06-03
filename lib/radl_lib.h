#include "ros/ros.h"

template <typename msg_type> 
class Default_sub {

private:
  typedef typename msg_type::ConstPtr msg_ptr;
  msg_ptr mailbox;

public:
  Default_sub(const msg_ptr& init) {
    this->mailbox = init;
  }

  void operator()(const msg_ptr& msg){    
    this->mailbox = msg;
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
