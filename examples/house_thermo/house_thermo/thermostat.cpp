
#include "thermostat.h"

namespace house_thermo {

Thermostat::Thermostat() {
  this->set_temp = 75.0;
  this->status = false;
  this->tol = 2.0;
}

void Thermostat::step(_in_thermostat * inmsgs, _flags_thermostat* flags, _out_thermostat * outmsgs) {
  // change the set temperature 
  this->set_temp = inmsgs->thermostat_set_temp->temp;

  // set the status
  this->status = inmsgs->thermostat_switch->status;

  // decide whether to switch on the heater
  if (inmsgs->thermometer_temp->temp > (this->set_temp + this->tol)) {
    outmsgs->heater_switch->switch_on = false;
  } else if (this->status && (inmsgs->thermometer_temp->temp < this->set_temp)) {
    outmsgs->heater_switch->switch_on = true;
  } else {
    outmsgs->heater_switch->switch_on = this->status;
  } 
}

}
