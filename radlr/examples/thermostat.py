'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


code="""
default_sub : subscriber {
    CXX { PATH "radl_lib.h" CLASS "Default_sub" }
}
default_pub : publisher {
    CXX { PATH "radl_lib.h" CLASS "Default_pub" }
}

basic_rate : int 50

house : node {
    SUBSCRIBES
        heater_rate { TOPIC heater_data SUBSCRIBER default_sub }
    PUBLISHES
        house_temp : publication { TOPIC house_data PUBLISHER default_pub }
    PERIOD basic_rate
    CXX { PATH "house.h" CLASS "House" }
}

house_data : topic {
    FIELDS
        temp : float32 75.0
}

thermometer : node {
    SUBSCRIBES
        house_temp { TOPIC house_data SUBSCRIBER default_sub }
    PUBLISHES
        thermometer_temp { TOPIC thermometer_data PUBLISHER default_pub }
    PERIOD 10
    CXX { PATH "thermometer.h" CLASS "Thermometer" }
}

thermometer_data : topic {
    FIELDS
        temp : float32 75
}


#who is publishing this ?
thermostat_button : topic {
    FIELDS
        status : bool true
}


#who is publishing this ?
thermostat_set : topic {
    FIELDS
        temp : float32 75
}


thermostat : node {
    SUBSCRIBES
        thermostat_switch { TOPIC thermostat_button SUBSCRIBER default_sub }
        thermometer_temp { TOPIC thermometer_data SUBSCRIBER default_sub }
    PUBLISHES
        heater_switch { TOPIC thermostat_data PUBLISHER default_pub }
    SUBSCRIBES
        thermostat_set_temp { TOPIC thermostat_set SUBSCRIBER default_sub }
    CXX
        { PATH "thermostat.h" CLASS "Thermostat" }
    PERIOD basic_rate
}

thermostat_data : topic {
    FIELDS
        switch_on : bool true
}

heater : node {
    SUBSCRIBES
        heater_switch { TOPIC thermostat_data SUBSCRIBER default_sub }
    PUBLISHES
        heater_rate { TOPIC heater_data PUBLISHER default_pub }
    PERIOD
        basic_rate
    CXX { PATH "heater.h" CLASS "Heater" }
}

heater_data : topic {
    FIELDS
        rate : float32 3
}

"""