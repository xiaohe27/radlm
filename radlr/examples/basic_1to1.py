'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

"""
This example the simplest communication between two nodes.
n_a publish the empty topic messages to n_b



Remarks:
Field name and type are optional.
Names should be automatically filled with non colliding names.
"""

code="""

#TODO: 4 move to pervasives
pub_step_sync : publisher {
    CXX
        cxx_class { PATH "pervasives.h" CLASS "pub_step_sync" }
}

sub_step_sync : subscriber {
    CXX
        a : cxx_class { PATH "pervasives.h" CLASS "sub_step_sync" }
}

#### Actual example

empty : topic {}

n_a : node {
    PUBLISHES
        out1 : publication { TOPIC empty PUBLISHER pub_step_sync }
    CXX
        { PATH "basic_1to1.c" CLASS "stepa" }
    RATE 50
}

n_b : node {
    SUBSCRIBED
        in1 { TOPIC empty SUBSCRIBER sub_step_sync }
    CXX
        { PATH "basic_1to1.c" CLASS "stepb" }
    RATE 50
}

"""