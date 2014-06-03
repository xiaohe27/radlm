'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


#TODO: 6 Where do the error message is updated (it seems it is not with _end)


#TODO: 3 In CXX, std::int8_t, etc require <cstdint>


#TODO: 3 Add CXX keywords... (issues when generating code)


radlr_language = r"""

# type int8
#     REGEX ~r"(?P<value>\d+)"
#     CXX "std::int8_t"

type float32
    REGEX ~r"(?P<value>[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)"
    CXX "std::float32_t"

type int
    REGEX ~r"(?P<value>\d+)"
    CXX "std::int32_t"

type bool
    REGEX ~r"(?P<value>true|false)"
    CXX "std::bool"

type string
    REGEX ~r'"(?P<value>[^"]*)"'
    CXX "std::string"

class cxx_class
    PATH string
    CLASS string

#TODO: 6 support '?' modifier in the meta grammar
#TODO: 8 support string calling code (not the order has to be respected
#    PATH string ? 'src'
#    FILENAME string ? @ {this}._name @
#    CLASS string ? @ str.capitalize({this}['FILENAME']) @

class topic
    FIELDS int/bool/string/float32 *

class publication
    TOPIC topic
    PUBLISHER publisher

class subscriber
    CXX cxx_class

class publisher
    CXX cxx_class

class subscription
    TOPIC topic
    SUBSCRIBER subscriber

class node
    PUBLISHES publication *
    SUBSCRIBED subscription *
    CXX cxx_class
    RATE int



"""