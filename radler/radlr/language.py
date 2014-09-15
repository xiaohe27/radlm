'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


version = 'RADL 0.92'

extra_keywords = {
#C++ keywords
'alignas', 'alignof', 'and', 'and_eq', 'asm', 'auto', 'bitand', 'bitor',
'bool', 'break', 'case', 'catch', 'char', 'char16_t', 'char32_t', 'class',
'compl', 'const', 'constexpr', 'const_cast', 'continue', 'decltype', 'default',
'delete', 'do', 'double', 'dynamic_cast', 'else', 'enum', 'explicit', 'export',
'extern', 'false', 'float', 'for', 'friend', 'goto', 'if', 'inline', 'int',
'long', 'mutable', 'namespace', 'new', 'noexcept', 'not', 'not_eq', 'nullptr',
'operator', 'or', 'or_eq', 'private', 'protected', 'public', 'register',
'reinterpret_cast', 'return', 'short', 'signed', 'sizeof', 'static',
'static_assert', 'static_cast', 'struct', 'switch', 'template', 'this',
'thread_local', 'throw', 'true', 'try', 'typedef', 'typeid', 'typename',
'union', 'unsigned', 'using', 'virtual', 'void', 'volatile', 'wchar_t',
'while', 'xor', 'xor_eq'
}

forbidden_prefix = "radl__"

defs = r"""

#TODO: 6 size checks
type int8
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint8
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int16
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint16
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int32
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint32
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type int64
    REGEX ~r"(?P<value>(\b|[+-])\d+)\b(?!\.)"

type uint64
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"

type float32
    REGEX ~r"(?P<value>(\b|[+-])(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"

type float64
    REGEX ~r"(?P<value>(\b|[+-])(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"

type bool
    REGEX ~r"\b(?P<value>true|false)"

type string
    REGEX ~r'"(?P<value>[^"]*)"'

type msec
    REGEX ~r"\b(?P<value>\d+)"

type ip
    REGEX ~r"\b(?P<value>\d\d\d\.\d\d\d\.\d\d\d\.\d\d\d)\b"

class cxx_class
    PATH string ?
    HEADER string
    #TODO: 6 change FILENAME to SRC
    FILENAME string *
    LIB cmake_library/static_library *
    CLASS string

class cxx_file
    PATH string ?
    #TODO: 6 change FILENAME to SRC
    FILENAME string *
    LIB cmake_library/static_library *

class cmake_library
    PATH string ?
    CMAKE_MODULE string
    COMPONENTS string *

class static_library
    PATH string ?
    HEADER_PATHS string *
    CXX cxx_file *

class external_rosdef
    PATH string ?
    FULLNAME string
    HEADER string ?

class struct
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array *
    EXTERNAL_ROS_DEF external_rosdef ?

class array
    VALUES int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array *

class topic
    #Pay attention to the order of the types, parsing higher priority first
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array *
    EXTERNAL_ROS_DEF external_rosdef ?

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
    MAXLATENCY msec ?

class node
    PATH string ?
    PUBLISHES publication *
    SUBSCRIBES subscription *
    CXX cxx_class
    CXX_ANNEX cxx_file *
    PERIOD msec
    WCET msec ?
    DEVICES device_interface *

class device_interface
    HEADER cxx_file ?
    NAME string

################
# Physical description
################

class processor
    NAME string
    #TODO: 5 what is needed to decide if marshalling is necessary?
    BITS int8
    ENDIANESS string

class device
    IMPLEMENTS device_interface
    REQUIRES_LINUX bool
    CXX cxx_file *

class bus
    ENDPOINTS processor/device *


################
# Mapping
################

class hypervisor
    PARTITIONS partition *

class os
    LINUX bool
    REALTIME bool

class partition
    OS os ?
    PROC processor
    IP ip ?
    RUNS mapped_node *

class mapped_node
    NODE node
    IP ip ?
    PORT uint16 ?

"""

#######################################
#TODO: 0 RADL
# min and max period
# generate firewall rules
# generate alert messages to monitors
# check connectivity
# deploy and run scripts
# time type to provide and put in messages (useful for users to timestamp) Issue with the ROS time is that seconds are uint32, how does one get a full date?
# char type? string? how to have fixed size, platform independant? uint8 array ?
# maybe allow ident to begin with _
# array issue with repeating the type.... subtyping... (cast in generated code?)
#######################################
#TODO: 8 support string calling code (note the order has to be respected)
#    PATH string ? 'src'
#    FILENAME string ? @ {this}._name @
#    CLASS string ? @ str.capitalize({this}['FILENAME']) @
#######################################
# TODO: 3 big issue in the publishers (ROS require data not to be overwriten)
# We should abstract and require a two step publishing:
# 1) msg* get_slot() 2) void publish()
# Or use C++11 move semantics publish(msg&&), but it is probably not wanted.
#######################################