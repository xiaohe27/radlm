'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


#TODO: 6 Where do the error message is updated (it seems it is not with _end)


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
    REGEX ~r"\b(?P<value>[+-]?\d+)\b(?!\.)"
    CXX "std::int8_t"

type uint8
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"
    CXX "std::uint8_t"

type int16
    REGEX ~r"\b(?P<value>[+-]?\d+)\b(?!\.)"
    CXX "std::int16_t"

type uint16
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"
    CXX "std::uint16_t"

type int32
    REGEX ~r"\b(?P<value>[+-]?\d+)\b(?!\.)"
    CXX "std::int32_t"

type uint32
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"
    CXX "std::uint32_t"

type int64
    REGEX ~r"\b(?P<value>[+-]?\d+)\b(?!\.)"
    CXX "std::int64_t"

type uint64
    REGEX ~r"\b(?P<value>\d+)\b(?!\.)"
    CXX "std::uint64_t"

type float32
    REGEX ~r"\b(?P<value>[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"
    CXX "std::float32_t"

type float64
    REGEX ~r"\b(?P<value>[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\b(?!\.)"
    CXX "std::float64_t"

type bool
    REGEX ~r"\b(?P<value>true|false)"
    CXX "std::bool"

type string
    REGEX ~r'"(?P<value>[^"]*)"'
    CXX "std::string"

type msec
    REGEX ~r"\b(?P<value>\d+)"
    CXX "std::int32_t"


class cxx_class
    PATH string ?
    HEADER string
    FILENAME string *
    CLASS string
#TODO: 8 support string calling code (note the order has to be respected)
#    PATH string ? 'src'
#    FILENAME string ? @ {this}._name @
#    CLASS string ? @ str.capitalize({this}['FILENAME']) @

class cxx_file
    PATH string ?
    FILENAME string *

#TODO: 5 support libraries
# class library
#     CXX cxx_file *
# ->
# add_library(<name> [STATIC | SHARED | MODULE]
#               [EXCLUDE_FROM_ALL]
#               source1 source2 ... sourceN)
#class external_library
#     INCLUDE_PATH string
#     FILENAME string

class struct
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/string/struct/array *
    EXTERNAL_ROS_DEF string ?


class array
    VALUES int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/string/struct/array *


class topic
#Pay attention to the order of the types, parsing higher priority first
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/string/struct/array *
    EXTERNAL_ROS_DEF string ?


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
    DEVICES device *

################
# Physical description
################

class device
    CXX cxx_file *


"""

#######################################
#TODO: 0 RADL
# generate firewall rules
# generate alert messages to monitors
# check connectivity
# deploy and run scripts
#######################################