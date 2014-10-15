
'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
import decimal
from radler.radlr.errors import internal_error, error


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

type duration
    REGEX ~r"(?P<value>(\b|[+-])\d+)(?P<unit>sec|msec|usec|nsec)"

type date
    REGEX ~r"(?P<value>\b\d+)(?P<unit>sec|msec|usec|nsec)"

type ip
    REGEX ~r"\b(?P<value>\d\d\d\.\d\d\d\.\d\d\d\.\d\d\d)\b"

class cxx_class
    PATH string ?
    HEADER string
    FILENAME string *
    LIB cmake_library/static_library *
    CLASS string

class cxx_file
    PATH string ?
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
           bool/struct/array/duration/date *
    EXTERNAL_ROS_DEF external_rosdef ?

class array
    TYPE string ?
    SIZE int16 ?
    VALUES int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array/duration/date*

class topic
    #Pay attention to the order of the types, parsing higher priority first
    FIELDS int8/uint8/int16/uint16/int32/uint32/int64/uint64/
           float32/float64/
           bool/struct/array/duration/date *
    EXTERNAL_ROS_DEF external_rosdef ?

class publication
    TOPIC topic
    PUBLISHER publisher
    MONITOR bool ?

class subscriber
    CXX cxx_class

class publisher
    CXX cxx_class

class subscription
    TOPIC topic
    SUBSCRIBER subscriber
    MAXLATENCY duration ?

class node
    PATH string ?
    PUBLISHES publication *
    SUBSCRIBES subscription *
    CXX cxx_class
    CXX_ANNEX cxx_file *
    PERIOD duration
    WCET duration ?
    DEVICES device_interface *

class device_interface
    HEADER string ?
    CXX cxx_file *
    NAME string

################
# Physical description
################

class processor
    NAME string
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


sized_types = {
    # sizes are expected to be increasing.
    'int'  : [8, 16, 32, 64],
    'uint' : [8, 16, 32, 64],
    'float': [32, 64],
}

def int_fits(value, size):
    #Python ints are arbitrary precision, so we can simply do this.
    return -2**(size-1) <= int(value) < 2**(size-1)

def uint_fits(value, size):
    #Python ints are arbitrary precision, so we can simply do this.
    return 0 <= int(value) < 2**size

# The decimal module doesn't seem to conform exactly to IEEE 754, but anyway,
# the conversion from a string isn't well specified in IEEE 754.
context_IEEE_754_float32 = decimal.Context(
    prec=24,
    rounding=decimal.ROUND_HALF_EVEN,
    Emin=-126,
    Emax=127,
    capitals=1, clamp=0, flags=[], traps=[]
)
context_IEEE_754_float64 = decimal.Context(
    prec=53,
    rounding=decimal.ROUND_HALF_EVEN,
    Emin=-1022,
    Emax=1023,
    capitals=1, clamp=0, flags=[], traps=[]
)

context_IEEE_754_float128 = decimal.Context(
    prec=113,
    rounding=decimal.ROUND_HALF_EVEN,
    Emin=-16382,
    Emax=16383,
    capitals=1, clamp=0, flags=[], traps=[]
)

def float_fits(value, size):
    d = decimal.Decimal(value)
    if size == 32:
        return d.normalize() == d.normalize(context=context_IEEE_754_float32)
    if size == 64:
        return d.normalize() == d.normalize(context=context_IEEE_754_float64)
    if size == 128:
        return d.normalize() == d.normalize(context=context_IEEE_754_float128)
    raise internal_error("Trying to fit a float of size {}".format(size))

check_type_size = {
    'int'   : int_fits,
    'uint'  : uint_fits,
    'float' : float_fits,
}

def wrap_fit_to_check(fun, t, size):
    def f(value, loc):
        try:
            if not fun(value, size):
                error("The value {} doesn't fit in the {}bits of {}."
                      "".format(str(value), size, t), loc)
        except ValueError:
            error("A value of type {} is expected."
                  "".format(t), loc)
    return f

check_type = {
    'int8'    : wrap_fit_to_check(int_fits, 'int8', 8),
    'int16'   : wrap_fit_to_check(int_fits, 'int16', 16),
    'int32'   : wrap_fit_to_check(int_fits, 'int32', 32),
    'int64'   : wrap_fit_to_check(int_fits, 'int64', 64),
    'uint8'   : wrap_fit_to_check(uint_fits, 'uint8', 8),
    'uint16'  : wrap_fit_to_check(uint_fits, 'uint16', 16),
    'uint32'  : wrap_fit_to_check(uint_fits, 'uint32', 32),
    'uint64'  : wrap_fit_to_check(uint_fits, 'uint64', 64),
    'float32' : wrap_fit_to_check(float_fits, 'float32', 32),
    'float64' : wrap_fit_to_check(float_fits, 'float64', 64),
    #TODO: 5 complete for the other types in need of checking.
}


type_mapping = {
    'int8'    : {'cxx': 'std::int8_t'     , 'ROS': 'int8'   },
    'int16'   : {'cxx': 'std:int16_t'     , 'ROS': 'int16'  },
    'int32'   : {'cxx': 'std::int32_t'    , 'ROS': 'int32'  },
    'int64'   : {'cxx': 'std::int64_t'    , 'ROS': 'int64'  },
    'uint8'   : {'cxx': 'std::uint8_t'    , 'ROS': 'uint8'  },
    'uint16'  : {'cxx': 'std::uint16_t'   , 'ROS': 'uint16' },
    'uint32'  : {'cxx': 'std::uint32_t'   , 'ROS': 'uint32' },
    'uint64'  : {'cxx': 'std::uint64_t'   , 'ROS': 'uint64' },
    'float32' : {'cxx': 'std::float32_t'  , 'ROS': 'float32'},
    'float64' : {'cxx': 'std::float64_t'  , 'ROS': 'float64'},
    'bool'    : {'cxx': 'std::bool'       , 'ROS': 'bool'   },
    'duration': {'cxx': 'radl::duration_t', 'ROS': 'int64'  },
    'date'    : {'cxx': 'radl::duration_t', 'ROS': 'int64'  },
}

def main_unit(v): return v
def sec2nsec(sec):   return str(int(sec) *1000000000)
def msec2nsec(msec): return str(int(msec)*1000000)
def usec2nsec(usec): return str(int(usec)*1000)

unit_normalize = {
    'duration': {'nsec': main_unit, 'sec' : sec2nsec,
                 'msec': msec2nsec, 'usec': usec2nsec},
    'date'    : {'nsec': main_unit, 'sec' : sec2nsec,
                 'msec': msec2nsec, 'usec': usec2nsec},
    }

#TODO: 5 what is needed to decide if marshalling is necessary? see processor
#TODO: 6 change FILENAME to SRC
#TODO: 7 allow subtyping with alias x : int8 0  y : int16 = x
#######################################
#TODO: 0 RADL
# min and max period
# generate firewall rules
# generate alert messages to monitors
# check connectivity
# deploy and run scripts
# char type? string? how to have fixed size, platform independant? uint8 array ?
# maybe allow ident to begin with _
#######################################
#TODO: 8 support string calling code (note the order has to be respected)
#    PATH string ? 'src'
#    FILENAME string ? @ {this}._name @
#    CLASS string ? @ str.capitalize({this}['FILENAME']) @
#######################################
#TODO: 3 big issue in the publishers (ROS require data not to be overwriten), spinonce flush outputs?
# We should abstract and require a two step publishing:
# 1) msg* get_slot() 2) void publish()
# Or use C++11 move semantics publish(msg&&), but it is probably not wanted.
#######################################
