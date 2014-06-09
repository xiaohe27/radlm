'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


#TODO: 6 Where do the error message is updated (it seems it is not with _end)


#TODO: 3 In CXX, std::int8_t, etc require <cstdint>

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
    REGEX ~r"(?P<value>\d+)"
    CXX "std::int8_t"

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

type msec
    REGEX ~r"(?P<value>\d+)"
    CXX "std::int32_t"

class cxx_class
    PATH string
    FILENAME string
    CLASS string

#TODO: 8 support string calling code (note the order has to be respected)
#    PATH string ? 'src'
#    FILENAME string ? @ {this}._name @
#    CLASS string ? @ str.capitalize({this}['FILENAME']) @

class topic
    FIELDS int/bool/string/float32 *
#    PACKED bool

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
#TODO: 5 support '?' modifier in the meta grammar and default values.
    MAXLATENCY msec

class node
    PUBLISHES publication *
    SUBSCRIBES subscription *
    CXX cxx_class
    PERIOD msec

"""