'''
Created on Apr 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

import re

from pypeg2 import Symbol, attr, K, optional, maybe_some, word, parse


# Symbols should be distinct from Keywords
Symbol.check_keywords = True


##############################################################################
# Utils to construct a regular grammar

def mk_entry(Sname = "name", Cname=Symbol,
             Sdesc = "desc", Cdesc=Symbol,
             Cparams={}):
    """mk_entry("x", X, "t", T, { "P1" : Cp1, "P2" : Cp2 })
    returns a grammar class which will parse an entry of the form:
    [X] : [T] {
        P1 [Cp1]
        P2 [Cp2]
    }
    with [X] a value parsed by X, etc.
    The value of [X] is stored in the attribute x, [T] in t, [Cp1] in P1, etc.
    """
    params_attribute = [(K(k), attr(k, Cparams[k])) for k in Cparams.keys()]
    class Entry:
        grammar = (attr(Sname, Cname),
                   optional(':', attr(Sdesc, Cdesc)),
                   optional('{', maybe_some(params_attribute), '}'))
    return Entry

def mk_entry_list(*args, **kargs):
    """returns a grammar class parsing a list of entries made with args, kargs
    """
    class EntryList(list):
        grammar = maybe_some(mk_entry(args, kargs))
    return EntryList

def mk_ref(Cvalue=mk_entry(), Cref=Symbol):
    class Ref:
        grammar = [Cref, Cvalue]
    return Ref

def mk_ref_list(*args, **kargs):
    class RefList(list):
        grammar = maybe_some(mk_ref(args, kargs))
    return RefList

def mk_OR(*C):
    class OR:
        grammar = list(C)
    return OR

class InlineValue():
    """ Enable to quote arbitrary code:
        'word
    or
        '<key> arbitrary code with line feeds, etc <key>
    The short version allows only a word.
    The long version allows arbitrary code by using user specified key.

    Instance attributes:
        value:
             gives the quoted value without the quote markers
             (' for the short and '<key> <key> for the long)
        key:
            is the key used to quote, None for short quotes.
    """

    short = word
    long = re.compile(r"(?ms)<(?P<key>.*?)>(.*?)<(?P=key)>")

    def __init__(self, value, key=None):
        self.value = value
        self.key = key

    def __repr__(self):
        """Pythonic representation"""
        if self.key == None:
            k = 'None'
        else:
            k = repr(self.key)
        return "InlineValue(" + repr(self.value) + ", " + k + ")"

    def __str__(self):
        """Parsable representation"""
        if self.key == None:
            return "'" + str(self.value)
        else:
            k = "<" + str(self.key) + ">"
            return "'" + k + str(self.value) + k

    def __eq__(self, other):
        if type(self) != type(other):
            return False
        return self.value == other.value

    @classmethod
    def parse(cls, parser, text, pos):
        if text[0] == "'":
            m = cls.long.match(text[1:])
            if m:
                t, r = text[len(m.group(0))+1:], m.group(2)
                parser._got_regex = True #TODO Really needed?
                return t, InlineValue(r, key=m.group(1))
            else:
                m = cls.short.match(text[1:])
                if m:
                    t, r = text[len(m.group(0))+1:], m.group(0)
                    parser._got_regex = True #TODO Really needed?
                    return t, InlineValue(r)
        return text, SyntaxError("expecting a quoted external value"
                                 " like '42 or '<key>f(x) { ... }<key>")

#bug, compose is a method not a class method..
    def compose(self, parser, attr_of):
        return str(self)


class String(str):
    grammar = re.compile(r"(?ms)\".*?\"")

##############################################################################
# Generics
Package = mk_entry()
Packages = mk_entry_list()

ExternalValue = mk_entry(
    Cparams = {"FUN"         : InlineValue,
               "VAL"         : InlineValue})

Code = mk_OR(ExternalValue, InlineValue)

##############################################################################
# Node

State = mk_entry_list(
    Cparams = {"INIT"        : InlineValue})

Publishes = mk_entry_list()

Subscribed = mk_entry_list(
    Cparams = {"MAXLATENCY"  : int})

Node = mk_entry(Cdesc = K("NODE"),
    Cparams = {"PACKAGES"    : Packages,
               "STATE"       : State,
               "PUBLISHES"   : Publishes,
               "SUBSCRIBES"  : Subscribed,
               "INIT"        : Code,
               "STEP"        : Code,
               "PERIOD"        : int})

##############################################################################
# Topic

TopicFields = mk_entry_list()

Topic = mk_entry(Cdesc = K("TOPIC"),
    Cparams = {"PACKAGES"    : Packages,
               "FIELDS"      : TopicFields,
#                "TYPE"        : InlineValue,
               "INIT"        : Code,
               "MAXPUB"      : int, #Max
               "PERIOD"        : int})

##############################################################################
# Hardware layer referencing the logical layer

Partition = mk_entry(Cdesc = K("PARTITION"),
    Cparams = {"OS"          : String,
               "PACKAGES"    : Packages,
               "NODES"       : mk_ref_list(Node)})

Processor = mk_entry(Cdesc = K("PROCESSOR"),
    Cparams = {"HYPERVISOR"  : String,
               "PARTITIONS"  : mk_ref_list(Partition)})

Device = mk_entry(Cdesc = K("DEVICE"),
    Cparams = {"DRIVER"      : Package})

Bus = mk_entry(Cdesc = K("BUS"),
    Cparams = {"KIND"        : String,
               "PARTITIONS"  : mk_ref_list(Partition),
               "DEVICES"     : mk_ref_list(Device)})




