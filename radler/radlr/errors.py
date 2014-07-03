'''
Created on June, 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''
from collections import Callable

warning_as_errors = False
continue_when_errors = False
verbosity_level = 0

def _logn(n):
    def __logn(message):
        if verbosity_level >= n:
            if isinstance(message, Callable):
                message = message()
            print(message)
    return __logn

log_err   = _logn(-1)
log_warn  = _logn(0)
log1  = _logn(1)
log2  = _logn(2)
log3  = _logn(3)

#TODO: 5 base class for user errors, internal errors, etc to be able to catch them

#TODO: 8 use the inflect package to get plurals, etc correctly.

def _txt_format(message, location):
    return "{}{}".format(location, message)

#verbosity level = 0
def warning(message, location):
    log_warn('\nWARNING ' + _txt_format(message, location))
    if warning_as_errors:
        exit(-1)

#verbosity level = -1
def error(message, location):
    log_err('\nERROR ' + _txt_format(message, location))
    if not continue_when_errors:
        exit(-1)

def internal_error(message):
    raise Exception("\n=!= internal error =!=\n" + message)

def internal_assert(v, message):
    if not v:
        raise Exception("\n=!= internal error =!=\n" + message)