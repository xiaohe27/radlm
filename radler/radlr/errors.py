'''
Created on June, 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''
from collections import Callable
from radler.astutils.location import no_loc

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

#TODO: 8 use the inflect package to get plurals, etc correctly.

def _txt_format(message, location):
    if location:
        return "{}{}".format(location, message)
    else:
        return message

def warning(message, location):
    """ Output a user warning (verbosity level of 0),
    then exit(-1) if warning_as_errors is set.)
    """
    log_warn('\nWARNING ' + _txt_format(message, location))
    if warning_as_errors:
        exit(-1)

def error(message, location):
    """Output a user error (verbosity level of -1),
    then exit(-1) unless continue_when_errors is set.
    """
    log_err('\nERROR ' + _txt_format(message, location))
    if not continue_when_errors:
        exit(-1)

def error_noloc(message):
    """Same as error but without any provided location."""
    error(message, no_loc)

def internal_error(message):
    """ Error message for internal errors, ideally this should never happens.
    """
    raise Exception("\n=!= internal error =!=\n" + message)

def internal_assert(v, message):
    if not v: internal_error(message)
