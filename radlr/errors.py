'''
Created on June, 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''

warning_as_errors = False
continue_when_errors = False

def _txt_format(message, location):
    return "{}{}".format(message, location)


def warning(message, location):
    txt = '\nWARNING:\n' + _txt_format(message, location)
    if warning_as_errors:
        raise Exception(txt)
    else:
        print(txt)

def error(message, location):
    txt = '\nERROR:\n' + _txt_format(message, location)
    if continue_when_errors:
        print(txt)
    else:
        raise Exception(txt)


def internal_error(message):
    raise Exception("\n=!= internal error =!=\n" + message)