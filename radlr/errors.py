'''
Created on June, 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''

warning_as_errors = False
continue_when_errors = False

def _txt_format(message, location):
    return "{}{}".format(location, message)


def warning(message, location):
    txt = _txt_format(message, location)
    if warning_as_errors:
        raise Exception(txt)
    else:
        print(txt)

def error(message, location):
    txt = _txt_format(message, location)
    if continue_when_errors:
        print(txt)
    else:
        raise Exception(txt)


def internal_error(message):
    raise Exception("=!= internal error =!=\n" + message)