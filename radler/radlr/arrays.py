'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Verify array values are coherent with their type.

'''
from radlr import types
from radlr.errors import error
from radlr.rast import AstVisitor


def _tc_arrays(visitor, array, _):
    atype = types.of(array)
    expected_elem_type = atype['ELEM_TYPE']
    expected_size = int(atype['SIZE']._val)
    values = array['VALUES']
    #check length
    if len(values) != expected_size:
        msg = ("Given array is of size {} when {} is expected."
               "".format(len(values), expected_size))
        error(msg, array._location)
    #check elements' types
    for e in values:
        te = types.of(e)
        if not types.are_eq(te, expected_elem_type):
            msg = ("This array element has datatype {} when {} is expected."
                   "".format(types.to_str(te), types.to_str(expected_elem_type)))
            error(msg, e._location)
    return visitor.node_mapacc(array, _) #recursive checks

_tc_visitor = AstVisitor({'array' : _tc_arrays}, mapacc=True)

def typecheck(ast):
    _tc_visitor.visit(ast, ())