'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Verify array values are coherent .

'''
from radler.radlr import types
from radler.radlr.rast import AstVisitor


def _tc_arrays(visitor, array, _):
    """Type Check arrays, simply call types.of() """
    types.of(array)
    return (), _

_tc_visitor = AstVisitor({'array' : _tc_arrays}, mapacc=True)

def typecheck(ast):
    _tc_visitor.visit(ast, ())