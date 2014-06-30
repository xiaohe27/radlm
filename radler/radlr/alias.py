'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.astutils.idents import Ident
from radler.radlr.errors import internal_assert
from radler.radlr.rast import AstVisitor


class AliasLeaf(Ident):
    __slots__ = ['_name', '_generated', '_location', '_is_alias_of']
    #duplicate the 
    def __init__(self, name, generated, location, target_ident):
        self._name = name
        self._generated = generated
        self._location = location
        self._is_alias_of = target_ident
    @property
    def _node(self):
        a = self._is_alias_of
        return a._node


def make_transparent(ast):
    """ Transform alias nodes into leafs of type AliasLeaf,
    behaving like the Ident it points to.
    This has to be done with a frozen ast or extra care is needed after this.
    """
    def _alias(visitor, node, _):
        internal_assert(len(node._children)==1, "incorrect alias node")
        ident = node._children[0]
        internal_assert(isinstance(ident, Ident), "incorrect alias node")
        a = AliasLeaf(node._name, ident._generated, node._location, ident)
        return a, _
    visitor = AstVisitor({'_alias': _alias}, inplace=True)
    visitor.visit(ast, ())
