'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.radlr.errors import internal_assert
from radler.radlr.rast import AstVisitor, Ident, Alias
from radler.radlr.sanitize import update_idents


def make_transparent(ast, root_namespace):
    """ Transform alias nodes into leafs of type Alias,
    behaving like the Ident it points to.
    This has to be done with a frozen ast or extra care is needed after this.
    """
    def _alias(visitor, node, _):
        internal_assert(len(node._children)==1, "incorrect alias node")
        ident = node._children[0]
        internal_assert(isinstance(ident, Ident), "incorrect alias node")
        a = Alias(node._qname, node._location, ident)
        return a, _
    visitor = AstVisitor({'_alias': _alias}, inplace=True)
    visitor.visit(ast, ())
    update_idents(ast, root_namespace)
