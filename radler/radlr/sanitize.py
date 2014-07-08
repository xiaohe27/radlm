'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.radlr.rast import AstVisitor, Ident, Alias

def _un_onleaf(visitor, leaf, namespace):
    """ Make sure Alias leaf are target for their name. """
    if isinstance(leaf, Alias):
        namespace.refresh(leaf._qname, leaf)
    return leaf, namespace

def _un_onnode(visitor, node, namespace):
    """ Make sure nodes are target for their name. """
    namespace.refresh(node._qname, node)
    visitor.mapacc(node._children, node._namespace)
    return node, namespace


updater_namespace = AstVisitor(default=_un_onnode, inplace=True)
""" Make sure namespace has correct name->node associations. """

def _ui_onleaf(visitor, leaf, namespace):
    """ Correct Ident->node and Alias->node associations. """
    if isinstance(leaf, Ident):
        leaf._reattach(namespace.lookup_node(leaf._qname))
    if isinstance(leaf, Alias):
        ident = leaf._is_alias_of
        ident._reattach(namespace.lookup_node(ident._qname))
    return leaf, namespace

def _ui_onnode(visitor, node, namespace):
    """ Keep track of the namespace. """
    visitor.mapacc(node._children, node._namespace)
    return node, namespace

updater_idents = AstVisitor(default=_ui_onnode, onleaf=_ui_onleaf, inplace=True)
""" Make sure Idents and Alias are correctly associated. """

def update_idents(ast, namespace):
    updater_namespace.visit(ast, namespace)
    updater_idents.visit(ast, namespace)
    return ast