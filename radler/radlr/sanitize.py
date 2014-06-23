'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.radlr.rast import AstVisitor, Ident
from radler.radlr.alias import Alias


def _un_onnode(visitor, node, namespace):
    namespace.refresh(node._qname, node)
    visitor.mapacc(node._children, node._namespace)
    return node, namespace

updater_namespace = AstVisitor(default=_un_onnode, inplace=True)


def _ui_onleaf(visitor, leaf, namespace):
    if isinstance(leaf, Ident):
        leaf._reattach(namespace.lookup_node(leaf._qname))
    if isinstance(leaf, Alias):
        leaf[0]._reattach(namespace.lookup_node(leaf[0]._qname))
    return leaf, namespace

def _ui_onnode(visitor, node, namespace):
    visitor.mapacc(node._children, node._namespace)
    return node, namespace

updater_idents = AstVisitor(default=_ui_onnode, onleaf=_ui_onleaf, inplace=True)


def update_idents(ast, namespace):
    updater_namespace.visit(ast, namespace)
    updater_idents.visit(ast, namespace)
    return ast