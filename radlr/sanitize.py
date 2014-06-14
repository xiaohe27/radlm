'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from astutils.idents import Ident
from radlr.rast import AstVisitor


def _ui_onleaf(visitor, leaf, namespace):
    if isinstance(leaf, Ident):
        leaf._reattach(namespace.get_node(leaf._name))
    return leaf, namespace

def _ui_onnode(visitor, node, namespace):
    namespace.refresh(node._name, node)
    node._children, _ = visitor.mapacc(node._children, node._namespace)
    return node, namespace

updater_idents = AstVisitor(onleaf=_ui_onleaf, default=_ui_onnode)

def update_idents(ast, namespace):
    ast, _ = updater_idents.visit(ast, namespace)
    return ast