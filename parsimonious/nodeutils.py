'''
Created on May 2014

@author: Léonard Gérard leonard.gerard@sri.com

'''

from parsimonious.nodes import Node
from astutils.nodetrees import Functor
from copy import copy

nodetreesutils = Functor(Node, 'children', 'expr_name')
ParseVisitor = nodetreesutils.NodeTreeVisitor
pprint_node = nodetreesutils.pprint_node


def prune(node, to_prune=[]):
    """Remove to_prune subnodes in a parsimonious.Node.
        Nodes boundaries are updated according to pruned subnodes.
        (Tries to be smart, but may be quadratic...)
        If the root is pruned, return None.
    """
    if node.expr_name in to_prune:
        return None
    #Note, the mapred accumulator stores the computed boundaries as a tuple
    def prune(visitor, l, _):
        # Filter first, BEFORE visiting
        # -> which prevents lifting in the same pass.
        def isto_keep(x):
            if not isinstance(x, Node):
                return True
            else:
                return x.expr_name not in to_prune
        l = list(filter(isto_keep, l))
        return visitor.list_mapred(l, _)

    def default(visitor, n, _):
        #store the current positions
        nb = [n.start, n.end]
        cb = [visitor.left_node(n).start,
              visitor.right_node(n).end]
        children, _ = visitor.visit(n.children) #depth first
        if n.children and nb[0] == cb[0]:
            #we believe the boundary comes from the left children: update it.
            if children and n.children[0] != children[0]: #update only when necessary
                nb[0] = visitor.left_node(n).start
        if n.children and nb[1] == cb[1]:
            if children:
                if n.children[-1] != children[-1]:
                    nb[1] = visitor.right_node(n).start
            else: # lost children giving position, empty the node:
                nb[1] = nb[0]
        node = copy(n)
        node.start = nb[0]
        node.end = nb[1]
        node.children = children
        return node, _

    node, _ = ParseVisitor(default=default, onlist=prune).visit(node)
    return node

def lift(node):
    """lift as follow:
            exp* (ZeroOrMore) becomes a list
            exp+ (OneOrMore) becomes a list
            exp exp (Sequence) becomes a list
            exp / exp (OneOf) is replaced by its only child
    """
    lift = ParseVisitor.lift_mapacc
    lift_the_child = ParseVisitor.left_mapacc
    d = {'__ZeroOrMore__' : lift, '__OneOrMore__' : lift,
         '__Sequence__' : lift, '__OneOf__' : lift_the_child}
    node, _ = ParseVisitor(d).visit(node)
    return node


def eval_leafs(node, txt_leaf=[], keep_regex=False):
    """ any node in txt_leaf is replaced by str of itself,
        'exp' (Literal) becomes the value
        ~regex (Regex) becomes the matched string
        exp? (Optional) becomes None or the value
    """
    def textify(visitor, node, _):
        return node.text, _
    def optional(visitor, node, _):
        children, _ = visitor.visit(node.children) #depth first
        l = len(children)
        if l==0:
            return None, _
        elif l==1:
            return children[0], _
        else:
            visitor.error("Optional leaf with multiple children.")

    d = dict()
    if keep_regex:
        basic_txt_leaf = ['__Literal__']
    else:
        basic_txt_leaf = ['__Literal__', '__Regex__']
    for n in basic_txt_leaf + txt_leaf:
        d[n] = textify
    d['__Optional__'] = optional
    node, _ = ParseVisitor(d).visit(node)
    return node


def clean_node(node, to_prune=[], txt_leaf=[], keep_regex=False):
    """prune, lift and eval leafs
    """
    return eval_leafs(lift(prune(node, to_prune)), txt_leaf, keep_regex)

