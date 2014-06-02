'''
Created on May 4, 2014

@author: lgerard
'''
from nose.tools import eq_

from parsimonious.grammar import Grammar
from parsimonious.nodeutils import clean_node
from parsimonious.nodes import Node

def test_clean_node():
    """ Assert cleaning is not in place, is correct and handle its own result.
    """
    g = """ A = ('a' / B / C / '')*
                B = 'b'+
                C = 'c'*
            """
    gg = Grammar(g, 'A')
    s = 'ab'
    test1 = gg.parse(s)
    t1 = clean_node(test1)
    eq_(t1, Node('A', s, 0, 2, children=
                 [Node('__OneOf__', s, 0, 1, children=
                       [Node('__Literal__', s, 0, 1)]),
                        Node('__OneOf__', s, 1, 2, children=
                             [Node('B', s, 1, 2, children=
                                   [Node('__Literal__', s, 1, 2)])])]))
    t2 = clean_node(t1)
    eq_(t1, t2)
    tt1 = clean_node(test1, basic_lift=True)
    eq_(tt1, Node('A', s, 0, 2, ['a', Node('B', s, 1, 2, ['b'])]))
    tt2 = clean_node(tt1, basic_lift=True)
    eq_(tt1, tt2)

def test_mapreduce_node():
    """ Assert mapreduce is not in place but is by default the identity
    """
    g = """ A = ('a' / B / C / '')*
                B = 'b'+
                C = 'c'*
            """
    gg = Grammar(g, 'A')
    s = 'ab'
    test1 = gg.parse(s)
    test2, acc = mapreduce_node(test1, acc=42)
    eq_(acc, 42)
    eq_(test1, test2)
    if id(test1) == id(test2):
        raise AssertionError("mapreduce is in place!")