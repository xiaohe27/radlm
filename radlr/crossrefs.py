'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radlr.rast import AstVisitor
from astutils.idents import Ident
from radlr.errors import warning

def _check_and_map_topics_publisher(ast):
    """ When there is one and only one publisher per topic, cross ref the publisher
    in the topic as topic._publisher"""
    def publication(visitor, pub, acc):
        """ Publication are not recursives """
        node, maping = acc
        top = pub['TOPIC']
        if top._name in maping:
            raise Exception("Topic {top} has multiple publisher.".format(top=top))
        maping[top._name] = Ident.of(node)
        return pub, (node, maping)
    def node(visitor, node, acc):
        """ set the current node in the accumulator """
        _, maping = acc
        acc = node, maping
        return visitor.node_mapacc(node, acc)

    visitor = AstVisitor(locals(), inplace=True)
    _, (_, maping) = visitor.visit(ast, (None, {}))
    return maping

def _link(ast, maping):
    def topic(visitor, topic, maping):
        try:
            i = maping[topic._name]
            topic._publisher = i
        except KeyError:
            warning("The topic {} doesn't have any publisher.".format(
                        topic._name), topic._location)
        return topic, maping
    AstVisitor({'topic' : topic}, inplace=True).visit(ast, maping)

def add(ast):
    maping = _check_and_map_topics_publisher(ast)
    _link(ast, maping)