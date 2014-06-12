'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radlr.rast import AstVisitor

def check_and_link_topics_publisher(ast):
    """ When there is one and only one publisher per topic, cross ref the publisher
    in the topic as topic._publisher"""
    def publication(visitor, pub, acc):
        """ Publication are not recursives """
        node, maping = acc
        top = pub['TOPIC']
        if top._name in maping:
            raise Exception("Topic {top} has multiple publisher.".format(top=top))
        maping[top._name] = node._ident
        return pub, (node, maping)
    def node(visitor, node, acc):
        """ set the current node in the accumulator """
        _, maping = acc
        acc = node, maping
        return visitor.node_mapacc(node, acc)
        

    visitor = AstVisitor(locals())
    ast, _ = visitor.visit(ast, (None, {}))
    return ast