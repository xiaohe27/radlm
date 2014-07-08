'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Add a _pwd attribute to every nodes qualifying the current path.
When a relative path is used, 
'''

from radler.radlr.rast import AstVisitor
from radler.radlr.ros import utils


def _pwd(visitor, node, pwd):
    p = node.get('PATH', None)
    subdir = p._val if p else ''
    pwd = pwd / subdir
    node._pwd = pwd
    return visitor.node_mapacc(node, pwd)

def add(ast):
    """ Add a _pwd attribute to nodes indicating current user path."""
    visitor = AstVisitor(default=_pwd, mapacc=True)
    path = utils.user_filepath(ast._qname)
    visitor.visit(ast, path)
