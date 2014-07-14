'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Add a _pwd attribute to every nodes qualifying the current user path.
This path is a Path object from pathlib.
It may be a relative or an absolute path, following the user input.
'''

from radler.radlr.rast import AstVisitor
from pathlib import Path


def _pwd(visitor, node, pwd):
    p = node.get('PATH', None)
    subdir = p._val if p else ''
    pwd = pwd / subdir
    node._pwd = pwd
    return visitor.node_mapacc(node, pwd)

def add(ast):
    """ Add a _pwd attribute to nodes indicating current user path."""
    visitor = AstVisitor(default=_pwd, mapacc=True)
    path = Path()
    visitor.visit(ast, path)
