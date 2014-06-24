'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Add a _pwd attribute to every nodes qualifying the current path.
When a relative path is used, 
'''

from radlr.rast import AstVisitor

def _pwd(visitor, node, pwd):
    try:
        #TODO 5: PATH should be a ? not a *
        subdir = node['PATH'][0]._val
    except (KeyError, #for nodes without 'PATH' field
            TypeError, #needed since a list will throw it (str not int) Python coherence...
            IndexError): #needed by the [0]
        subdir = ''
    pwd = pwd / subdir
    node._pwd = pwd
    return visitor.node_mapacc(node, pwd)

def add(ast, root_dir):
    """ root_dir needs to be a Path object. """
    visitor = AstVisitor(default=_pwd, mapacc=True)
    visitor.visit(ast, root_dir)
