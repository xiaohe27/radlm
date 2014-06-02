'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com


Generate one ROS .msg file per topic declaration.

'''
from radlr.rast import AstVisitor
from astutils.tools import write_file

def _topic(visitor, node, msg_directory):
    """ Topics are not recursive """
    filepath = msg_directory / (node._name + '.msg')
    filecontent = ''
    for n in node['FIELDS']:
        filecontent += '{} {}\n'.format(n._kind, n._name)
    write_file(filepath, filecontent)
    return (), msg_directory


_visitor = AstVisitor({'topic' : _topic})

def gen(msg_directory, ast):
    _visitor.visit(ast, msg_directory)