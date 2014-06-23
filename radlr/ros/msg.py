'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com


Generate one ROS .msg file per topic and struct declaration.

'''

from radlr.rast import AstVisitor
from astutils.tools import write_file

def _topic(visitor, node, acc):
    """ Topics needs a radl__flags field. """
    filenames, msg_directory = acc
    filename = node._name + '.msg'
    filenames.append(filename)
    filepath = msg_directory / filename
    filecontent = 'uint8 radl__flags\n'
    for n in node['FIELDS']:
        filecontent += '{} {}\n'.format(n._kind, n._name)
    write_file(filepath, filecontent)
    return visitor.node_mapacc(node, acc) #recursive call

def _struct(visitor, node, acc):
    filenames, msg_directory = acc
    filename = node._name + '.msg'
    filenames.append(filename)
    filepath = msg_directory / filename
    filecontent = ''
    for n in node['S']:
        filecontent += '{} {}\n'.format(n._kind, n._name)
    write_file(filepath, filecontent)
    return visitor.node_mapacc(node, acc) #recursive call


_visitor = AstVisitor({'topic' : _topic, 'struct' : _struct})

def gen(msg_directory, ast):
    _, (msg_file_list, _) = _visitor.visit(ast, ([], msg_directory))
    return msg_file_list