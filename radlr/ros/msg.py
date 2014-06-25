'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com


Generate one ROS .msg file per topic and struct declaration.

'''

from radlr.rast import AstVisitor
from astutils.tools import write_file
from radlr import types

def to_ros_typedef(field):
    if field._kind == 'field_struct':
        kind = field['STRUCT']._name
    elif field._kind == 'struct':
        kind = field._name
    else:
        kind = types.to_str(types.of(field))
    return '{} {}\n'.format(kind, field._name)

def _msg(static_part):
    def __msg(visitor, node, acc):
        filenames, msg_directory = acc
        filename = node._name + '.msg'
        filenames.append(filename)
        filepath = msg_directory / filename
        filecontent = static_part
        for f in node['FIELDS']:
            filecontent += to_ros_typedef(f)
        write_file(filepath, filecontent)
        return visitor.node_mapacc(node, acc) #recursive call
    return __msg

#Topics needs a radl__flags field.
_visitor = AstVisitor({'topic' : _msg('uint8 radl__flags\n'),
                       'struct' : _msg('')})

def gen(msg_directory, ast):
    _, (msg_file_list, _) = _visitor.visit(ast, ([], msg_directory))
    return msg_file_list