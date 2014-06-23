'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com


Generate one ROS .msg file per topic and struct type.
If a struct as a EXTERNAL_ROS_DEF field, it is used instead of generating
a new one.
'''

from radler.astutils.tools import write_file
from radler.radlr import types, infos
from radler.radlr.rast import AstVisitor
from radler.radlr.errors import warning, internal_error
from radler.radlr.types import StructType, ArrayType


def struct_of_topic(struct_t):
    """ It adds the header fields used for ros topics """
    return StructType((('radl__flags', 'uint8'),) + struct_t.elems_t)


def collect(ast):
    """ return a set of struct types to be generated.
    It is not to be generated if it has a field EXTERNAL_ROS_DEF,
    or if it is already in infos.ros_types.
    """
    def _st(visitor, node, s):
        if node._kind == 'topic':
            t = struct_of_topic(types.of(node))
        else:
            t = types.of(node)
        rt = infos.ros_type_of_struct.get(t, None)
        expect_rt = node['EXTERNAL_ROS_DEF']
        if rt:
            if expect_rt and expect_rt != rt:
                warning("This struct will use already defined"
                        " ros type {}".format(str(rt)), node._location)
        else:
            if not expect_rt:
                rt = ast._namespace.generate('radl__msg')
            else:
                rt = expect_rt
            s[t] = rt
            infos.ros_type_of_struct[t] = rt
        node._ros_msg_typename = rt
        return visitor.node_mapacc(node, s) #recursive call

    visitor = AstVisitor({'topic': _st, 'struct' : _st})
    _, s = visitor.visit(ast, dict())
    return s


def ros_typename(t):
    if isinstance(t, str):
        return t
    elif isinstance(t, ArrayType):
        return '{}[{}]'.format(ros_typename(t.elem_t), t.size)
    elif isinstance(t, StructType):
        return infos.ros_type_of_struct[t]
    else:
        internal_error("Unexpected type.")


def ros_msgdef(struct_t):
    elems = ('{} {}'.format(ros_typename(ft), fb)
                        for (fb, ft) in struct_t.elems_t)
    return '\n'.join(elems)


def gen(msg_directory, ast):
    msg_filenames = []
    msgtogen = collect(ast)
    #Generate the needed message files
    for (struct_t, rosname) in msgtogen.items():
        filename = rosname.name() + '.msg'
        msg_filenames.append(filename)
        filepath = msg_directory / filename
        filecontent = ros_msgdef(struct_t)
        write_file(filepath, filecontent)
    return msg_filenames
