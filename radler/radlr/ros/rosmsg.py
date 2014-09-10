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
from radler.radlr.errors import internal_error
from radler.radlr.types import StructType, ArrayType
from radler.radlr.ros.rosutils import qn_msgfile, filepath, qn_file, qn_cpp,\
    qn_msgtype


def struct_of_topic(struct_t):
    """ It adds the header fields used for ros topics """
    return StructType((('radl__flags', 'uint8'),) + struct_t.elems_t)


def collect(ast):
    """ return a set of struct types to be generated.
    It is not to be generated if it has a field EXTERNAL_ROS_DEF,
    or if it is already in infos.ros_types.
    """
    def _st(visitor, node, s):
        """ s is a mapping between names and filepath of messages to generate.
        """
        if node._kind == 'topic': #special treatment to add private fields
            t = struct_of_topic(types.of(node))
        else:
            t = types.of(node)
        ext_rd = node['EXTERNAL_ROS_DEF']
        if ext_rd: #External def to be used
            name = ext_rd['FULLNAME']._val
            header = ext_rd['HEADER']._val
        else:
            reg_name = infos.ros_type_of_struct.get(t, None)
            if not reg_name: #Msg name and file to create
                name = ast._namespace.generate('radl__msg')
                #note: the .msg file is different from the header linked to it
                s[t] = filepath(qn_msgfile(name, suffix='.msg'))
                infos.ros_type_of_struct[t] = name
            else: #Msg file already created
                name = reg_name
            header = qn_file(name, suffix='.h')
            #keep only the actual string to identify the type and header file
            name = qn_cpp(name)
        #Store our findings in the node for future retrieval
        node._ros_msgtype_name = name
        node._ros_msgtype_header = header
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
        return qn_msgtype(infos.ros_type_of_struct[t])
    else:
        internal_error("Unexpected type.")


def ros_msgdef(struct_t):
    elems = ('{} {}'.format(ros_typename(ft), fb)
                        for (fb, ft) in struct_t.elems_t)
    return '\n'.join(elems)


def gen(ast):
    msgs= []
    msgtogen = collect(ast)
    #Generate the needed message files
    for (struct_t, file) in msgtogen.items():
        msgs.append(file)
        filecontent = ros_msgdef(struct_t)
        write_file(file, filecontent)
    return msgs
