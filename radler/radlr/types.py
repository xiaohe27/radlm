'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

In RADL, types are structural.
'''
from radler.radlr import infos
from radler.radlr.errors import internal_error, error
from collections import namedtuple


class ArrayType(namedtuple('array_type', ['size', 'elem_t'])):
    """ An array type has two fields, size and elem_t.
    """
    def __str__(self):
        return "{}[{}]".format(str(self.elem_t), self.size)

class StructType(namedtuple('struct_type', ['elems_t'])):
    """elems_t should be a tuple of tuples (field_name, field_type).
    """
    def __str__(self):
        return "struct {{{}}}".format(', '.join(map(str, self.elems_t)))


def are_eq(t1, t2):
    """Return types equality, structs are structurally compared
    """
    return t1 == t2


def of(node):
    """Return the type of a node, memoize it in node._type"""
    try: #first check if it already has a _type field
        return node._type
    except AttributeError: pass
    mk = infos.ast._metakind_of_kind(node._kind)  # @UndefinedVariable
    if mk == 'type':
        t = node._kind
    elif node._kind == 'array':
        values = node['VALUES']
        elem_t = of(values[0])
        for v in values[1:]: #check elements' types
            if not are_eq(elem_t, of(v)):
                msg = ("This array element has type {} when {} is expected."
                       "".format(str(of(v)), str(elem_t)))
                error(msg, v._location)
        size = len(values)
        t = ArrayType(size, elem_t)
    elif node._kind == 'struct' or node._kind == 'topic':
        t = StructType(tuple(((f._name, of(f)) for f in node['FIELDS'])))
    elif node._kind == 'topic_of_struct':
        t = of(node['STRUCT'])
    else:
        internal_error("Trying to get the datatype of non data node.\n{}"
                       "".format(str(node._location)))
    node._type = t
    return t