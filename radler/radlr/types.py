'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

In RADL, types are structural.
'''
from collections import namedtuple
import re

from radler.radlr import infos
from radler.radlr.errors import internal_error, error


class ArrayType(namedtuple('array_type', ['size', 'elem_t'])):
    """ An array type has two fields, size and elem_t.
    """
    def __str__(self):
        return "{}[{}]".format(str(self.elem_t), self.size)

class StructType(namedtuple('struct_type', ['elems_t'])):
    """elems_t should be a tuple of tuples (field_name, field_type).
    The type is canonical since field_name are reordered to be sorted.
    """
    def __new__(cls, elems_t):
        elems_t = sorted(elems_t, key=(lambda v:v[0]))
        return super().__new__(cls, tuple(elems_t))

    def __str__(self):
        return "struct {{{}}}".format(', '.join(map(str, self.elems_t)))


def are_eq(t1, t2):
    """Return types equality, structs are structurally compared
    """
    return t1 == t2

sub_regex = re.compile(r"(?P<base>[a-zA-Z]+)(?P<size>\d+)")

def is_sub(t1, t2):
    """Returns whether t1 is a subtype of t2.
    """
    if isinstance(t1, ArrayType) and isinstance(t1, ArrayType):
        return (t1.size == t2.size) and is_sub(t1.elem_t, t2.elem_t)
    if isinstance(t1, StructType) and isinstance(t1, StructType):
        e1l = t1.elems_t
        e2l = t2.elems_t
        if len(e1l) != len(e2l): return False
        for ((f1, ft1), (f2, ft2)) in zip(e1l, e2l):
            # note that fields are sorted by definition of StructType
            if f1 != f2 or not is_sub(ft1, ft2):
                return False
        return True
    if isinstance(t1, str) and isinstance(t2, str):
        if t1 == t2:
            return True
        r1 = sub_regex.match(t1)
        r2 = sub_regex.match(t2)
        if not r1 or not r2: #one of them doesn't match the regex
            return False
        (b1, s1), (b2, s2) = r1.groups(), r2.groups()
        return b1 == b2 and int(s1) <= int(s2)

def of(node):
    """Return the type of a node, memoize it in node._type"""
    if hasattr(node, '_type'): #first check if it already has a _type field
        return node._type
    mk = infos.semantics.metakinds[node._kind]  # @UndefinedVariable
    if mk == 'type':
        t = node._kind
    elif node._kind == 'array':
        def check(el, ex_t):
            t = of(el)
            if not is_sub(t, ex_t):
                msg = ("This array element has type {} when {} is expected."
                       "".format(str(t), str(ex_t)))
                error(msg, el._location)
        expected_t = node['TYPE']
        expected_size = node['SIZE']
        values = node['VALUES']
        # check size
        if expected_size and len(values) != int(expected_size._val):
            msg = ("This array is of size {} when {} is expected."
                   "".format(len(values), expected_size))
            error(msg, node._location)
        # use first element if no expected_elem_t is found
        if not expected_t:
            expected_t = of(values[0])
        else:
            expected_t = expected_t._val
            check(values[0], expected_t)
        for el in values[1:]:
            check(el, expected_t)
        t = ArrayType(len(values), expected_t)
    elif node._kind == 'struct' or node._kind == 'topic':
        t = StructType(tuple(((f._qname.name(), of(f)) for f in node['FIELDS'])))
    else:
        internal_error("Trying to get the datatype of non data node.\n{}"
                       "".format(str(node._location)))
    node._type = t
    return t