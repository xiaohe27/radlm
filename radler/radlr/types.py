'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

In RADL, types are structural.
'''
from collections import namedtuple
import re

from radler.radlr import infos, language
from radler.radlr.errors import error
from radler.radlr.rast import AstVisitor


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

sized_type_regex = re.compile(r"(?P<base>[a-zA-Z]+)(?P<size>\d+)")

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
        r1 = sized_type_regex.match(t1)
        r2 = sized_type_regex.match(t2)
        if not r1 or not r2: #one of them doesn't match the regex
            return False
        (b1, s1), (b2, s2) = r1.groups(), r2.groups()
        return b1 == b2 and b1 in language.sized_types and int(s1) <= int(s2)


def smallest_type(value, t):
    """ Look for the smallest type able to store value.
    Assumes that t is a correct type.
    """
    #TODO: 8 the implementation should do a dichotomic search
    b, s = sized_type_regex.match(t)
    sizes = language.sized_types.get(t, [])
    for cs in sizes: # sizes are expected to be increasing.
        if cs >= s:
            return t
        if language.check_type_size[t](value, cs):
            return b+str(cs)
    return t

def _basic_check(node):
    """ Use the language checkers if it exists.
    """
    try:
        check = language.check_type[node._kind]
        if check:
            check(node._val, node._location)
    except KeyError:
        # No checker registered, simply pass
        pass

def _check(node):
    """ Compute and check the type of a node.
    If the node was user annotated, do not change the type, only check it.
    Otherwise, try to compute the best (smallest) type.
    """
    mk = infos.semantics.metakinds.get(node._kind, None)  # @UndefinedVariable
    if mk == 'type':
        _basic_check(node)
        t = node._kind
        if not node._user_type_annoted and node._kind in language.sized_types:
            # we try to optimize the size
            t = smallest_type(node._val, node._kind)
    elif node._kind == 'array':
        def check_el(el, ex_t):
            elt = of(el)
            if not is_sub(elt, ex_t):
                error("This array element has type {} when {} is expected."
                      "".format(str(elt), str(ex_t)), el._location)
        expected_t = node['TYPE']
        expected_size = node['SIZE']
        values = node['VALUES']
        # check size
        if expected_size and len(values) != int(expected_size._val):
            error("This array is of size {} when {} is expected."
                  "".format(len(values), expected_size), node._location)
        # use first element if no expected_elem_t is found
        if not expected_t:
            expected_t = of(values[0])
        else:
            expected_t = expected_t._val
            check_el(values[0], expected_t)
        for el in values[1:]:
            check_el(el, expected_t)
        t = ArrayType(len(values), expected_t)
    elif node._kind == 'struct' or node._kind == 'topic':
        fields = (((f._qname.name(), of(f)) for f in node['FIELDS']))
        t = StructType(tuple(fields))
    else:
        t = None #TODO: 6 probably to change when outputing the AST as data.
    return t


def of(node):
    """Return the type of a node, memoize it in node._type
    """
    if not hasattr(node, '_type'):
        node._type = _check(node)
    return node._type

def typecheck(node):
    """ Check the type of a node recursively.
    It is typically used to check the entire AST in one call.
    """
    def mapredof(visitor, node, _):
        of(node)
        return visitor.node_mapacc(node, _)
    visitor = AstVisitor(default=mapredof, inplace=True)
    #ast isn't an actual kind (not in metakinds), so only go over the children.
    visitor.children_mapacc(node, ())
    return
