'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.radlr import infos
from radler.radlr.errors import internal_error


def of(node):
    """Return the type of a node."""
    mk = infos.ast._metakind_of_kind(node._kind)
    if mk == 'type':
        return node._kind
    elif node._kind == 'array':
        return node['TYPE']
    else:
        internal_error("Trying to get the datatype of non data node.\n{}"
                       "".format(str(node._location)))

def to_str(t):
    """For basic types, it is the associated value type kind.
    For arrays, this string is a structural representation.
    For structs, it is qualified name.
    """
    if isinstance(t, str):
        return t
    elif t._kind == 'array_type':
        return "{et}[{s}]".format(et=to_str(t['ELEM_TYPE']), s=t['SIZE']._val)
    elif t._kind == 'basic_type':
        return t._val
    else:
        internal_error("Trying to get the typename of non type.")

def are_eq(t1, t2):
    """Return types equality,
    structural check is done on arrays only.
    """
    return to_str(t1) == to_str(t2)

#     if t1 == t2:
#         return True
#     elif t1._kind == 'array_type' and t2._kind == 'array_type':
#         same_size = t1['SIZE']==t2['SIZE']
#         return same_size and are_eq(t1['ELEM_TYPE'], t2['ELEM_TYPE'])
#     else:
#         return False