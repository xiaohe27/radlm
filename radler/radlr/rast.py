'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.nodetrees import Functor
from collections import Mapping
from radler.astutils.location import no_loc
from radler.radlr.errors import internal_error
from radler.astutils.names import NonExistingIdent

class AlreadyAttached(Exception): pass

class Ident(Mapping):
    """" An ident is a reference to an actual node, it bears the same name
    but has its own location"""
    __slots__ = ['_qname', '_location', '_node']
    def __init__(self, qname, location=None, node=None):
        self._qname = qname
        if node and not location:
            self._location = node._location
        else:
            self._location = location if location else no_loc
        self._node = node

    def __getstate__(self):
        """ Allows pickling: necessary because of __getattr__"""
        return tuple(getattr(self, a) for a in Ident.__slots__)
    def __setstate__(self, state):
        """ Allows pickling: necessary because of __getattr__"""
        for a,v in zip(Ident.__slots__, state): setattr(self, a, v)

    @classmethod
    def of(cls, node_or_ident):
        if isinstance(node_or_ident, Ident):
            return Ident(node_or_ident._qname, True, node=node_or_ident._node)
        else:
            return Ident(node_or_ident._qname, True, node=node_or_ident)

    def _attach(self, node):
        if self._node:
            raise AlreadyAttached()
        self._node = node
    def _reattach(self, node):
        self._node = node

    def __str__(self):
        return "{}".format(self._qname)
    def __repr__(self):
        return self.__str__()
    #container convention, behave like node
    def __len__(self):
        return len(self._node)
    def __getitem__(self, key):
        return self._node[key]
    def __iter__(self):
        return iter(self._node)
    def __setattr__(self, attr, value):
        if attr in self.__slots__:
            object.__setattr__(self, attr, value)
        else:
            setattr(self._node, attr, value)
    def __getattr__(self, attr):
        return getattr(self._node, attr)
    def __copy__(self):
        internal_error("Trying to copy an Ident.")
    def __deepcopy__(self, d):
        internal_error("Trying to deepcopy an Ident.")


class Alias(Ident):
    __slots__ = ['_qname', '_location', '_is_alias_of']
    #duplicate the Ident __slots__ for ease of use.
    def __init__(self, qname, location, target_ident):
        self._qname = qname
        self._location = location
        self._is_alias_of = target_ident
    def __getstate__(self):
        """ Allows pickling: necessary because of __getattr__"""
        return tuple(getattr(self, a) for a in Alias.__slots__)
    def __setstate__(self, state):
        """ Allows pickling: necessary because of __getattr__"""
        for a,v in zip(Alias.__slots__, state): setattr(self, a, v)
    @property
    def _node(self):
        return self._is_alias_of._node

class AstNode(Mapping):
    """ Basically a named kind with children.
    Inherit its container behavior from its children container.
    """
    def __init__(self, kind, qname, children, namespace, location):
        self._kind = kind
        self._qname = qname
        self._children = children
        self._namespace = namespace
        self._location = location

    @property
    def _val(self):
        """ Used for nodes holding one value as their unique child."""
        if len(self._children) != 1:
            internal_error("Tried to get _val of a node with {} childs"
                           "".format(len(self._children)))
        return self._children[0]
    #container convention, behave like _children
    def __len__(self):
        return len(self._children)
    def __getitem__(self, key):
        try:
            return self._children[key]
        except (KeyError, TypeError):
            pass #TypeError is useful in case _children is a list and key a str
        pa = list(iter(self._children))
        raise KeyError("{a} is not among the possibilities:"
                       "{pa}".format(a=key, pa=pa))
    def __iter__(self):
        return iter(self._children)
    def __getattr__(self, attr):
        """Attributes are namespace lookup.
        """
        #ensure we have a _namespace attribute in case __getattr__ is called
        #before init (for example when using copy).
        object.__getattribute__(self, '_namespace')
        try:
            return self._namespace[attr]
        except NonExistingIdent: pass
        raise AttributeError(attr)

    @property
    def _typed_name(self):
        return "{n} : {k}".format(n=self._qname.name(), k=self._kind)

    def __str__(self): pass # This is replaced bellow since we need the class
    #to be defined before we can define a visitor which will give us a nice
    #printing function. 

    def __repr__(self):
        return self.__str__()


AstVisitor = Functor(AstNode, '_children', '_kind').Visitor
"""The visitor for Ast and AstNodes"""

AstNode.__str__ = Functor(AstNode, '_children', '_typed_name').spprint_node
"""Pretty printer for AstNodes register in the class"""

