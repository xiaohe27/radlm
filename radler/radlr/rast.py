'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.idents import Namespace
from radler.astutils.nodetrees import Functor
from collections import Mapping


class AstNode(Mapping):
    """ Basically a named kind with children.
    Inherit its container behavior from its children container.
    """
    def __init__(self, kind, name, children, namespace, location):
        self._kind = kind
        self._name = name
        self._children = children
        self._namespace = namespace
        self._location = location
    @property
    def _qualname(self):
        """ Return fully qualified name """
        #TODO: 6 namespaces and qualnames
        pass
    @property
    def _val(self):
        """ Used for nodes holding one value as their unique child."""
        if len(self._children) != 1:
            raise Exception("tried to get the _val of a node with {} childs"
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
        """Attributes are children lookup.
        """
#         #ensure we have a _children attribute in case __getattr__ is called
#         #before init (for example when using copy).
#         object.__getattribute__(self, '_children')
#         class Found(Exception): pass
#         def stop(visitor, n, attr):
#             if n._name == attr: raise Found(n)
#             else: return n, attr
#         try:
#             AstVisitor(default = stop).visit(self)
#         except Found as e:
#             return e.args[0]
#         raise AttributeError
        #ensure we have a _namespace attribute in case __getattr__ is called
        #before init (for example when using copy).
        object.__getattribute__(self, '_namespace')
        return self._namespace.get_node(attr)

    @property
    def _typed_name(self):
        return "{n} : {k}".format(n=self._name, k=self._kind)

    def __str__(self): pass # This is replaced bellow since we need the class
    #to be defined before we can define a visitor which will give us a nice
    #printing function. 

    def __repr__(self):
        return self.__str__()


AstVisitor = Functor(AstNode, '_children', '_kind').Visitor
"""The visitor for Ast and AstNodes"""

AstNode.__str__ = Functor(AstNode, '_children', '_typed_name').spprint_node
"""Pretty printer for AstNodes register in the class"""


class Ast(AstNode):
    """ An ast object stores the toplevel definitions in the mappings
    ast.kinds maps kind_name -> 'clas' | 'typ' | 'enum' | 'struct'
    ast.keywords is a set of keywords
    """
    def __init__(self, name, location, kinds, keywords, namespace, children=None):
        self._kinds = kinds
        self._keywords = keywords
        if not namespace:
            namespace = Namespace()
        if not children:
            children = dict()
        AstNode.__init__(self, '_ast', name, children, namespace, location)

    def _metakind_of_kind(self, k):
        return self._kinds[k]
#     def __str__(self):
#         return "{defs}".format(defs=str(self._children))
