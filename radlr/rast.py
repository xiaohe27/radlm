'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from astutils.nodetrees import Functor
from astutils.idents import Namespace
from astutils.tools import str



class AstNode:
    """ Basically a kind (_name) with children (_children).
    Inherit its container behavior from its children container.
    """
    def __init__(self, kind, ident, children, namespace, location):
        self._kind = kind
        self._ident = ident
        self._children = children
        self._namespace = namespace
        self._location = location
    @property
    def _name(self):
        return self._ident._name
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
        except KeyError:
            pass
        pa = list(iter(self._children))
        raise KeyError("{a} is not among the possibilities:"
                       "{pa}".format(a=key, pa=pa))
    def __iter__(self):
        return iter(self._children)
    def __getattr__(self, attr):
        """Attributes are namespace lookup.
        TODO 8: should it be only local lookup?
        """
        #ensure we have a _namespace attribute
        #in case __getattr__ is called before init
        #(for example when using copy)
        object.__getattribute__(self, '_namespace')
        return self._namespace.get_ident(attr)

    def __str__(self):
        s = "{n} : {k} {c}".format(n=self._ident._name, k=self._kind, c=str(self._children))
        return s
    def __repr__(self):
        return self.__str__()


rastutils = Functor(AstNode, '_children', '_kind')
AstVisitor = rastutils.NodeTreeVisitor
pp_ast = rastutils.pprint_node


class Ast(AstNode):
    """ An ast object stores the toplevel definitions in the mappings
    ast.kinds maps kind_name -> 'clas' | 'typ' | 'enum' | 'struct'
    ast.keywords is a set of keywords
    """
    def __init__(self, name, location, kinds, keywords, namespace=None, children=None):
        self._kinds = kinds
        self._keywords = keywords
        if not namespace:
            namespace = Namespace()
        if not children:
            children = dict()
        AstNode.__init__(self, name, None, children, namespace, location)

    @property
    def _name(self):
        return self._kind

    def _kind_of(self, ident):
        return self.kinds[ident]
    def _def_of(self, ident):
        return self.defs[ident]
    def __str__(self):
        return "defs:\n{defs}".format(defs=str(self._namespace))

    #Container convention
#     def __len__(self):
#         """ return 0 if empty : false
#         """
#     def __getitem__(self):
#         """ the accepted keys should be integers and slice objects.
#         Negative indexes
#         wrong key type : TypeError
#         out of bounds key : IndexError (used by for loops)
#         """
#         #http://www.stackoverflow.com/questions/2936863/python-implementing-slicing-in-getitem
#     def __iter__(self):
#         """ returns an iterator object. probably a coroutine.
#         """

