'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

The central concept here is a node tree.
    A node in a node tree is an instance of node_class, list, tuple or dict.
    Anything else is considered a leaf.

'''
from collections import MutableSequence, Sequence, MutableMapping
from copy import copy
from functools import partial

from radler.astutils.tools import Bunch
from radler.radlr.errors import internal_error


def mapred(f, l, acc, inplace):
    """ map reduce the function f on l with starting accumulator acc
    @param f: has the behavior (old_elem, old_acc) -> (new_elem, new_acc)
    @param l: the input MutableSequence (like list)
    @param acc : the input accumulator
    @param inplace: if true output inplace in l otherwise in a copy of l
    """
    nl = l if inplace else copy(l)
    for i in range(len(nl)):
        nl[i], acc = f(nl[i], acc)
    return nl, acc

def mapacc(f, l, acc, inplace=False):
    """ map acc, same as mapred, but the same (the input one) accumulator is
    given to the mapped function f. During the mapping, the returned
    accumulators are dropped. The input accumulator is returned unchanged.
    """
    nl = l if inplace else copy(l)
    for i in range(len(nl)):
        nl[i], _ = f(nl[i], acc)
    return nl, acc

def imapred(f, t, acc):
    """ same as mapred, but works for immutable sequences (like tuple).
    NB, create a new instance of t.__class__  from the mapred list result.
    """
    l = list(iter(t))
    l, acc = mapred(f, l, acc, inplace=True)
    return t.__class__(l), acc

def imapacc(f, t, acc):
    """ same as imapred but with mapacc.
    """
    l = list(iter(t))
    l, acc = mapacc(f, l, acc, inplace=True)
    return t.__class__(l), acc

def dmapred(f, d, acc, inplace=False):
    """ same as mapred but MutableMapping (like dict)
    NB, the function f takes a (key, value) couple as old_elem and new_elem.
    """
    nd = d if inplace else copy(d)
    for k in iter(nd):
        nd[k], acc = f(nd[k], acc)
    return nd, acc

def dmapacc(f, d, acc, inplace=False):
    """ same as dmapred but with mapacc.
    """
    nd = d if inplace else copy(d)
    for k in iter(nd):
        nd[k], _ = f(nd[k], acc)
    return nd, acc


class E(Exception):
    """Raise an instance of E during the visitation and
    the given string will be formatted with the attached node as node.
    """
    def __init__(self, message, node=None):
        self.message = message
        self.node = node
    def attach_to_first_node(self, node):
        """Only the first attachment is kept"""
        if not self.node:
            self.node = node
    def __str__(self):
        return self.args[0].format(node=self.node)


def Functor(node_class, attr_children, attr_name):
    """ Generate the classes and functions according to the attribute names.
    Return them in a dictionary.
    """
    def children_of(node):
        return getattr(node, attr_children)
    def set_children(node, children):
        setattr(node, attr_children, children)
    def name_of(node):
        return getattr(node, attr_name)
    def is_node(node):
        return isinstance(node, node_class)

    class Visitor:
        """ A nodetree Visitor is by default the identity mapred on node trees.
        To behave differently on leafs, provide a onleaf function,
            on lists, provide a onlist function,
            on nodes, - the default one may be changed by providing default
                      - specific ones may be added in the definitions dict
                        whose keys are the node names.

        The class defines some convenient methods to mapred/mapacc trees.
        """

        def error(self, s, node=None):
            raise E("{node.std_pos}"+s, node)

        def node_mapred(self, n, acc):
            """ shallow copy n and mapred self.visit on its children
            returns the accumulated acc
            """
            children = children_of(n)
            try:
                v = self.mapred(children, acc)
                children, acc = v
            except E as e:
                e.attach_to_first_node(n)
                raise e
            nn = n if self.inplace else copy(n)
            set_children(nn, children)
            return nn, acc

        def node_mapacc(self, n, acc):
            """ shallow copy n and mapacc self.visit on its children
            returns acc untouched
            """
            children = children_of(n)
            try:
                children, acc = self.mapacc(children, acc)
            except E as e:
                e.attach_to_first_node(n)
                raise e
            nn = n if self.inplace else copy(n)
            set_children(nn, children)
            return nn, acc

        def list_mapred(self, l, acc):
            """ mapred self.visit on l"""
            return mapred(self.visit, l, acc, self.inplace)

        def list_mapacc(self, l, acc):
            """mapacc self.visit on l"""
            return mapacc(self.visit, l, acc, self.inplace)

        def tuple_mapred(self, t, acc):
            """ imapred self.visit on t"""
            if self.inplace:
                internal_error("Trying to modify in place a tuple.")
            return imapred(self.visit, t, acc)

        def tuple_mapacc(self, t, acc):
            """ tmapacc self.visit on t"""
            if self.inplace:
                internal_error("Trying to modify in place a tuple.")
            return imapacc(self.visit, t, acc)

        def dict_mapred(self, d, acc):
            """ dmapred self.visit on d"""
            return dmapred(self.visit, d, acc, self.inplace)

        def dict_mapacc(self, d, acc):
            """ dmapacc self.visit on d"""
            return dmapacc(self.visit, d, acc, self.inplace)

        def leaf_mapred(self, leaf, acc):
            return leaf, acc

        def leaf_mapacc(self, leaf, acc):
            return leaf, acc

        def __init__(self, definitions=None, default=None,
                     onlist=None, ontuple=None,
                     ondict=None, onleaf=None, params=None,
                     inplace=False, mapacc=False):
            """ definitions is a dict specifing some functions 'fun
            which will be called on Nodes named 'fun
            if 'fun doesn't exist, we use default.
            """
            def optarg(var, mapacc_ver, mapred_ver):
                return var if var else mapacc_ver if mapacc else mapred_ver

            self.definitions = definitions if definitions else {}

            self.default = optarg(default, Visitor.node_mapacc, Visitor.node_mapred)# @UndefinedVariable
            self.onlist = optarg(onlist, Visitor.list_mapacc, Visitor.list_mapred)# @UndefinedVariable
            self.ontuple = optarg(ontuple, Visitor.tuple_mapacc, Visitor.tuple_mapred)# @UndefinedVariable
            self.ondict = optarg(ondict, Visitor.dict_mapacc, Visitor.dict_mapred)# @UndefinedVariable
            self.onleaf = optarg(onleaf, Visitor.leaf_mapacc, Visitor.leaf_mapred)# @UndefinedVariable
            self.inplace = inplace
            self.params = params

#         def change(self, definitions={}, default=None, onlist=None,
#                    ontuple=None, ondict=None, onleaf=None):
#             """ return a new visitor equal to self but updated. """
#             d = copy(self)
#             d.definitions.update(definitions)
#             if default: d.default = default
#             if onlist: d.onlist = onlist
#             if ontuple: d.ontuple = ontuple
#             if ondict: d.ondict = ondict
#             if onleaf: d.onleaf = onleaf
#             return d

        def __getitem__(self, key):
            if self.params: return self.params[key]
            else: raise KeyError

        ##### visiting methods

        def visit(self, node, acc=None):
            """ Call the correct visiting function depending on the type of node.
            """
            if is_node(node) :
                try:
                    method = self.definitions[name_of(node)]
                except KeyError:
                    method = self.default
                try:
                    return method(self, node, acc)
                except E as e:
                    e.attach_to_first_node(node)
                    raise e
            elif isinstance(node, list) :
                return self.onlist(self, node, acc)
            elif isinstance(node, tuple) :
                return self.ontuple(self, node, acc)
            elif isinstance(node, dict) :
                return self.ondict(self, node, acc)
            else :
                return self.onleaf(self, node, acc)

        def mapred(self, node, acc=None):
            """ call the correct mapred.
                Useful in the user defined function to do one normal mapred step
            """
            if is_node(node):
                return self.node_mapred(node, acc)
            elif isinstance(node, MutableSequence):
                return self.list_mapred(node, acc)
            elif isinstance(node, Sequence): #Immutable sequence
                return self.tuple_mapred(node, acc)
            elif isinstance(node, MutableMapping):
                return self.dict_mapred(node, acc)
            else :
                return self.leaf_mapred(node, acc)

        def mapacc(self, node, acc=None):
            """ call the correct mapacc.
                Useful in the user defined function to do one normal mapacc step
            """
            if is_node(node):
                return self.node_mapacc(node, acc)
            elif isinstance(node, MutableSequence):
                return self.list_mapacc(node, acc)
            elif isinstance(node, Sequence): #Immutable sequence
                return self.tuple_mapacc(node, acc)
            elif isinstance(node, MutableMapping):
                return self.dict_mapacc(node, acc)
            else :
                return self.leaf_mapacc(node, acc)

        #Utilitary methods

        def lift(self, node, acc=None):
            """ Replace a node with its children.
            If a list, a tuple or a dict is given, it is returned unchanged.
             No visitation is done, if wanted, see lift_mapacc/lift_mapred.
            """
            if is_node(node):
                return children_of(node), acc
            elif isinstance(node, (list, tuple, dict)):
                return node, acc
            else:
                self.error("Visitation error, unable to lift a leaf.")
        def lift_mapacc(self, node, acc=None):
            return self.lift(*self.mapacc(node, acc))
        def lift_mapred(self, node, acc=None):
            return self.lift(*self.mapred(node, acc))

        def left(self, node, acc=None):
            """Replace a node with its first children, dicts, tuples and list
            are replaced with their first element (container.[0]).
            No visitation is done, if wanted, see left_mapacc/left_mapred.
            """
            if is_node(node):
                return self.left(children_of(node), acc)
            elif isinstance(node, (list, tuple, dict)):
                try:
                    return node[0], acc
                except KeyError:
                    pass
                self.error("Visitation error, no left element of empty.")
            else:
                self.error("Visitation error, no left element of a leaf.")
        def left_mapacc(self, node, acc=None):
            return self.left(*self.mapacc(node, acc))
        def left_mapred(self, node, acc=None):
            return self.left(*self.mapred(node, acc))
# 
#         def left_leaf(self, node):
#             """Recursively lift left until hitting a leaf.
#             If the last node as no children, None is returned.
#             """
#             if isinstance(node, node_class):
#                 return self.left_leaf(getattr(node, attr_children))
#             elif isinstance(node, (list, tuple)):
#                 if not node: #empty
#                     return None
#                 return self.left_leaf(node[0])
#             else:
#                 return node
#         def left_leaf_mapred(self, node, acc=None):
#             node, acc = self.mapred(node, acc)
#             leaf = self.left_leaf(node)
#             return leaf, acc
# 
        def left_node(self, node):
            """Recursively lift left until a node is found.
            If none is found the given node is returned.
            """
            if isinstance(node, node_class):
                r = self.left_node(getattr(node, attr_children))
                if not r: #no children available, we are the one
                    return node
                return r
            elif isinstance(node, (list, tuple)):
                if not node: #empty
                    return None
                return self.left_node(node[0])
            else:
                return None
#         def left_node_mapred(self, node, acc=None):
#             node, acc = self.mapred(node, acc)
#             child = self.left_node(node)
#             return child, acc
# 
        def right_node(self, node):
            if isinstance(node, node_class):
                r = self.right_node(getattr(node, attr_children))
                if not r: #no children available, we are the one
                    return node
                return r
            elif isinstance(node, (list, tuple)):
                if not node:
                    return None
                return self.right_node(node[-1])
            else:
                return None


    def spprint_node(node, indentsize=2, maxwidth=100):
        """ pretty print a node
        @Return a string
        """
        class NeedMoreThanOneLine(Exception): pass

        ind = ' ' * indentsize

        def indent(text):
            return '\n'.join((ind + line) for line in text.splitlines())

        def subwidth(width, length):
            if width<=0:
                nwidth = width + length
                if nwidth>0: raise NeedMoreThanOneLine()
                return nwidth
            else:
                return width - length

        class Around:
            def __init__(self, left, node, right):
                self.node = node
                self.left = left
                self.right = right
                self.apl = len(right)+len(left)
            def visit(self, visitor, width):
                width = subwidth(width, self.apl)
                s, w = visitor.visit(self.node, width)
                return self.left + s + self.right, w
        class Atom:
            def __init__(self, atom):
                self.atom = atom

        def print_leaf(visitor, x, width):
            """ Negative width asks for one liner.
            Absolute value of width gives available space.
            @Return the string and if width was negative, the resting space.
                if it was positive, the return width is meaningless
            """
            if isinstance(x, Around):
                return x.visit(visitor, width)
            elif isinstance(x, Atom):
                s = x.atom
#             elif hasattr(x, 'pp'):
#                 s = x.pp(width)
            else:
                s = repr(x)
            return s, subwidth(width, len(s))

        def print_box(visitor, l, width):
            """ Negative width asks for one liner.
            Absolute value of width gives available space.
            @Return the string and if width was negative, the resting space.
                if it was positive, the return width is meaningless
            """
            if width <= 0: #We need to fit in one line
                sep, lsep = '', 0
                nwidth = subwidth(width, lsep*(len(l)-1)) #add required spaces
                sl, nwidth = visitor.list_mapred(l, nwidth)
            else:
                try: #Try to fit in one line
                    s, nwidth = print_box(visitor, l, -width)
#                     s = '{{{n}@{s}@{nn}}}'.format(n=width, s=s, nn=-nwidth)
                    return s, -nwidth
                except NeedMoreThanOneLine:
                #Span on multiple lines and indent if possible
                    sep = '\n'
                    doindent = (width - indentsize) > 0
                    nwidth = width - indentsize if doindent else width
                    sl, nwidth = visitor.list_mapacc(l, nwidth)
                    if doindent and len(sl)>1:
                        sl[1:-1] = map(indent, sl[1:-1])
            s = sep.join(sl)
#             s = '{{{n}@{sl}@{nn}}}'.format(n=width, sl=s, nn=nwidth)
            return s, nwidth

        def print_seq(leftsep, sep, rightsep, visitor, l, width):
            if isinstance(l, dict):
                l = list(l.items())
            if len(l)==0:
                s = leftsep+rightsep
                return s, subwidth(width, len(s))
            elif len(l)==1:
                return Around(leftsep, l[0], rightsep).visit(visitor, width)
            else:
                nl = [Atom(leftsep)]
                for x in l[0:-1]:
                    nl.append(Around('', x, sep))
                nl.append(l[-1])
                nl.append(Atom(rightsep))
                return print_box(visitor, nl, width)

        def print_node(visitor, n, width):
            left = '<{nn} '.format(nn=name_of(n))
            return print_seq(left, ' ', '>', visitor, children_of(n), width)

        visitor = Visitor({}, default=print_node,
                          onlist=partial(print_seq, '[', ', ', ']'),
                          ontuple=partial(print_seq, '(', ', ', ')'),
                          ondict=partial(print_seq, '{', ', ', '}'),
                          onleaf=print_leaf)
        s, _ = visitor.visit(node, maxwidth)
        return s

    return Bunch(**locals())