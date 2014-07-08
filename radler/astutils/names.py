'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Here we define what is a qualified name.
'''

from radler.radlr.errors import internal_error
from collections.abc import MutableMapping


class ExistingIdent(Exception): pass
class NonExistingIdent(KeyError): pass

class QualifiedName(tuple):
    def __new__(cls, modname, name, generated):
        """Create a qualified name,
        modname should be a Name,
        name a string,
        generated is True if this name is not extracted from the source."""
        return tuple.__new__(cls, (modname, name, generated))
#     @classmethod
#     def namein(cls, name, generated, namespace):
#         """Create a qualified name using its enclosing namespace."""
#         return cls(namespace.modname(), name, generated)
    def name(self):
        """local name"""
        return self[1]
    def modname(self):
        """enclosing module of this name"""
        return self[0]
    def asmodule(self, sep='/', root=None):
        return self.qname(sep, root) + sep
    def qname(self, sep='/', root=None):
        """Qualified name"""
        return self[0].asmodule(sep, root) + self[1]
    def generated(self):
        return self[2]
    def pathwalk(self):
        yield from self[0].pathwalk()
        yield self[1]
    def pathwalk_relativeto(self, qname):
        me = self.pathwalk()
        root = qname.pathwalk()
        for n1,n2 in zip(me, root):
            if n1 != n2: return None
        if next(root, False): return None
        else: return me
    def includes(self, qname):
        p = self.pathwalk_relativeto(qname)
        return p == None
    def str_relativeto(self, qname):
        """ return the representation relative to modname.
        If it is not included in modname, print qualified name."""
        p = self.pathwalk_relativeto(qname)
        return '/'.join(iter(p)) if p else self.qname()
    def __str__(self):
        return self.qname()
    def __len__(self, acc=0):
        return self.modname().__len__(acc+1)


class _RootQName(QualifiedName):
    """Names need a root since they are left recursive."""
    def __new__(cls, rootname):
        if not rootname: internal_error("Empty names are not allowed.")
        return tuple.__new__(cls, [rootname])
    def name(self): return self[0]
    modname = name
    def qname(self, sep='/', root=None):
        return root if root else ''
    asmodule = qname
    def generated(self): return True
    def pathwalk(self): return iter([])
    def __len__(self, acc=0):
        return acc


class Namespace(MutableMapping):
    def __init__(self, qname, father):
        """ A namespace have one father namespace. """
        self.father = father
        self.qname = qname
        self.idents = dict()
        self.gen_last_num = dict()
        self.root = father.root

    def qualify(self, name):
        """ Book the name in the namespace and return the qualified name.
        @raise ExistingIdent when name is already defined in this namespace.
        """
        try: self[name] #Verify it is not booked
        except KeyError: self[name] = None #Book it
        else: raise ExistingIdent(name)
        return QualifiedName(self.qname, name, False)

    def generate(self, name):
        """ Tries to register a name, if it already exists append a
        fresh number to it.
        """
        #The generation happens a lots with the same name
        try: n = self.gen_last_num[name]
        except KeyError: n = 0
        while True: #Loop until an undefined name is found.
            n = n+1
            iname = "{name}_{n}".format(name=name, n=n)
            try:
                self.idents[iname]
            except KeyError:
                self.gen_last_num[name] = n
                self.idents[name] = None #Book it
                return QualifiedName(self.qname, iname, True)

    def resolve(self, source_qname, generated=False):
        """ Tries to qualify source_qname to the nearest definition.
        Contrary to internal ast qname, source_qname may be partially qualified
        """
        #TODO: 4 for now we don't have source access to qualifiers (no syntax)
        try:
            self[source_qname]
            return QualifiedName(self.qname, source_qname, generated)
        except NonExistingIdent: pass
        # Only a root namespace stop the recursion.
        return self.father.resolve(source_qname, generated)

    def associate(self, qname, node):
        assert(qname.modname() == self.qname)
        self[qname.name()] = node

    def refresh(self, qname, node):
        """ Reassociating name with a new node. """
        assert(qname.modname() == self.qname)
        try: self[qname.name()] #verify it already exists
        except KeyError: internal_error("Refreshing an unknown ident.")
        self.associate(qname, node)

    def lookup_node(self, qname):
        """ Get the node from the namespace, if not defined, tries to get it
        from its father, if not found, raise NonExistingIdent(name)
        """
        if qname.modname() == self.qname: #shortcut for fast local name lookups
            return self[qname.name()]
        try:
            p = qname.pathwalk()
            return self.root._lookdown_node(next(p), p)
        except (KeyError, AttributeError): pass
        raise NonExistingIdent(qname)

    def _lookdown_node(self, name, pathwalkleft):
        nextname = next(pathwalkleft, False)
        if nextname:
            return self[name]._namespace._lookdown_node(nextname, pathwalkleft)
        else:
            return self[name]

    def push(self, qname):
        """ Return a new namespace with self as father.
        Rmq, the father isn't copied, a reference is kept, allowing any later
        modification of the father to be taken into account in its childs. """
        return Namespace(qname, self)

    def pop(self):
        """ Return the father namespace. """
        return self.father

    def __str__(self): return str(self.idents.keys())
    def __repr__(self): return str(self)
    #MutableMapping convention, behave like the idents dict
    def __getitem__(self, name):
        try :
            return self.idents[name]
        except KeyError:
            raise NonExistingIdent(name)
    def __setitem__(self, name, value): self.idents[name] = value
    def __delitem__(self, name): del self.idents[name]
    def __len__(self): return len(self.idents)
    def __iter__(self): return iter(self.idents)
    #No copy
    def __copy__(self): internal_error("Trying to copy an Ident.")
    def __deepcopy__(self, d): internal_error("Trying to deepcopy an Ident.")

class __RootNamespace(Namespace):
    def __init__(self, rootname):
        self.qname = _RootQName(rootname)
        self.idents = dict()
        self.gen_last_num = dict()
    def resolve(self, source_qname, generated=False):
        self[source_qname]
        return QualifiedName(self.qname, source_qname, generated)
    @property
    def root(self): return self
    @property
    def father(self): internal_error("A root namespace has no father.")
    def pop(self): internal_error("A root namespace can't be popped.")