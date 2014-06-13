'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Used to ensure uniqueness of identifiers.
'''
from astutils.tools import str
from astutils.location import dummy_loc


class ExistingIdent(Exception): pass
class NonExistingIdent(AttributeError): pass
class AlreadyAttached(Exception): pass


class Ident:
    """ For now an ident is nothing more than a string.
    """
    def __init__(self, name, generated, location=None, node=None):
        self._name = name
        self._generated = generated
        if node and not location:
            self._location = node._location
        else:
            self._location = location if location else dummy_loc
        self._node = node

    def _attach(self, node):
        if self._node:
            raise AlreadyAttached()
        self._node = node
    def _reattach(self, node):
        self._node = node

    def __str__(self):
        if self._node:
            return "${self._name} -> {self._node}".format(self=self)
        else:
            return "${} -/>/".format(self._name)
    def __repr__(self):
        return self.__str__()
    #container convention, behave like node
    def __len__(self):
        return len(self._node)
    def __getitem__(self, key):
        return self._node[key]
    def __iter__(self):
        return iter(self._node)
    def __getattr__(self, attr):
        return getattr(self._node, attr)
    def __copy__(self):
        raise Exception("Trying to copy an Ident.")
    def __deepcopy__(self, d):
        raise Exception("Trying to deepcopy an Ident.")


class Namespace:
    def __init__(self, father=None):
        """ A namespace may have one father namespace.
        """
        self.father = father
        self.idents = dict()
        self.gen_last_num = dict()

    def register(self, name, node):
        """ Before registering the name in the namespace,
        check that name is not already defined : may raise ExistingIdent(name)
        """
        try:
            self.get_node(name)
        except NonExistingIdent:
            self.idents[name] = node
        else:
            raise ExistingIdent(name)

    def refresh(self, name, node):
        #verify it already exists
        try: self.idents[name]
        except KeyError: raise Exception("Refreshing an unknown ident.")
        self.idents[name] = node

#     def ident_of_source(self, name, node):
#         """ Before registering the name in the namespace,
#         check that name is not already defined : may raise ExistingIdent(name)
#         """
#         self.register(name, node)
#         return Ident(name, False, node=node)

    def gen_fresh(self, name):
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
                self.get_node(iname)
            except NonExistingIdent:
                self.gen_last_num[name] = n
                return iname

    def get_node(self, name):
        """ Get the node from the namespace, if not defined, tries to get it
        from its father, if not found, raise NonExistingIdent(name)
        """
        try:
            return self.idents[name]
        except KeyError:
            pass
        if not self.father:
            raise NonExistingIdent(name)
        return self.father.get_node(name)


    def push(self):
        """ Return a new namespace with self as father.
        Rmq, the father isn't copied, a reference is kept, allowing any later
        modification of the father to be taken into account in its childs.
        """
        return Namespace(self)

    def pop(self):
        """ Return the father namespace.
        """
        return self.father

    def __str__(self):
        return str(self.idents.keys())
    def __repr__(self):
        return str(self)

    #container convention, behave like the idents dict
    def __len__(self):
        return len(self.idents)
    def __getitem__(self, name):
        return self.get_ident(name)
    def __iter__(self):
        return iter(self.idents)