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
    def __init__(self, name, generated, location):
        self._node = None
        self._name = name
        self._generated = generated
        self._location = location
#     @property
#     def _val(self):
#         return self._node._val

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
            return "${}".format(self._name)
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


class Namespace:
    def __init__(self, father=None):
        """ A namespace may have one father namespace.
        """
        self.father = father
        self.idents = dict()
        self.gen_ident_last_num = dict()

    def ident_of_source(self, name, location):
        """ Before registering the name in the namespace,
        check that name is not already defined : may raise ExistingIdent(name)
        """
        try:
            self.get_ident(name)
        except NonExistingIdent:
            i = Ident(name, False, location)
            self.idents[name] = i
            return i
        else:
            raise ExistingIdent(name)

    def get_ident(self, name):
        """ Get the ident from the namespace, if not defined, tries to get it
        from its father, if not found, raise NonExistingIdent(name)
        """
        try:
            return self.idents[name]
        except KeyError:
            pass
        if not self.father:
            raise NonExistingIdent(name)
        return self.father.get_ident(name)

    def gen_ident(self, name, location=dummy_loc):
        """ Tries to generate an ident with name, if it already exists append a
        fresh number to it.
        """
        #The generation of ident happens a lots with the same name
        try:
            n = self.gen_ident_last_num[name]
        except KeyError:
            n = 0
        while True: #Loop until an undefined ident is found.
            n = n+1
            iname = "{name}_{n}".format(name=name, n=n)
            try:
                self.get_ident(iname)
            except NonExistingIdent:
                i = Ident(iname, True, location)
                self.idents[iname] = i
                self.gen_ident_last_num[name] = n
                return i

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