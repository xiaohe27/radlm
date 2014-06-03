'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
import builtins
from collections import OrderedDict


class Bunch(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)

def str(o):
    """ Fix PEP 3140 -- str(container) should call str(item), not repr(item)
    PS: tuples are not affected by this 'bug'.
    """
    if isinstance(o, dict):
        l = ["{k} : {v}".format(k=str(k), v=str(v)) for (k, v) in o.items()]
        s = "{{{el}}}".format(el=', '.join(l))
        return s
    if isinstance(o, list):
        l = ["{k}".format(k=str(k)) for k in o]
        s = "[{el}]".format(el=', '.join(l))
        return s
    else:
        return builtins.str(o)
#TODO: 8 BucketDict allow (indices: [0], ...) access.
class BucketDict(OrderedDict):
    """ A bucket dictionary is simply a dict allowing to add mappings
        by storing them in lists:
    d = BucketDict()
    d.add('a', 1)
    assert(d == {'a': 1})
    d.add('a', 2)
    assert(d == {'a': [1, 2]})
    # keywords are executed before the initialization list
    d = BucketDict([('a', 1), ('b', 1)], a = 2)
    assert(d == {'a': [2, 1], 'b': 1})
    """
    def __init__(self, *args, **kargs):
        """ Keyword arguments will be added first """
        OrderedDict.__init__(self)
        if len(args) > 1:
            raise TypeError('expected at most 1 arguments, got %d' % len(args))
        self.update(kargs) #keywords are unique, just add them first
        if args:
            if isinstance(args, dict):
                for (k, v) in args.values():
                    self.add(k, v)
            else: #wainting for (key, val) tuples
                for p in args[0]:
                    if len(p) != 2:
                        raise TypeError("expected (key, value) tuples,"
                                        "got %s" % str(p))
                    self.add(*p)
    def add(self, key, value):
        """ value may be a list of new mappings or simply a value."""
        try:
            v = self[key]
        except KeyError: #new mapping
            self[key] = value
            return
        if isinstance(v, list):
            v += value if isinstance(value, list) else [value]
        else:
            v = [v] + value if isinstance(value, list) else [v, value]
            self[key] = value
    #def append (like in __init__)
    #def union (wait for a bucketdict : list are bucket

def write_file(filepath, filecontent):
    if __debug__:
        exists = ' (existing)' if filepath.exists() else ''
        print("file {filepath}{exists} :\n"
              "------------------------\n"
              "{filecontent}\n"
              "------------------------\n".format(**locals()))
    with filepath.open('w') as f:
        f.write(filecontent)

def ensure_dir(path):
    """{path} is expected to be a libpath.Path object"""
    if path.exists():
        if not path.is_dir():
            raise Exception("{} should be a directory".format(dir))
    else:
        path.mkdir()