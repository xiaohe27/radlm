'''
Created on Jul, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.radlr import infos, language
from radler.radlr.errors import warning, log_err
import pickle

version = "0.2"

class Objectfile:
    def __init__(self, root_node):
        self.root_node = root_node
        self.type_of_struct = infos.ros_type_of_struct
        self.lang_version = language.version
        self.obj_version = version
    def check_compat(self, filepath):
        if self.lang_version != language.version:
            log_err(lambda:
                "Object file {} has been created with language version {}"
                " (current version is {})."
                "".format(str(filepath), language.version, self.lang_version))
        if self.obj_version != version:
            log_err(lambda:
                "Object file {} is version {} (current version is {})."
                "".format(str(filepath), self.obj_version, version))
    def load_in_namespace(self, namespace, filepath):
        self.check_compat(filepath)
        n = self.root_node
        # reroot node
        n._namespace.father = namespace
        # add node to the root namespace
        try: #verify we are adding a new name
            namespace.lookup_node(n._qname)
            warning("Loading {} is shadowing a previous definition of {}"
                    "".format(str(filepath), str(n._qname)))
        except: pass #everything is ok
        namespace.associate(n._qname, n)


def save(node, filepath):
    """Save a node in a file (Path object)
    """
    o = Objectfile(node)
    with filepath.open('wb') as f:
        pickle.dump(o, f)

def load(filepath):
    """Load a file (Path object) in the global namespace infos.root_namespace.
    """
    with filepath.open('rb') as f:
        o = pickle.load(f)
        o.load_in_namespace(infos.root_namespace, filepath)